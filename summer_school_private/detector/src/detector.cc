#include "detector/detector.h"

DEFINE_string(yolo_cfg, "", "Path to yolo network configuration file.");
DEFINE_string(yolo_weights, "", "Path to yolo network weights file.");
DEFINE_bool(show_detections, false, "Show debug images with the detections.");
DEFINE_string(ncamera_calibration, "camera.yaml", "Camera calibration yaml.");
DEFINE_string(camera_topic_suffic, "/image_raw", "Camera topic suffic.");

int main(int argc, char** argv) {
  // Initialize logging and parse gflags.
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InstallFailureSignalHandler();
  FLAGS_alsologtostderr = true;
  FLAGS_colorlogtostderr = true;

  // Check necessary parameters are given.
  const std::string& yolo_cfg_path = FLAGS_yolo_cfg;
  const std::string& yolo_weights_path = FLAGS_yolo_weights;

  CHECK(!yolo_cfg_path.empty())
      << "Path to YOLO cfg file must be provided (using --yolo_cfg).";
  CHECK(!yolo_weights_path.empty())
      << "Path to YOLO weights file must be provided (using --yolo_weights).";

  // Initialize aslam camera
  aslam::NCamera::Ptr n_camera =
      aslam::NCamera::loadFromYaml(FLAGS_ncamera_calibration);
  CHECK(n_camera) << "Could not load the camera calibration from: \'"
                  << FLAGS_ncamera_calibration << "\'";

  if (n_camera->getNumCameras() > 1u) {
    LOG(WARNING) << "NCamera yaml contains more than one camera. Only the "
                 << "first camera will be used for detection.";
  }

  // Initialize object detection network.
  network net;

  // Read network configuration.
  char* yolo_cfg_path_c = new char[yolo_cfg_path.size() + 1];
  strcpy(yolo_cfg_path_c, yolo_cfg_path.c_str());
  net = parse_network_cfg_custom(yolo_cfg_path_c, 1, 1);

  // Read network layer weights.
  char* yolo_weights_path_c = new char[yolo_weights_path.size() + 1];
  strcpy(yolo_weights_path_c, yolo_weights_path.c_str());
  load_weights(&net, yolo_weights_path_c);

  // Fuse layers to speedup inference
  fuse_conv_batchnorm(net);

  // Initialize ROS
  ros::init(argc, argv, "detector");
  ros::NodeHandle nh;

  // Show debug images with the detections.
  ros::Publisher marker_pub =
      nh.advertise<visualization_msgs::Marker>("detections", 10);

  if (FLAGS_show_detections) {
    cv::namedWindow("detections");
    cv::startWindowThread();
  }

  // Listen to image topic
  auto imageCallback = std::bind(
      processImage, std::placeholders::_1, n_camera, &net, &marker_pub);
  image_transport::ImageTransport image_transporter(nh);
  std::string camera_topic =
      n_camera->getCamera(0u).getLabel() + FLAGS_camera_topic_suffic;
  image_transport::Subscriber image_subscriber =
      image_transporter.subscribe(camera_topic, 1, imageCallback);

  // Ros spin waiting for image messages
  ros::spin();

  // Clean up.
  if (FLAGS_show_detections) {
    cv::destroyWindow("detections");
  }

  return 0;
}

void processImage(
    const sensor_msgs::ImageConstPtr& msg, const aslam::NCamera::Ptr& n_camera,
    network* net, ros::Publisher* marker_pub) {
  const aslam::Camera& camera = n_camera->getCamera(0u);

  cv::Mat frame;
  try {
    frame = cv_bridge::toCvShare(msg, "mono8")->image;
  } catch (cv_bridge::Exception& e) {
    LOG(ERROR) << "Could not convert from '" << msg->encoding << "' to 'bgr8'.";
  }

  // Flip image if necessary.
  // cv::flip(frame, frame, -1);

  // Convert image to darknet format.
  image im = mat_to_image(frame);
  im = resize_image(im, net->w, net->h);

  // Perform a forward pass of the object detector.
  layer l = net->layers[net->n - 1];
  network_predict(*net, im.data);

  // Get network predicted bounding boxes
  int nboxes = 0;
  float thresh = 0.25f;
  float nms = 0.45f;
  detection* dets = get_network_boxes(
      net, frame.cols, frame.rows, thresh, 0.5f, 0, 1, &nboxes, 0);

  // Apply non-maxima supression to remove redudant or noisy detections.
  do_nms_sort(dets, nboxes, l.classes, nms);

  // Free darknet image
  free_image(im);

  // Publish point markers in rviz corresponding to the detections.
  visualization_msgs::Marker points;
  if (FLAGS_show_detections) {
    points.id = 0;
    points.header.frame_id = "/cam";
    points.header.stamp = msg->header.stamp;
    points.action = visualization_msgs::Marker::ADD;
    points.type = visualization_msgs::Marker::POINTS;
    points.ns = "points";
    points.pose.orientation.w = 1.0;
    points.scale.x = 0.2;
    points.scale.y = 0.2;
    points.color.g = 1.0;
    points.color.a = 1.0;
  }

  // Iterate over all the detections which are above the detection threshold.
  for (int i = 0; i < nboxes; i++) {
    if (dets[i].objectness < thresh) {
      continue;
    }

    int cls = -1;
    float prob = thresh;
    for (int j = 0; j < l.classes; j++) {
      if (dets[i].prob[j] > prob) {
        prob = dets[i].prob[j];
        cls = j;
      }
    }

    // Some class detection probability exceeeds the minimum threshold
    if (cls != -1) {
      // Compute object bearing vector using bounding box center
      Eigen::Vector3d bearing_vector;
      double bbox_center_x = dets[i].bbox.x * frame.cols;
      double bbox_center_y = dets[i].bbox.y * frame.rows;
      bool success = camera.backProject3(
          Eigen::Vector2d(bbox_center_x, bbox_center_y), &bearing_vector);

      if (!success) {
        LOG(WARNING) << "Failed to obtain bearing vector from detection.";
        continue;
      }

      if (FLAGS_show_detections) {
        // Create rviz markers.
        geometry_msgs::Point p;
        p.x = bearing_vector(0);
        p.y = bearing_vector(1);
        p.z = bearing_vector(2);
        points.points.push_back(p);

        // Draw detected bounding boxes
        LOG(INFO) << "Found object of class " << cls << " (" << prob * 100
                  << "\%)" << std::endl;

        int x_min = (dets[i].bbox.x - dets[i].bbox.w / 2) * frame.cols;
        int y_min = (dets[i].bbox.y - dets[i].bbox.h / 2) * frame.rows;
        int width = dets[i].bbox.w * frame.cols;
        int height = dets[i].bbox.h * frame.rows;

        if (x_min < 0) {
          x_min = 0;
        }

        if (y_min < 0) {
          y_min = 0;
        }

        if (x_min + width > frame.cols - 1) {
          width = frame.cols - 1 - x_min;
        }

        if (y_min + height > frame.rows - 1) {
          height = frame.rows - 1 - y_min;
        }

        cv::rectangle(
            frame, cv::Point(x_min, y_min),
            cv::Point(x_min + width, y_min + height), CV_RGB(255, 255, 255));
      }
    }
  }

  free_detections(dets, nboxes);

  if (FLAGS_show_detections) {
    // Publish rviz points.
    marker_pub->publish(points);

    // Update opencv image viewer.
    cv::imshow("detections", frame);
  }
}
