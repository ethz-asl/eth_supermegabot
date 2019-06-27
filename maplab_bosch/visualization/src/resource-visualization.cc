#include "visualization/resource-visualization.h"

#include <chrono>
#include <thread>

#include <Eigen/Dense>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <glog/logging.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>

DEFINE_bool(
    vis_pointcloud_accumulated_before_publishing, false,
    "Accumulate the point cloud resources before publishing them all at once. "
    "Use --vis_pointcloud_visualize_every_nth to sparsify the pointcloud, "
    "otherwise it will grow too large.");
DEFINE_bool(
    vis_pointcloud_publish_in_sensor_frame_with_tf, false,
    "If enabled, the point clouds are published in the sensor frame and the "
    "transformation between sensor frame and world frame is published to the "
    "tf tree. If disabled, the pointclouds are published in the world frame. "
    "This flag is not compatible with "
    "--vis_pointcloud_accumulated_before_publishing.");
DEFINE_bool(
    vis_pointcloud_use_distorted_camera_to_reproject_depth_maps, false,
    "If enabled, the distorted depth camera is used to reproject the depth "
    "maps to point cloud. This assumes that the depth map has been created "
    "using a distorted camera.");

DEFINE_double(
    vis_pointcloud_publishing_real_time_factor, 1.0,
    "Realtime factor of pointcloud publishing. Needs to be greater than 0");
DEFINE_int32(
    vis_pointcloud_visualize_every_nth, 1,
    "Visualize only every nth pointcloud.");

DEFINE_double(
    vis_resource_visualization_frequency, 8,
    "Frequency of the image resources visualization in Hz.");

namespace visualization {

bool visualizeCvMatResources(
    const vi_map::VIMap& map, backend::ResourceType type) {
  CHECK_GT(FLAGS_vis_resource_visualization_frequency, 0.0);

  VLOG(1) << "INFO: Visualization will run at "
          << FLAGS_vis_resource_visualization_frequency
          << " Hz. Hold any key to speed up.";
  pose_graph::VertexIdList vertex_ids;
  map.getAllVertexIds(&vertex_ids);
  common::ProgressBar progress_bar(vertex_ids.size());
  vi_map::MissionVertexIdList mission_to_vertex_ids;
  map.getVertexIdsByMission(&mission_to_vertex_ids);

  if (mission_to_vertex_ids.empty()) {
    VLOG(1) << "No missions found!";
    return true;
  }

  int mission_num = 0;
  for (const vi_map::MissionVertexIdPair& mission_vertex_id_pair :
       mission_to_vertex_ids) {
    VLOG(1) << "## Mission " << (mission_num + 1) << " of "
            << mission_to_vertex_ids.size() << " ##";

    const aslam::NCamera& ncamera = map.getSensorManager().getNCameraForMission(
        mission_vertex_id_pair.first);
    std::vector<std::string> cv_window_names;
    std::unordered_set<std::string> cv_active_window_names;
    getOpenCvWindowsForNCamera(ncamera, &cv_window_names);

    for (const pose_graph::VertexId& vertex_id :
         mission_vertex_id_pair.second) {
      const vi_map::Vertex& vertex = map.getVertex(vertex_id);
      const aslam::VisualNFrame& n_frame = vertex.getVisualNFrame();
      for (uint idx = 0u; idx < n_frame.getNumFrames(); ++idx) {
        cv::Mat image_resource;
        if (map.getFrameResource(vertex, idx, type, &image_resource)) {
          cv_active_window_names.insert(cv_window_names.at(idx));
          // Rescale depth and disparity maps to make them nice to look at.
          cv::Mat rescaled_image_resource;
          switch (type) {
            case backend::ResourceType::kRawImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kUndistortedImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kRawColorImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kUndistortedColorImage:
              rescaled_image_resource = image_resource;
              break;
            case backend::ResourceType::kRawDepthMap:
            // Fall through intended.
            case backend::ResourceType::kOptimizedDepthMap:
              cv::normalize(
                  image_resource, rescaled_image_resource, 0, 255,
                  cv::NORM_MINMAX, CV_8U);
              break;
            case backend::ResourceType::kDisparityMap:
              double min, max;
              cv::minMaxLoc(image_resource, &min, &max);
              image_resource.convertTo(
                  rescaled_image_resource, CV_8U, -255.0 / max, 255);
              break;
            default:
              LOG(FATAL) << "Non-compatible resource type found !";
          }
          cv::imshow(cv_window_names.at(idx), rescaled_image_resource);
        } else {
          if (cv_active_window_names.find(cv_window_names.at(idx)) ==
              cv_active_window_names.end()) {
            cv::destroyWindow(cv_window_names.at(idx));
          }
        }
      }
      cv::waitKey(1000.0 / FLAGS_vis_resource_visualization_frequency);
      progress_bar.increment();
    }
    destroyAllWindows(cv_window_names);
    ++mission_num;
  }
  return true;
}

void getOpenCvWindowsForNCamera(
    const aslam::NCamera& n_camera, std::vector<std::string>* named_windows) {
  CHECK_NOTNULL(named_windows);

  for (uint i = 0u; i < n_camera.getNumCameras(); ++i) {
    const aslam::Camera& camera = n_camera.getCamera(i);
    named_windows->push_back(
        "camera_" + std::to_string(i) + "_" + camera.getId().hexString());
    cv::namedWindow(named_windows->at(i), cv::WINDOW_NORMAL);
  }
  CHECK_EQ(named_windows->size(), n_camera.getNumCameras());
}

void destroyAllWindows(const std::vector<std::string>& windows_names) {
  for (uint i = 0; i < windows_names.size(); ++i) {
    cv::destroyWindow(windows_names.at(i));
    cv::waitKey(1);
  }
}
}  // namespace visualization
