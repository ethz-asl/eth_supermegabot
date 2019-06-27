#ifndef VISUALIZATION_RESOURCE_VISUALIZATION_H_
#define VISUALIZATION_RESOURCE_VISUALIZATION_H_

#include <algorithm>
#include <chrono>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <aslam/cameras/camera-factory.h>
#include <aslam/cameras/camera-pinhole.h>
#include <aslam/cameras/camera-unified-projection.h>
#include <aslam/cameras/distortion-null.h>
#include <aslam/cameras/ncamera.h>
#include <aslam/common/time.h>
#include <aslam/frames/visual-frame.h>
#include <aslam/frames/visual-nframe.h>
#include <cv_bridge/cv_bridge.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <landmark-triangulation/pose-interpolator.h>
#include <map-resources/resource-conversion.h>
#include <maplab-common/progress-bar.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>
#include <vi-map/sensor-manager-utils.h>
#include <vi-map/vertex.h>
#include <vi-map/vi-map.h>
#include <visualization/common-rviz-visualization.h>

DECLARE_bool(vis_pointcloud_accumulated_before_publishing);

DECLARE_bool(vis_pointcloud_publish_in_sensor_frame_with_tf);

DECLARE_bool(vis_pointcloud_use_distorted_camera_to_reproject_depth_maps);

DECLARE_double(vis_pointcloud_publishing_real_time_factor);

DECLARE_int32(vis_pointcloud_visualize_every_nth);

namespace visualization {

bool visualizeCvMatResources(
    const vi_map::VIMap& map, backend::ResourceType type);
typedef std::unordered_map<aslam::CameraId, aslam::Camera::Ptr> CameraCache;

template <typename SensorOrCameraId>
void visualizePointCloud(
    const backend::ResourceType type, const vi_map::MissionIdList& mission_ids,
    const vi_map::VIMap& map);

void getOpenCvWindowsForNCamera(
    const aslam::NCamera& n_camera, std::vector<std::string>* named_windows);

void destroyAllWindows(const std::vector<std::string>& windows_names);

template <typename SensorOrCameraId>
void visualizePointCloud(
    const backend::ResourceType input_resource_type,
    const vi_map::MissionIdList& mission_ids, const vi_map::VIMap& vi_map) {
  CHECK(!mission_ids.empty());
  CHECK_GT(FLAGS_vis_pointcloud_publishing_real_time_factor, 0.0);
  CHECK(
      !(FLAGS_vis_pointcloud_accumulated_before_publishing &&
        FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf));

  resources::PointCloud accumulated_point_cloud;

  for (const vi_map::MissionId& mission_id : mission_ids) {
    const vi_map::VIMission& mission = vi_map.getMission(mission_id);

    const aslam::Transformation& T_G_M =
        vi_map.getMissionBaseFrameForMission(mission_id).get_T_G_M();

    // Check if there is IMU data to interpolate the optional sensor poses.
    landmark_triangulation::VertexToTimeStampMap vertex_to_time_map;
    int64_t min_timestamp_ns;
    int64_t max_timestamp_ns;
    const landmark_triangulation::PoseInterpolator pose_interpolator;
    pose_interpolator.getVertexToTimeStampMap(
        vi_map, mission_id, &vertex_to_time_map, &min_timestamp_ns,
        &max_timestamp_ns);
    if (vertex_to_time_map.empty()) {
      VLOG(2) << "Couldn't find any IMU data to interpolate exact optional "
              << "sensor position in mission " << mission_id;
      continue;
    }

    LOG(INFO) << "All resources within this time range will be integrated: ["
              << min_timestamp_ns << "," << max_timestamp_ns << "]";

    // Retrieve sensor id to resource id mapping.
    typedef std::unordered_map<
        SensorOrCameraId, backend::OptionalSensorResources>
        SensorsToResourceMap;

    const SensorsToResourceMap* sensor_id_to_res_id_map;
    sensor_id_to_res_id_map =
        mission.getAllOptionalSensorResourceIdsOfType<SensorOrCameraId>(
            input_resource_type);

    if (sensor_id_to_res_id_map == nullptr) {
      continue;
    }
    VLOG(1) << "Found " << sensor_id_to_res_id_map->size()
            << " optional sensors with this depth type.";

    // Integrate them one sensor at a time.
    for (const typename SensorsToResourceMap::value_type& sensor_to_res_ids :
         *sensor_id_to_res_id_map) {
      const backend::OptionalSensorResources& resource_buffer =
          sensor_to_res_ids.second;

      const SensorOrCameraId& sensor_or_camera_id = sensor_to_res_ids.first;

      // Get transformation between reference (e.g. IMU) and sensor.
      aslam::Transformation T_I_S;
      vi_map.getSensorManager().getSensorOrCamera_T_R_S(
          sensor_or_camera_id, &T_I_S);

      // Retrieve the camera as well if this function is called using an
      // aslam::CameraId. This is needed for depth maps to reproject them.
      aslam::Camera::Ptr camera_ptr = vi_map::getOptionalCameraIfCameraId(
          vi_map.getSensorManager(), sensor_or_camera_id);

      if (!FLAGS_vis_pointcloud_use_distorted_camera_to_reproject_depth_maps &&
          camera_ptr) {
        aslam::Camera::Ptr camera_no_distortion;
        backend::createCameraWithoutDistortion(
            *camera_ptr, &camera_no_distortion);
        CHECK(camera_no_distortion);
        camera_ptr = camera_no_distortion;
      }

      const size_t num_resources = resource_buffer.size();
      VLOG(1) << "Sensor " << sensor_or_camera_id.shortHex() << " has "
              << num_resources << " such resources.";

      // Collect all timestamps that need to be interpolated.
      Eigen::Matrix<int64_t, 1, Eigen::Dynamic> resource_timestamps(
          num_resources);
      size_t idx = 0u;
      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer) {
        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position. To
        // keep this efficient, we simply replace timestamps outside the range
        // with the min or max. Since their transformation will not be used
        // later, that's fine.
        resource_timestamps[idx] = std::max(
            min_timestamp_ns,
            std::min(max_timestamp_ns, stamped_resource_id.first));

        ++idx;
      }

      // Interpolate poses at resource timestamp.
      aslam::TransformationVector poses_M_I;
      pose_interpolator.getPosesAtTime(
          vi_map, mission_id, resource_timestamps, &poses_M_I);

      CHECK_EQ(static_cast<int>(poses_M_I.size()), resource_timestamps.size());
      CHECK_EQ(poses_M_I.size(), resource_buffer.size());

      // Retrieve and integrate all resources.
      idx = 0u;
      common::ProgressBar tsdf_progress_bar(resource_buffer.size());
      for (const std::pair<int64_t, backend::ResourceId>& stamped_resource_id :
           resource_buffer) {
        tsdf_progress_bar.increment();

        if (idx % FLAGS_vis_pointcloud_visualize_every_nth != 0u) {
          ++idx;
          continue;
        }

        // We assume the frame of reference for the sensor system is the IMU
        // frame.
        const aslam::Transformation& T_M_I = poses_M_I[idx];
        const aslam::Transformation T_G_S = T_G_M * T_M_I * T_I_S;
        ++idx;

        const int64_t timestamp_ns = stamped_resource_id.first;
        int64_t diff_timestamp_to_next_resource_ns = 0;
        if (idx < num_resources) {
          diff_timestamp_to_next_resource_ns =
              resource_timestamps[idx] - timestamp_ns;
        }
        // If the resource timestamp does not lie within the min and max
        // timestamp of the vertices, we cannot interpolate the position.
        if (timestamp_ns < min_timestamp_ns ||
            timestamp_ns > max_timestamp_ns) {
          LOG(WARNING) << "The optional depth resource at " << timestamp_ns
                       << " is outside of the time range of the pose graph, "
                       << "skipping.";
          continue;
        }

        resources::PointCloud point_cloud;

        switch (input_resource_type) {
          case backend::ResourceType::kRawDepthMap:
          // Fall through intended.
          case backend::ResourceType::kOptimizedDepthMap: {
            CHECK(camera_ptr) << "For depth maps we should have "
                                 "retrieved a camera for reprojection, "
                                 "but the camera is not available!";
            const aslam::Camera& camera = *camera_ptr;

            cv::Mat depth_map;
            if (!vi_map.getOptionalSensorResource(
                    mission, input_resource_type, sensor_or_camera_id,
                    timestamp_ns, &depth_map)) {
              LOG(FATAL) << "Cannot retrieve optional depth map resources at "
                         << "timestamp " << timestamp_ns << "!";
            }

            // Check if there is a dedicated grayscale or color image for this
            // depth map.
            cv::Mat image;
            if (!vi_map.getOptionalSensorResource(
                    mission, backend::ResourceType::kImageForDepthMap,
                    sensor_or_camera_id, timestamp_ns, &depth_map)) {
              VLOG(3) << "Found depth map with intensity information "
                         "from the dedicated grayscale image.";
            } else if (!vi_map.getOptionalSensorResource(
                           mission,
                           backend::ResourceType::kColorImageForDepthMap,
                           sensor_or_camera_id, timestamp_ns, &depth_map)) {
              VLOG(3) << "Found depth map with RGB information "
                         "from the dedicated color image.";
            } else {
              VLOG(3)
                  << "Found depth map without any color/intensity information.";
            }

            CHECK(backend::convertDepthMapToPointCloud<resources::PointCloud>(
                depth_map, image, camera, &point_cloud));

            break;
          }
          case backend::ResourceType::kPointCloudXYZI:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZ:
          // Fall through intended.
          case backend::ResourceType::kPointCloudXYZRGBN: {
            // Check if a point cloud is available.
            if (!vi_map.getOptionalSensorResource(
                    mission, input_resource_type, sensor_or_camera_id,
                    timestamp_ns, &point_cloud)) {
              LOG(FATAL) << "Cannot retrieve optional point cloud resources at "
                         << "timestamp " << timestamp_ns << "!";
            }
            break;
          }
          default:
            LOG(FATAL) << "This depth type is not supported! type: "
                       << backend::ResourceTypeNames[static_cast<int>(
                              input_resource_type)];
        }

        if (!FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf) {
          point_cloud.applyTransformation(T_G_S);
        }

        VLOG(3) << "Found point cloud at timestamp " << timestamp_ns;

        if (!FLAGS_vis_pointcloud_accumulated_before_publishing &&
            !point_cloud.empty()) {
          sensor_msgs::PointCloud2 ros_point_cloud;
          backend::convertPointCloudType(point_cloud, &ros_point_cloud);

          ros_point_cloud.header.stamp =
              ros::Time(1e-9 * static_cast<double>(timestamp_ns));

          if (FLAGS_vis_pointcloud_publish_in_sensor_frame_with_tf) {
            const std::string kGenericSensorFrame = "sensor";
            ros_point_cloud.header.frame_id = kGenericSensorFrame;
            Eigen::Affine3d T_G_S_eigen;
            T_G_S_eigen.matrix() = T_G_S.getTransformationMatrix();
            tf::Transform T_G_S_tf;
            tf::transformEigenToTF(T_G_S_eigen, T_G_S_tf);
            static tf::TransformBroadcaster tf_broadcaster;
            tf_broadcaster.sendTransform(tf::StampedTransform(
                T_G_S_tf, ros_point_cloud.header.stamp, kDefaultMapFrame,
                kGenericSensorFrame));
          } else {
            ros_point_cloud.header.frame_id = kDefaultMapFrame;
          }

          const std::string kPointCloudTopic = "/point_cloud";
          RVizVisualizationSink::publish(kPointCloudTopic, ros_point_cloud);
          VLOG(2) << "Visualized point-cloud with " << point_cloud.size()
                  << " points on topic " << kPointCloudTopic;

          // Sleep for a bit to not publish the pointclouds too fast.
          int64_t sleep_time_ms = static_cast<int64_t>(
              (diff_timestamp_to_next_resource_ns * 1e-6) /
              FLAGS_vis_pointcloud_publishing_real_time_factor);
          std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));

        } else {
          accumulated_point_cloud.append(point_cloud);
        }
      }
    }
  }

  if (FLAGS_vis_pointcloud_accumulated_before_publishing &&
      !accumulated_point_cloud.empty()) {
    sensor_msgs::PointCloud2 ros_point_cloud;
    backend::convertPointCloudType(accumulated_point_cloud, &ros_point_cloud);

    const std::string kPointCloudTopic = "/point_cloud";
    ros_point_cloud.header.frame_id = kDefaultMapFrame;
    RVizVisualizationSink::publish(kPointCloudTopic, ros_point_cloud);
    VLOG(2) << "Visualized accumulated point-cloud with "
            << accumulated_point_cloud.size() << " points on topic "
            << kPointCloudTopic;
  }
}

}  // namespace visualization

#endif  // VISUALIZATION_RESOURCE_VISUALIZATION_H_
