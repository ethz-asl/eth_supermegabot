#ifndef DETECTOR_H_
#define DETECTOR_H_

#include <cstring>
#include <iostream>

#include <aslam/cameras/camera.h>
#include <aslam/cameras/ncamera.h>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

// Darknet libraries.
#include <darknet.h>
#include <parser.h>

#include "detector/utils.h"

void processImage(
    const sensor_msgs::ImageConstPtr& msg, const aslam::NCamera::Ptr& n_camera,
    network* net, ros::Publisher *marker_pub);

#endif  // DETECTOR_H_
