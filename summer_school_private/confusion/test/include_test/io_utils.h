/*
 * Copyright 2018 Timothy Sandy
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef INCLUDE_HYA_EST_IO_UTILS_H_
#define INCLUDE_HYA_EST_IO_UTILS_H_

#include <iostream>
#include <fstream>
#include <vector>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>

#include "confusion/utilities/imu_utils.h"
#include "confusion/models/TagMeas.h"
#include "confusion/ImuState.h"
#include "confusion/TagArray.h"


//todo Deque or vector for measurements?
//This imports imu, tag and igps data from one data file. Used on the 230617 datasets.
bool importDataFromBag(std::string bag_in,
		std::deque<std::shared_ptr<confusion::ImuMeas>>& imuMeasVec, std::vector<std::shared_ptr<confusion::TagMeas>>& tagMeasVec,
		confusion::ImuCalibration& imuCalibration, const confusion::TagMeasCalibration& tagMeasCalibration,
		Eigen::Vector2d& gravity_rot, Eigen::Matrix<double,3,4>& projMat,
		confusion::Pose<double>& T_c_i) {
	//Open the bag file
	rosbag::Bag bag(bag_in);

	//First check if there is a camera_info message to pull in the camera calibration
	std::vector<std::string> caminfo_topic_vec;
	caminfo_topic_vec.push_back("/camera/camera_info");
	caminfo_topic_vec.push_back("camera/camera_info");
	rosbag::View caminfo_view(bag, rosbag::TopicQuery(caminfo_topic_vec));
	for (rosbag::MessageInstance const m: caminfo_view) {
		sensor_msgs::CameraInfo::ConstPtr camInfoMsg = m.instantiate<sensor_msgs::CameraInfo>();

		if (camInfoMsg != NULL) {
			projMat.setZero();
			projMat(0,0) = camInfoMsg->P[0];
			projMat(0,2) = camInfoMsg->P[2];
			projMat(1,1) = camInfoMsg->P[5];
			projMat(1,2) = camInfoMsg->P[6];
			projMat(2,2) = 1.0;

			std::cout << "projMat set while importing data:\n" << projMat << std::endl;
			break;
		}
	}

	//Import all measurements
	std::vector<std::string> topic_vec;
	topic_vec.push_back("/imu");
	topic_vec.push_back("/tags");
	topic_vec.push_back("imu");
	topic_vec.push_back("tags");

	rosbag::View imu_view(bag, rosbag::TopicQuery(topic_vec));

	int count = 0;
	for (rosbag::MessageInstance const m: imu_view) {
//		hya_est::ImuMsg::ConstPtr imuMsg = m.instantiate<hya_est::ImuMsg>();
//
//		if (imuMsg != NULL) {
//			//Read IMU message and add it to the queue.
//			double t_imu_source = imuMsg->header.stamp.toSec();
//
//			Eigen::Vector3d a(imuMsg->linear_acceleration.x,
//							  imuMsg->linear_acceleration.y,
//							  imuMsg->linear_acceleration.z);
//			Eigen::Vector3d w(imuMsg->angular_velocity.x,
//							  imuMsg->angular_velocity.y,
//							  imuMsg->angular_velocity.z);
//
//			ImuMeas meas(t_imu_source, imuMsg->seq, a, w, imuMsg->syncOutFlag);
//			imuMeasVec.push_back(meas);
//
//			continue;
//		}

		sensor_msgs::Imu::ConstPtr imuMsg2 = m.instantiate<sensor_msgs::Imu>();

		if (imuMsg2 != NULL) {
			//Read IMU message and add it to the queue.
			double t_imu_source = imuMsg2->header.stamp.toSec();

			Eigen::Vector3d a(imuMsg2->linear_acceleration.x,
							  imuMsg2->linear_acceleration.y,
							  imuMsg2->linear_acceleration.z);
			Eigen::Vector3d w(imuMsg2->angular_velocity.x,
							  imuMsg2->angular_velocity.y,
							  imuMsg2->angular_velocity.z);

//			ImuMeas meas(t_imu_source, a, w, imuCalibration, gravity_rot, 0);
			imuMeasVec.push_back(std::make_shared<confusion::ImuMeas>(t_imu_source, a, w, &imuCalibration, &gravity_rot, IMU));

			continue;
		}

		confusion::TagArray::ConstPtr tagMsg = m.instantiate<confusion::TagArray>();

		if (tagMsg != NULL) {
			//Add the pose measurements to the filter
			for (int i=0; i<(tagMsg->tags).size(); ++i) {
				std::array<std::array<double,2>,4> corners;
				corners[3][0] = tagMsg->tags[i].corners[0].x;
				corners[3][1] = tagMsg->tags[i].corners[0].y;
				corners[2][0] = tagMsg->tags[i].corners[1].x;
				corners[2][1] = tagMsg->tags[i].corners[1].y;
				corners[0][0] = tagMsg->tags[i].corners[2].x;
				corners[0][1] = tagMsg->tags[i].corners[2].y;
				corners[1][0] = tagMsg->tags[i].corners[3].x;
				corners[1][1] = tagMsg->tags[i].corners[3].y;

//				TagMeas tagMeas(tagMsg->header.stamp.toSec(), td, T_c_i, tagMeasCalibration, 0);
                std::string referenceFrameName = "tag" + std::to_string(tagMsg->tags[i].id);
				tagMeasVec.push_back(std::make_shared<confusion::TagMeas>(tagMsg->header.stamp.toSec(), referenceFrameName, T_c_i, corners, tagMeasCalibration, TAG));
			}

			continue;
		}
	}

	if (!imuMeasVec.empty())
		std::cout << "Imported " << imuMeasVec.size() << " imu measurements between " <<
				imuMeasVec.front()->t()-imuMeasVec.front()->t() << " and " << imuMeasVec.back()->t()-imuMeasVec.front()->t() << std::endl;
	else
		std::cout << "No imu measurements found???" << std::endl;

	if (!tagMeasVec.empty())
		std::cout << "Imported " << tagMeasVec.size() << " pose measurements between " <<
				tagMeasVec.front()->t()-imuMeasVec.front()->t() << " and " << tagMeasVec.back()->t()-imuMeasVec.front()->t() << std::endl;
	else
		std::cout << "No pose measurements found???" << std::endl;

	return true;
}

#endif /* INCLUDE_HYA_EST_IO_UTILS_H_ */
