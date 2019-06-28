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

#ifndef CONFUSION_TEST_APRILTAG_UTILS_H_
#define CONFUSION_TEST_APRILTAG_UTILS_H_

#include <vector>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>

//Minimal utilities for using fiducial measurements are included here to support
//the TagMeas class and for unit testing and the example

namespace confusion {

//This is copied from the external implementation
Eigen::Matrix4d getRelativeTransform(const std::array<std::array<double,2>,4> corners, double tag_size,
		double fx, double fy, double cx, double cy) {
	std::vector<cv::Point3d> objPts;
	std::vector<cv::Point2d> imgPts;
	double s = tag_size/2.;
	objPts.push_back(cv::Point3d(-s,-s, 0)); // bottom left
	objPts.push_back(cv::Point3d( s,-s, 0)); // bottom right
	objPts.push_back(cv::Point3d( s, s, 0)); // top right
	objPts.push_back(cv::Point3d(-s, s, 0)); // top left

	imgPts.push_back(cv::Point2d(corners[0][0], corners[0][1]));
	imgPts.push_back(cv::Point2d(corners[1][0], corners[1][1]));
	imgPts.push_back(cv::Point2d(corners[2][0], corners[2][1]));
	imgPts.push_back(cv::Point2d(corners[3][0], corners[3][1]));

	cv::Mat rvec, tvec;
	cv::Matx33d cameraMatrix(
						   fx, 0, cx,
						   0, fy, cy,
						   0,  0,  1);
	cv::Vec4d distParam(0,0,0,0); // all 0?

	cv::solvePnP(objPts, imgPts, cameraMatrix, distParam, rvec, tvec);
	cv::Matx33d r;
	cv::Rodrigues(rvec, r);
	Eigen::Matrix3d wRo;
	wRo << r(0,0), r(0,1), r(0,2), r(1,0), r(1,1), r(1,2), r(2,0), r(2,1), r(2,2);

	Eigen::Matrix4d T;
	T.topLeftCorner(3,3) = wRo;
	T.col(3).head(3) << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2);
	T.row(3) << 0,0,0,1;

	return T;
}

} //namespace confusion

#endif /* CONFUSION_TEST_APRILTAG_UTILS_H_ */
