/*
 * april_tag_detector.hpp
 *
 * Uses functions from feature_detection node written by TSandy
 * (https://bitbucket.org/adrlab/feature_extraction)
 *
 *  Created on: Jun 2, 2017
 *      Author: Manuel Lussi <mlussi@ethz.ch>
 */

#ifndef APRIL_TAG_DETECTOR_HPP_
#define APRIL_TAG_DETECTOR_HPP_


#include "confusion/modules/apriltag/external/apriltag.h"
#include "confusion/modules/apriltag/external/tag36h11.h"
#include "confusion/modules/apriltag/external/tag36h10.h"
#include "confusion/modules/apriltag/external/tag36artoolkit.h"
#include "confusion/modules/apriltag/external/tag25h9.h"
#include "confusion/modules/apriltag/external/tag25h7.h"

#include <iostream>
#include <vector>
#include <string>
#include <Eigen/Core>

#include "opencv2/opencv.hpp"
#include "confusion/TagMsg.h"
#include "confusion/TagArray.h"

struct TagDetection {
	int id;
	double corners[4][2];
	int image_seq;

	TagDetection(): id(-1), image_seq(-1) { }

	TagDetection(const confusion::TagMsg& tagMsg) {
		id = (int)(tagMsg.id);
		for (int i=0; i<4; ++i) {
			corners[i][0] = tagMsg.corners[i].x;
			corners[i][1] = tagMsg.corners[i].y;
		}
	}
	TagDetection(const apriltag_detection_t& det, int seq_): image_seq(seq_) {
		id = det.id;
        for (int i=0; i<4; ++i) {
        	corners[i][0] = det.p[i][0];
        	corners[i][1] = det.p[i][1];
        }
	}
};

void convertTagDetections(zarray_t* detections, std::vector<TagDetection>& tagDetections, int seq = -1) {
	for (int i=0; i < zarray_size(detections); i++) {
		apriltag_detection_t* det;
        zarray_get(detections, i, &det);

        TagDetection td(*det, seq);
        tagDetections.push_back(td);
	}
}

//This is copied from the external version used in rcars because it was easier to understand
Eigen::Matrix4d getRelativeTransform(TagDetection td, double tag_size,
		double fx, double fy, double cx, double cy) {
	std::vector<cv::Point3d> objPts;
	std::vector<cv::Point2d> imgPts;
	double s = tag_size/2.;
	objPts.push_back(cv::Point3d(-s,-s, 0)); // bottom left
	objPts.push_back(cv::Point3d( s,-s, 0)); // bottom right
	objPts.push_back(cv::Point3d( s, s, 0)); // top right
	objPts.push_back(cv::Point3d(-s, s, 0)); // top left

	imgPts.push_back(cv::Point2d(td.corners[0][0], td.corners[0][1]));
	imgPts.push_back(cv::Point2d(td.corners[1][0], td.corners[1][1]));
	imgPts.push_back(cv::Point2d(td.corners[2][0], td.corners[2][1]));
	imgPts.push_back(cv::Point2d(td.corners[3][0], td.corners[3][1]));

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

//Publish a tagArray message from the TagDetections
confusion::TagArray getTagArrayMessage(ros::Time t, std::vector<TagDetection> td) {
	//publish where the corners are and (if cam infos available) and estimation of the tag pose wrt to the cam
	confusion::TagMsg tag;
	confusion::TagArray tags;
	tags.header.stamp = t;

	for (size_t i=0; i<td.size(); ++i) {
		// set the tag id in the message
		tag.id = td[i].id;

		// Fill in the corner points
		for (int j=0; j<4; ++j) {
			tag.corners[j].x = td[i].corners[j][0];
			tag.corners[j].y = td[i].corners[j][1];
			tag.corners[j].z = 0;
		}

//		//calculate pose wrt to camera if available
//		if(cam_info_available) {
//			Eigen::Matrix4d Tmat_c_t = getRelativeTransform(td[i], base_tag_size, cam_info.P[0], cam_info.P[5], cam_info.P[2], cam_info.P[6]);
//
//			Eigen::Matrix<double,3,1> trans = Tmat_c_t.template block<3,1>(0,3) / Tmat_c_t(3,3);
//			Eigen::Matrix<double,3,3> rotMat = Tmat_c_t.block(0,0,3,3);
//			Eigen::Quaternion<double> rot = Eigen::Quaternion<double>(rotMat);
//
//			// copy the translation
//			tag.pose.position.x = trans.x();
//			tag.pose.position.y = trans.y();
//			tag.pose.position.z = trans.z();
//
//			// copy the orientation
//			tag.pose.orientation.x = rot.x();
//			tag.pose.orientation.y = rot.y();
//			tag.pose.orientation.z = rot.z();
//			tag.pose.orientation.w = rot.w();
//		}

		tags.tags.push_back(tag);
	}

	return tags;
}

class AprilTagDetector {
public:
	AprilTagDetector(const std::string& tagFamily, int numThreads);
	~AprilTagDetector();
	void detectTags(const cv::Mat& img, std::vector<TagDetection>& tagDetections, int seq = -1);
	void drawTags(cv::Mat& img, const std::vector<TagDetection>& tagDetections);

	apriltag_detector_t* td;
	apriltag_family_t* tf;
};


#endif /* APRIL_TAG_DETECTOR_HPP_ */
