/*
 * april_tag_detector.cpp
 *
 * Uses functions from feature_detection node written by TSandy
 * (https://bitbucket.org/adrlab/feature_extraction)
 *
 *
 *  Created on: Jun 2, 2017
 *      Author: Manuel Lussi <mlussi@ethz.ch>
 */

#include "april_tag_detector.hpp"


AprilTagDetector::AprilTagDetector(const std::string& tagFamily, int numThreads) {
	//Set the tag family
	if (tagFamily.compare("tag36h11") == 0)
        tf = tag36h11_create();
    else if (tagFamily.compare("tag36h10") == 0)
        tf = tag36h10_create();
    else if (tagFamily.compare("tag36artoolkit") == 0)
        tf = tag36artoolkit_create();
    else if (tagFamily.compare("tag25h9") == 0)
        tf = tag25h9_create();
    else if (tagFamily.compare("tag25h7") == 0)
        tf = tag25h7_create();
    else {
        printf("Unrecognized tag family name. Use e.g. \"tag36h11\".\n");
        std::cout << "Using tag36h11 for now..." << std::endl;
        tf = tag36h11_create();
    }

	//Create the detector. Uninteresting options commented out for now.
	td = apriltag_detector_create();
    apriltag_detector_add_family(td, tf);
//    td->quad_decimate = getopt_get_double(getopt, "decimate");
//    td->quad_sigma = getopt_get_double(getopt, "blur");
    td->nthreads = numThreads;
//    td->debug = getopt_get_bool(getopt, "debug");
//    td->refine_edges = getopt_get_bool(getopt, "refine-edges");
//    td->refine_decode = getopt_get_bool(getopt, "refine-decode");
//    td->refine_pose = getopt_get_bool(getopt, "refine-pose");


}

AprilTagDetector::~AprilTagDetector() {
	apriltag_detector_destroy(td);
	tag36h11_destroy(tf);
}

//This assumes that the image is in mono8 format!
void AprilTagDetector::detectTags(const cv::Mat& img, std::vector<TagDetection>& tagDetections, int seq) {
	//Put the image in an external-friendly bin
	image_u8_t im =
	{
		.width = img.cols,
		.height = img.rows,
		.stride = img.cols,
		.buf = img.data
	};

	zarray_t* detections = apriltag_detector_detect(td, &im);
//std::cout << zarray_size(detections) << " tags detected" << std::endl;

	tagDetections.clear();
	convertTagDetections(detections, tagDetections, seq);

	apriltag_detections_destroy(detections);
}

void AprilTagDetector::drawTags(cv::Mat& img, const std::vector<TagDetection>& tagDetections) {
	for(size_t i=0; i<tagDetections.size(); ++i) {
		// calculate length of tag diagonal
		int dx = tagDetections[i].corners[0][0]-tagDetections[i].corners[2][0];
		int dy = tagDetections[i].corners[0][1]-tagDetections[i].corners[2][1];
		int disparity = sqrt(dx*dx + dy*dy);

		// set corner circle size, thickness and tag id font size depending on disparity
		int circleSize = disparity/15;
		int circleThickness = sqrt(disparity/30);
		double fontSize = disparity/100.0;

		// threshholding
		if (circleSize < 5) { circleSize = 5; }
		if (circleThickness < 2) { circleThickness = 2; }
		if (fontSize < 0.5) { fontSize = 0.5; }
		if (fontSize > 4.0) { fontSize = 4.0; }

		// Draw circles at corners of detected tags on the video stream
		for (size_t c=0; c<4; c++) {
			cv::circle(img, cv::Point(tagDetections[i].corners[c][0], tagDetections[i].corners[c][1]),
					circleSize, CV_RGB(0,255,0), circleThickness);
		}

		// Draw tag id
		cv::putText(img, std::to_string(tagDetections[i].id),
				cv::Point(0.5*(tagDetections[i].corners[0][0]+tagDetections[i].corners[3][0]), 0.5*(tagDetections[i].corners[0][1]+tagDetections[i].corners[3][1])),
				cv::FONT_HERSHEY_SIMPLEX, fontSize, CV_RGB(0,255,0), 2);
	}
}


