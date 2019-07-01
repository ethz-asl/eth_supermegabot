#ifndef UTILS_H_
#define UTILS_H_

#include <opencv2/highgui/highgui.hpp>

// Darknet libraries.
#include <darknet.h>

// Transform greyscale opencv image to darknet format.
image mat_to_image(const cv::Mat& m);

#endif  // UTILS_H_
