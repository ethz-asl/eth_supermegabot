#include "detector/utils.h"

image mat_to_image(const cv::Mat& m) {
  IplImage ipl = m;

  int h = ipl.height;
  int w = ipl.width;
  int c = ipl.nChannels;

  image im = make_image(w, h, c);
  unsigned char* data = (unsigned char*)ipl.imageData;
  int step = ipl.widthStep;
  for (int i = 0; i < h; ++i) {
    for (int k = 0; k < c; ++k) {
      for (int j = 0; j < w; ++j) {
        im.data[k * w * h + i * w + j] = data[i * step + j * c + k] / 255.;
      }
    }
  }

  return im;
}
