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

#ifndef INCLUDE_CONFUSION_EXAMPLES_APRILTAGPARAMETERSH_
#define INCLUDE_CONFUSION_EXAMPLES_APRILTAGPARAMETERSH_

#include <Eigen/Core>
#include "../../models/TagMeas.h"

struct AprilTagParameters {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  void initialize() {
    tagMeasCalibration_.w_cx_ = 1.0 / tag_corner_stddev_;
    tagMeasCalibration_.w_cy_ = 1.0 / tag_corner_stddev_;

    double s = tagMeasCalibration_.tagSize_ / 2.0;
    tagMeasCalibration_.t_t_corner_[0] = Eigen::Vector3d(-s, -s, 0.0);
    tagMeasCalibration_.t_t_corner_[1] = Eigen::Vector3d(s, -s, 0.0);
    tagMeasCalibration_.t_t_corner_[2] = Eigen::Vector3d(s, s, 0.0);
    tagMeasCalibration_.t_t_corner_[3] = Eigen::Vector3d(-s, s, 0.0);

    initialized_ = true;
  }

  bool initialized_ = false;

  double tag_corner_stddev_;

  double t_c_i_init_stddev; //[m]
  double q_c_i_init_stddev; //[rad]

  confusion::TagMeasCalibration tagMeasCalibration_;
};

bool readTagPoses(const std::string &fname,
                  std::map<std::string, std::shared_ptr<confusion::Pose<double>>> &externalReferenceFrames) {
  std::cout << "Reading tag map from " << fname << std::endl;
  std::ifstream file(fname, std::ios::in);

  if (!file.good()) {
    std::cout << "ERROR: Unable to open the tag map file!" << std::endl;
    return false;
  }

  char line[200];
  char delimiters[] = " ";
  while (file.good()) {
    file.getline(line, sizeof(line));
    char *token = strtok(line, delimiters);
    if (strcmp(token, "donetags") == 0) {
      break;
    } else {
      if (token == NULL) {
        std::cout << "Error in readTagPoses. Got a NULL token when expecting a tag pose or donetags" << std::endl;
        return false;
      }
//			int tagId = atoi(token);
//			std::string frameName = "tag" + std::to_string(tagId);
      std::string frameName(token);

      Eigen::Matrix<double, 7, 1> poseData;
      for (int i = 0; i < 7; ++i) {
        token = strtok(NULL, delimiters);
        if (token == NULL) {
          std::cout << "Error in readTagPoses. Got a NULL token when reading the " << i
                    << "-th element of a tag pose" << std::endl;
          return false;
        } else
          poseData(i) = atof(token);
      }

      confusion::Pose<double>
          T_w_t(poseData(0), poseData(1), poseData(2), poseData(3), poseData(4), poseData(5), poseData(6));
      T_w_t.rot.normalize();
      T_w_t.print();

      externalReferenceFrames[frameName] = std::make_shared<confusion::Pose<double>>(T_w_t);
    }
  }
  std::cout << "Read " << externalReferenceFrames.size() << " tag poses from the tag map file" << std::endl;

  return true;
}

//Note that this does not write the gravity vector orientation when OPT_GRAVITY is defined in ImuState.h
void writeTagPoses(const std::string &fname,
                   const std::map<std::string,
                                  std::shared_ptr<confusion::Pose<double>>> &externalReferenceFrames) {
  std::ofstream file(fname);

  //Write the tag poses
  for (auto extFrameIter = externalReferenceFrames.begin(); extFrameIter != externalReferenceFrames.end();
       ++extFrameIter) {
    file << extFrameIter->first << " " << extFrameIter->second->trans(0) << " "
         << extFrameIter->second->trans(1) << " " << extFrameIter->second->trans(2) << " " <<
         extFrameIter->second->rot.w() << " " << extFrameIter->second->rot.x() << " "
         << extFrameIter->second->rot.y() << " " << extFrameIter->second->rot.z() << "\n";
//		std::cout << "Tag " << extFrameIter->first << std::endl;
//		extFrameIter->second->print("T_w_t");
  }
  file << "donetags\n";

  file.close();

  std::cout << "Wrote tag map to " << fname << std::endl;
}

#endif /* INCLUDE_CONFUSION_EXAMPLES_APRILTAGPARAMETERSH_ */
