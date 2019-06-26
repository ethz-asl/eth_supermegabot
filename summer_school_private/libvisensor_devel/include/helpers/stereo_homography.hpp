/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef STEREOHOMOGRAPHY_HPP_
#define STEREOHOMOGRAPHY_HPP_

#include <Eigen/Dense>

#include <visensor/visensor_datatypes.hpp>

class StereoHomography {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  StereoHomography(const visensor::ViCameraCalibration& calib_cam0,
                   const visensor::ViCameraCalibration& calib_cam1);

  //Computes homography for stereo rectification
  void getHomography(Eigen::Matrix3d& H0, Eigen::Matrix3d& H1, double& f_new, Eigen::Vector2d& p0_new,
                     Eigen::Vector2d& p1_new);

 private:
  double r0_[9];
  double t0_[3];
  double f0_[2];
  double p0_[2];
  double d0_[5];
  double r1_[9];
  double t1_[3];
  double f1_[2];
  double p1_[2];
  double d1_[5];
  int image_width_;
  int image_height_;
};

#endif /* STEREOHOMOGRAPHY_HPP_ */
