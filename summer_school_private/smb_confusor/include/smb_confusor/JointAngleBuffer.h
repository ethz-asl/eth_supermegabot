/*
 * JointAngleBuffer.h
 *
 *  Created on: Feb 16, 2018
 *      Author: Tim Sandy
 */

#ifndef INCLUDE_JOINTANGLEBUFFER_H_
#define INCLUDE_JOINTANGLEBUFFER_H_

#include <boost/circular_buffer.hpp>
#include <Eigen/Core>

namespace smb_state_estimator {

template<size_t N>
class JointAngleBuffer {
 public:
  JointAngleBuffer(size_t queueSize) : buffer(queueSize) {}

  void addMeasurement(double t, Eigen::Matrix<double, N, 1> &q) {
    buffer.push_back(std::pair<double, Eigen::Matrix < double, N, 1>>
    (t, q));
  }

  bool get(const double &t, Eigen::Matrix<double, N, 1> &q_out) {
    if (buffer.empty()) {
      printf("JointAngleBuffer is empty!\n");
      return false;
    }
    if (t < buffer.front().first) {
      printf("Requested the joint angles at a time earlier than those stored"
             " in the buffer. t_query=%15.4f, t_front=%15.4f\n", t, buffer.front().first);
      return false;
    }

    std::pair<double, Eigen::Matrix < double, N, 1>> previous;
    do {
      previous = buffer.front();
      buffer.pop_front();

      if (buffer.empty()) {
        q_out = previous.second;
        if (t - previous.first > 0.002) { //Allow queries to come a little bit later than the measurements since we are reading the Kinova measurements slower than the controller is running
          printf("Requested the joint angles at a time later than those stored"
                 " in the buffer. t_query=%15.4f, t_back=%15.4f\n", t, previous.first);
          return false;
        } else
          return true;
      }
    } while (t > buffer.front().first);

    q_out = interpolate(t, previous, buffer.front());

    return true;
  }

 private:
  boost::circular_buffer<std::pair<double, Eigen::Matrix < double, N, 1>>> buffer;

  Eigen::Matrix<double, N, 1> interpolate(const double &t,
                                          const std::pair<double, Eigen::Matrix < double, N, 1>> & q0,
                                          const std::pair<double, Eigen::Matrix < double, N, 1>>& q1) {
//		printf("t=%f, t0=%f, t1=%f",t,q0.first,q1.first);
    return q0.second + (q1.second - q0.second) * (t - q0.first) / (q1.first - q0.first);
  }
};

} // namespace smb_confusor

#endif /* INCLUDE_JOINTANGLEBUFFER_H_ */
