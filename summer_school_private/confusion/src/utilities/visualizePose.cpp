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

#include <ros/ros.h>
#include "confusion/utilities/Pose.h"
#include "confusion/utilities/ros_conversions.h"

using namespace confusion;

int main(int argc, char** argv) {
	ros::init(argc, argv, "visualizePose");
	ros::NodeHandle nh;

	ros::Publisher TaaPub = nh.advertise<geometry_msgs::PoseStamped>("/T_a_a", 10);
	ros::Publisher TabPub = nh.advertise<geometry_msgs::PoseStamped>("/T_a_b", 10);

	Pose<double> T_a_a;
	geometry_msgs::PoseStamped TaaMsg = getMsg(T_a_a, "/world");

	Pose<double> T_a_b;
	Eigen::Vector3d rpy;
	if (!nh.getParam("/x_a_b", T_a_b.trans(0)))
		nh.setParam("/x_a_b", T_a_b.trans(0));
	if (!nh.getParam("/y_a_b", T_a_b.trans(1)))
		nh.setParam("/y_a_b", T_a_b.trans(1));
	if (!nh.getParam("/z_a_b", T_a_b.trans(2)))
		nh.setParam("/z_a_b", T_a_b.trans(2));
	if (!nh.getParam("/roll_a_b", T_a_b.trans(0)))
		nh.setParam("/roll_a_b", rpy(0));
	if (!nh.getParam("/pitch_a_b", T_a_b.trans(1)))
		nh.setParam("/pitch_a_b", rpy(1));
	if (!nh.getParam("/yaw_a_b", T_a_b.trans(2)))
		nh.setParam("/yaw_a_b", rpy(2));

	ros::Rate r(10);
	while (ros::ok()) {
		nh.getParam("/x_a_b", T_a_b.trans(0));
		nh.getParam("/y_a_b", T_a_b.trans(1));
		nh.getParam("/z_a_b", T_a_b.trans(2));
		nh.getParam("/roll_a_b", rpy(0));
		nh.getParam("/pitch_a_b", rpy(1));
		nh.getParam("/yaw_a_b", rpy(2));

		T_a_b.rot = rpy_to_quat(rpy);

		geometry_msgs::PoseStamped TabMsg = getMsg(T_a_b, "/world");
		TaaPub.publish(TaaMsg);
		TabPub.publish(TabMsg);

		T_a_b.print("T_a_b");

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}
