/*
 * TestGoToTask.cpp
 *
 *  Created on: Jun 29, 2019
 *      Author: farbod
 */

#include <csignal>
#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <any_measurements_ros/ConvertRosMessages.hpp>


std::vector<double> getCommandLine(size_t targetCommandSize) {

	std::vector<double> targetCommand(0);

	std::string line;
	std::getline(std::cin, line);
	std::istringstream stream(line);
	double in;
	while (stream >> in)
		targetCommand.push_back(in);

	// if the size is greater than targetCommandSize_
	const size_t n = targetCommand.size();
	if (n > targetCommandSize)
		targetCommand.erase(targetCommand.begin()+targetCommandSize, targetCommand.end());
	else
		for (size_t i=n; i<targetCommandSize; i++) {
			targetCommand.push_back(0.0);
		}  // end of i loop

	return targetCommand;
}


int main(int argc, char** argv) {

	// setup ROS
	::ros::init(argc, argv, "test_goto_publisher");
	::ros::NodeHandle nodeHandler;

	auto taskPublisher =  nodeHandler.advertise<nav_msgs::Path>(
			"mpc_trajectory", 1, false);

	 size_t cnt = 100;

	while (::ros::ok()) {

		const size_t commandSize = 3;

		// get command line
		std::cout << "Enter x y: ";
		std::vector<double> targetCommand = getCommandLine(commandSize);

		nav_msgs::Path msg;
		msg.header.frame_id = "base";
		msg.header.stamp = ros::Time::now();
		msg.header.seq = cnt;
		msg.poses.resize(1);
		auto& pose = msg.poses[0].pose;
		pose.position.x = targetCommand[0];
		pose.position.y = targetCommand[1];
		pose.position.z = 0.0;
		pose.orientation.x = 0.0;
		pose.orientation.y = 0.0;
		pose.orientation.z = 0.0;
		pose.orientation.w = 1.0;

		taskPublisher.publish(msg);

		cnt++;
	}

	return 0;
}
