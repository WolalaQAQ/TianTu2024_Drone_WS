//
// Created by wolala on 24-6-26.
//
#include <ros/ros.h>

#include "fuck_aruco_ros/aruco_node.h"

int main(int argc, char** argv) {
	ros::init(argc, argv, "aruco_node");
	ros::NodeHandle nh;
	ArucoNode aruco_node(nh);
	ros::spin();
}