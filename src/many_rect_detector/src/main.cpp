//
// Created by root on 7/8/24.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "many_rect_detector/many_rect_detector.h"
#include <geometry_msgs/PointStamped.h>

bool use_rtsp = false;
std::string rtsp_url;

int main(int argc, char** argv) {
    ros::init(argc, argv, "many_rect_detector_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("center", 1);

	nh.getParam("/many_rect_detector/use_rtsp", use_rtsp);
	nh.getParam("/many_rect_detector/rtsp_url", rtsp_url);

	cv::VideoCapture cap;

	if (use_rtsp) {
		cap.open(rtsp_url);
	} else {
		cap.open(0);
	}

    if (!cap.isOpened()) {
        ROS_ERROR("open failed!");
        return -1;
    }

    cv::Mat frame;
    while (ros::ok()) {
        cap >> frame;
        if (frame.empty()) {
            ROS_ERROR("frame is empty!");
            break;
        }

		cv::imshow("frame", frame);
        Center center = ManyRectDetector::detect(frame);
		cv::imshow("result", frame);
		cv::waitKey(1);

		if (center.if_find) {
			ROS_INFO("center: (%f, %f)", center.x, center.y);
			geometry_msgs::PointStamped msg;
			msg.header.stamp = ros::Time::now();
			msg.header.frame_id = "camera";
			msg.point.x = center.x;
			msg.point.y = center.y;
			msg.point.z = 0;
			pub.publish(msg);
		}
		else {
			ROS_INFO("not find");
		}


    }

    ros::spin();
}