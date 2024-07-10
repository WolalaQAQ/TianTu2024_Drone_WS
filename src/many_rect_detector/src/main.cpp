//
// Created by root on 7/8/24.
//
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include "many_rect_detector/many_rect_detector.h"
#include <geometry_msgs/PointStamped.h>
#include <cstdio>
#include <iostream>
#include <chrono>
#include <thread>
#include <DjiRtspImageSource.h>
#include <std_msgs/Bool.h>

bool use_rtsp = false;
std::string rtsp_url;
bool save_image = false;
bool camera_status_from_sub = false;
bool camera_status_true = false;

static inline int64_t now() {
	return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
}

void cameraStatusCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (msg->data != camera_status_from_sub) {
		camera_status_from_sub = msg->data;
	}
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "many_rect_detector_node");
	ros::NodeHandle nh;

	ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("center", 1);
	ros::Subscriber cam_status_sub = nh.subscribe("camera_status", 1, cameraStatusCallback);

	nh.getParam("/many_rect_detector/use_rtsp", use_rtsp);
	nh.getParam("/many_rect_detector/rtsp_url", rtsp_url);

	ROS_INFO("RTSP URL: %s", rtsp_url.c_str());

	DjiRtspImageSource service(rtsp_url);
	int64_t last_time = now();

	service.setImageCallback(nullptr, [&](void* handler, uint8_t* frmdata, int frmsize, int width, int height, int pixfmt) -> void {
		ROS_INFO("Image %d@%p  --  %dx%d -- %d\n", frmsize, frmdata, width, height, pixfmt);
		
		if (!frmdata) {
			ROS_ERROR("No frame data!");
			return;
		}

		int64_t now_time = now();

		if (now_time - last_time > 100) {
			last_time = now_time;	

			uint8_t* rgbData = new uint8_t[width * height * 3];
			int result = DjiRtspImageSource::FrameDataToRGB(frmdata, frmsize, width, height, pixfmt, rgbData);

			auto image = cv::Mat(height, width, CV_8UC3, rgbData);
			cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

			Center center = ManyRectDetector::detect(image);

			if (center.if_find) {
				ROS_INFO("center: (%f, %f)", center.x, center.y);
				geometry_msgs::PointStamped msg;
				msg.header.stamp = ros::Time::now();
				msg.header.frame_id = "camera";
				msg.point.x = center.x;
				msg.point.y = center.y;
				msg.point.z = 0;
				pub.publish(msg);
			} else {
				ROS_INFO("not find");
			}

			if (save_image) {
				char name[64];
				static int counter = 0;
				sprintf(name, "/root/TianTu2024_Drone_WS/images/%dx%d-%d_%d.jpg", width, height, pixfmt, ++counter);
				cv::imwrite(name, image);
			}
			
			delete[] rgbData;
		}
	});

	ros::Rate rate(10);
	while (ros::ok()) {
		if (camera_status_from_sub) {
			if (!camera_status_true) {
				service.start();
				camera_status_true = true;
				ROS_INFO("Camera started!!!!!!!!!!!");
			}
		} else {
			if (camera_status_true) {
				service.stop();
				camera_status_true = false;
				ROS_INFO("Camera stopped!!!!!!!!!!!");
			}
		}
	}

	return 0;
}