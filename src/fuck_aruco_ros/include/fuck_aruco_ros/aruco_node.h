//
// Created by wolala on 24-6-26.
//

#ifndef BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_NODE_H
#define BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_NODE_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "fuck_aruco_ros/aruco_detector.h"

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo> MySyncPolicy;

class ArucoNode {
public:
  ArucoNode() = delete;
  explicit ArucoNode(ros::NodeHandle& nh);

private:
  ros::NodeHandle nh_;
  std::shared_ptr<image_transport::ImageTransport> it_;

  std::shared_ptr<message_filters::Synchronizer<MySyncPolicy>> sync_;

  std::string image_topic_;
  message_filters::Subscriber<sensor_msgs::Image> image_sub_;
  std::string cam_info_topic_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> cam_info_sub_;

  std::string marked_image_topic_;
  image_transport::Publisher marked_image_pub_;

  std::string marker_pose_topic_;
  ros::Publisher marker_pose_pub_;

  std::string camera_frame_;

  sensor_msgs::ImageConstPtr image_;
  sensor_msgs::CameraInfoConstPtr cam_info_;

  ArucoDetector detector_;

  float marker_length_ = 0.0f;

  void imageCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& cam_info);
};


#endif //BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_NODE_H
