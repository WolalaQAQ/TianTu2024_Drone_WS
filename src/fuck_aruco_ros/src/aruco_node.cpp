//
// Created by wolala on 24-6-26.
//
#include "fuck_aruco_ros/aruco_node.h"
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/tf.h>

ArucoNode::ArucoNode(ros::NodeHandle &nh) : nh_(nh) {
	it_ = std::make_shared<image_transport::ImageTransport>(nh_);
	nh_.getParam("/aruco_node/image_topic", image_topic_);
	nh_.getParam("/aruco_node/camera_info_topic", cam_info_topic_);
	nh_.getParam("/aruco_node/marked_image_topic", marked_image_topic_);
	nh_.getParam("/aruco_node/marker_length", marker_length_);
	nh_.getParam("/aruco_node/marker_pose_topic", marker_pose_topic_);
	nh_.getParam("/aruco_node/camera_frame", camera_frame_);

	ROS_INFO("Image Topic: %s, Camera Info Topic: %s, Marker Length: %f, Marker Pose Topic: %s, Camera Frame: %s",
			 image_topic_.c_str(), cam_info_topic_.c_str(),
			 marker_length_, marker_pose_topic_.c_str(),
			 camera_frame_.c_str());

	image_sub_.subscribe(nh_, image_topic_, 1);
	cam_info_sub_.subscribe(nh_, cam_info_topic_, 1);

	sync_ = std::make_shared<message_filters::Synchronizer<MySyncPolicy>>(MySyncPolicy(10), image_sub_, cam_info_sub_);
	sync_->registerCallback(std::bind(&ArucoNode::imageCallback, this, std::placeholders::_1, std::placeholders::_2));

	marked_image_pub_ = it_->advertise(marked_image_topic_, 1);

	marker_pose_pub_ = nh_.advertise<geometry_msgs::PoseArray>(marker_pose_topic_, 1);
}

void ArucoNode::imageCallback(const sensor_msgs::ImageConstPtr &img, const sensor_msgs::CameraInfoConstPtr &cam_info) {
	image_ = img;
	cam_info_ = cam_info;

//	ROS_INFO("Received image with width: %d, height: %d", img->width, img->height);
//	ROS_INFO("Received camera info with width: %d, height: %d", cam_info->width, cam_info->height);

	cv_bridge::CvImageConstPtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvShare(image_, "bgr8");
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	} catch (cv::Exception &e) {
		ROS_ERROR("cv exception: %s", e.what());
		return;
	} catch (std::exception &e) {
		ROS_ERROR("std exception: %s", e.what());
		return;
	}

	std::vector<int> ids;
	std::vector<std::vector<cv::Point2f>> corners;
	ROS_INFO("Start to detect!!!!!");
	detector_.detect(cv_ptr->image, ids, corners);

	ROS_INFO("Detected %zu markers", ids.size());
	cv::Mat marked_image = cv_ptr->image.clone();
	geometry_msgs::PoseArray pose_array;
	pose_array.header.stamp = image_->header.stamp;
	pose_array.header.frame_id = camera_frame_;

	if (!ids.empty()) {
		cv::aruco::drawDetectedMarkers(marked_image, corners, ids);

		std::vector<cv::Vec3d> rvecs, tvecs;
		cv::Mat camera_matrix, dist_coeffs;
		camera_matrix = cv::Mat(3, 3, CV_64F, (void *)cam_info->K.data());
		dist_coeffs = cv::Mat(cam_info->D.size(), 1, CV_64F, (void *)cam_info->D.data());

		detector_.estimatePose(ids, corners, marker_length_, camera_matrix, dist_coeffs, rvecs, tvecs);

		geometry_msgs::Pose pose;
		for (size_t i = 0; i < ids.size(); i++) {
			cv::aruco::drawAxis(marked_image, camera_matrix, dist_coeffs, rvecs[i], tvecs[i], marker_length_/2);
			pose.position.x = tvecs[i][0];
			pose.position.y = tvecs[i][1];
			pose.position.z = tvecs[i][2];

			// 旋转向量转换为四元数
			cv::Mat rot_mat;
			cv::Rodrigues(rvecs[i], rot_mat);
			cv::Matx33d rot_matx(rot_mat);
			tf::Quaternion q;
			tf::Matrix3x3 m(rot_matx(0, 0), rot_matx(0, 1), rot_matx(0, 2),
							rot_matx(1, 0), rot_matx(1, 1), rot_matx(1, 2),
							rot_matx(2, 0), rot_matx(2, 1), rot_matx(2, 2));
			m.getRotation(q);
			pose.orientation.x = q.x();
			pose.orientation.y = q.y();
			pose.orientation.z = q.z();
			pose.orientation.w = q.w();

			pose_array.poses.push_back(pose);
		}

		marker_pose_pub_.publish(pose_array);
	}


	if (marked_image_pub_.getNumSubscribers() > 0) {
		auto marked_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", marked_image).toImageMsg();
		marked_image_pub_.publish(marked_image_msg);
		ROS_INFO("Published marked image");
	}
}
