#include "uav_decision.h"

UavDecision::UavDecision(ros::NodeHandle &nh) : nh_(nh) {
	ROS_INFO("UavDecision initialized");
	nh_.getParam("/target_points", target_points_flat_);
	// 这些是充电区的参数，和move2Target函数中的参数不一样
	nh_.getParam("/map_north_angle", map_north_angle_);
	nh_.getParam("/speed", speed);
	nh_.getParam("/fly_time", fly_time_);
	nh_.getParam("/Kp", Kp_);
	nh_.getParam("/x_offset", x_offset_);
	nh_.getParam("/y_offset", y_offset_);

	for (int i = 0; i < target_points_flat_.size(); i += 3) {
		uav_points temp{};
		temp.vel_x = target_points_flat_[i];
		temp.angel = target_points_flat_[i + 1];
		temp.time = target_points_flat_[i + 2];
		target_points_.push_back(temp);
	}

	detector_point_.point.x = -1;
	detector_point_.point.y = -1;

	start_service_ = nh_.advertiseService("start_processing", &UavDecision::startCallback, this);
	flightByVelclient_ = nh_.serviceClient<ttauav_node::flightByVel>("flightByVel");
	gimbalControlclient_ = nh_.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
	takeoffOrLandingclient_ = nh_.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");
	detector_point_sub_ = nh_.subscribe("/many_rect_detector/center", 5, &UavDecision::detectorPointCallback, this);
	camera_status_pub_ = nh_.advertise<std_msgs::Bool>("/camera_status", 1);
	work_thread_ = std::make_shared<std::thread>(&UavDecision::work, this);

}

UavDecision::~UavDecision() {
	if (work_thread_->joinable()) {
		work_thread_->join();
	}
}

void UavDecision::work() {
	std::this_thread::sleep_for(std::chrono::seconds(3));

	ros::Rate loop_rate(5); // 10Hz

	while (ros::ok()) {
		if (start_processing) {
			if (current_state_==0) {
				Takeoffmode();
			} else if (current_state_==1) {
				move2Target();
			} else if (current_state_==2) {
				Detectmode();
			} else if (current_state_==3) {
				moveback();
			} else if (current_state_==4) {
				Detectmode_over();
			} else {
				ROS_INFO("Error state");
			}
			// start_processing = false;
		}
	}
}

void UavDecision::Detectmode() {
	std_msgs::Bool camera_status;
	camera_status.data = true;
	camera_status_pub_.publish(camera_status);

	if (detector_point_.point.x==-1 && detector_point_.point.y==-1) {
		ROS_INFO("No mark detected");
		return;
	}
	// 检测aruco时间戳，如果太远也return
	double current_time = ros::Time::now().toSec();
	double aruco_time = detector_point_.header.stamp.toSec();
	if (current_time - aruco_time > 3) {
		ROS_INFO("detect point too old");
		return;
	}
	ROS_INFO("Detect mode");
	double speedx, speedy;
	detect_point_pid_tracker(speedx, speedy);
	cv::Mat detect_point_track_speed = cv::Mat::zeros(2, 1, CV_32F);
	detect_point_track_speed.at<float>(1, 0) = float(speedx);
	detect_point_track_speed.at<float>(0, 0) = float(speedy);
	float rotation_data[4] = {static_cast<float>(cos(map_north_angle_)), static_cast<float>(-sin(map_north_angle_)),
							  static_cast<float>(sin(map_north_angle_)), static_cast<float>(cos(map_north_angle_))};
	cv::Mat rotation_matrix = cv::Mat(2, 2, CV_32F, rotation_data).clone();
	cv::Mat vel_rotated = rotation_matrix*detect_point_track_speed;
	flightbyVel.request.vel_n = detect_point_track_speed.at<float>(0, 0);
	flightbyVel.request.vel_e = detect_point_track_speed.at<float>(1, 0);
	flightbyVel.request.vel_d = 0;
	flightbyVel.request.fly_time = fly_time_;
	//    flightByVelclient_.call(flightbyVel);
	bool ack = flightByVelclient_.call(flightbyVel);
	if (ack) {
		ROS_INFO("move to aruco success!!");
		ros::Duration(6).sleep();
	} else {
		ROS_INFO("move to aruco failed!!");
	}

	if (abs(detector_point_.point.x - img_width_) < 50 && abs(detector_point_.point.y - img_height_) < 50) {
		takeofforLanding.request.takeoffOrLanding = 2;
		bool ack = takeoffOrLandingclient_.call(takeofforLanding);
		if (ack) {
			ROS_INFO("landing success!!");
			ros::Duration(10).sleep();
		} else {
			ROS_INFO("landing faied!!");
		}
		takeofforLanding.request.takeoffOrLanding = 1;
		bool ack1 = takeoffOrLandingclient_.call(takeofforLanding);
		if (ack1) {
			ROS_INFO("takeoff success!!");
			ros::Duration(10).sleep();
			current_state_ = 3;
		} else {
			ROS_INFO("takeoff faied!!");
		}

	}

}

void UavDecision::Takeoffmode() {

	takeofforLanding.request.takeoffOrLanding = 1;
	bool ack = takeoffOrLandingclient_.call(takeofforLanding);

	if (ack) {
		ROS_INFO("takeoff success!!");
		ros::Duration(6).sleep();
		current_state_ = 1;
	} else {
		ROS_INFO("takeoff faied!!");
	}

	// 云台调整为朝向正下方
	gimbalcontrol.request.pitch = -90;
	bool tiaojian = gimbalControlclient_.call(gimbalcontrol);
	ros::Duration(1).sleep();
	gimbalcontrol.request.pitch = -90;
	tiaojian = gimbalControlclient_.call(gimbalcontrol);
	if (tiaojian) {
		ROS_INFO("gimbalcontrol success!!!!!");
		ros::Duration(5).sleep();
	} else {
		ROS_INFO("gimbalcontrol failed!!!!!!");
	}

	// 调整飞行高度
	flightbyVel.request.vel_n = 0;
	flightbyVel.request.vel_e = 0;
	flightbyVel.request.vel_d = 0.25;
	flightbyVel.request.fly_time = 2000;
	flightByVelclient_.call(flightbyVel);
	bool ack1 = flightByVelclient_.call(flightbyVel);
	if (ack1) {
		ROS_INFO("move up success!!");
		ros::Duration(3).sleep();
	} else {
		ROS_INFO("move up failed!!");
	}

}

void UavDecision::Landingmode() {
	takeofforLanding.request.takeoffOrLanding = 2;
	bool ack = takeoffOrLandingclient_.call(takeofforLanding);

	if (ack) {
		ROS_INFO("landing success!!");
		current_state_ = 0;
	} else {
		ROS_INFO("landing faied!!");
	}

}

void UavDecision::move2Target() {
	for (const auto &target_point : target_points_) {
		transposed_points velocity_ = speed_resolver(target_point);
		flightbyVel.request.vel_n = velocity_.vel_n;
		flightbyVel.request.vel_e = velocity_.vel_e;
		flightbyVel.request.vel_d = velocity_.vel_d;
		flightbyVel.request.fly_time = velocity_.fly_time;
		flightByVelclient_.call(flightbyVel);
		ROS_INFO("Move to target");
		ros::Duration(velocity_.fly_time/1000 + 2).sleep();
	}
	current_state_ = 2;
}

void UavDecision::moveback() {
	std_msgs::Bool camera_status;
	camera_status.data = false;
	camera_status_pub_.publish(camera_status);

	for (const auto &target_point : target_points_) {
		transposed_points velocity_ = speed_resolver(target_point);
		flightbyVel.request.vel_n = -velocity_.vel_n;
		flightbyVel.request.vel_e = -velocity_.vel_e;
		flightbyVel.request.vel_d = velocity_.vel_d;
		flightbyVel.request.fly_time = velocity_.fly_time;
		flightByVelclient_.call(flightbyVel);
		ROS_INFO("Move back");
		ros::Duration(velocity_.fly_time/1000 + 2).sleep();
	}
	current_state_ = 4;
}

void UavDecision::Detectmode_over() {
	std_msgs::Bool camera_status;
	camera_status.data = true;
	camera_status_pub_.publish(camera_status);

	if (detector_point_.point.x==-1 && detector_point_.point.y==-1) {
		ROS_INFO("No mark detected");
		return;
	}
	// 检测aruco时间戳，如果太远也return
	double current_time = ros::Time::now().toSec();
	double aruco_time = detector_point_.header.stamp.toSec();
	if (current_time - aruco_time > 3) {
		ROS_INFO("detect point too old");
		return;
	}
	ROS_INFO("Detect mode");
	double speedx, speedy;
	detect_point_pid_tracker(speedx, speedy);
	cv::Mat detect_point_track_speed = cv::Mat::zeros(2, 1, CV_32F);
	detect_point_track_speed.at<float>(1, 0) = float(speedx);
	detect_point_track_speed.at<float>(0, 0) = float(speedy);
	float rotation_data[4] = {static_cast<float>(cos(map_north_angle_)), static_cast<float>(-sin(map_north_angle_)),
							  static_cast<float>(sin(map_north_angle_)), static_cast<float>(cos(map_north_angle_))};
	cv::Mat rotation_matrix = cv::Mat(2, 2, CV_32F, rotation_data).clone();
	cv::Mat vel_rotated = rotation_matrix*detect_point_track_speed;
	flightbyVel.request.vel_n = detect_point_track_speed.at<float>(0, 0);
	flightbyVel.request.vel_e = detect_point_track_speed.at<float>(1, 0);
	flightbyVel.request.vel_d = 0;
	flightbyVel.request.fly_time = fly_time_;
	//    flightByVelclient_.call(flightbyVel);
	bool ack = flightByVelclient_.call(flightbyVel);
	if (ack) {
		ROS_INFO("move to aruco success!!");
		ros::Duration(6).sleep();
	} else {
		ROS_INFO("move to aruco failed!!");
	}

	if (abs(detector_point_.point.x - img_width_) < 50 && abs(detector_point_.point.y - img_height_) < 50) {
		takeofforLanding.request.takeoffOrLanding = 2;
		bool ack = takeoffOrLandingclient_.call(takeofforLanding);
		if (ack) {
			ROS_INFO("landing success!!");
			ros::Duration(10).sleep();
		} else {
			ROS_INFO("landing faied!!");
		}

	}
}

transposed_points UavDecision::speed_resolver(uav_points target_point) {
	float angel_rad = target_point.angel*CV_PI/180.0;
	float rotation_data[4] = {cos(angel_rad), -sin(angel_rad),
							  sin(angel_rad), cos(angel_rad)};
	cv::Mat rotation_matrix_ = cv::Mat(2, 2, CV_32F, rotation_data).clone();
	cv::Mat vel = (cv::Mat_<float>(2, 1) << target_point.vel_x, 0);
	cv::Mat vel_rotated = rotation_matrix_*vel;
	transposed_points temp{};
	temp.vel_n = vel_rotated.at<float>(0, 0);
	temp.vel_e = vel_rotated.at<float>(1, 0);
	temp.vel_d = 0;
	temp.fly_time = target_point.time;
	return temp;

}

bool UavDecision::startCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
	start_processing = true;
	res.success = true;
	ROS_INFO("Start processing set to: %s", start_processing ? "true" : "false");
	return true;
}

void UavDecision::detectorPointCallback(const geometry_msgs::PointStamped::ConstPtr &msg) {
	detector_point_ = *msg;
	ROS_INFO("Get detect point: %f, %f, %f", detector_point_.point.x,
			 detector_point_.point.y, detector_point_.point.z);
}

void UavDecision::detect_point_pid_tracker(double &speedx, double &speedy) const {
	double errx = img_width_/2 + x_offset_ - detector_point_.point.x;
	double erry = img_height_/2 + y_offset_ - detector_point_.point.y;
	speedx = -Kp_*errx;
	speedy = Kp_*erry;

}
