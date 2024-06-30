#include "uav_decision.h"

UavDecision::UavDecision(ros::NodeHandle &nh):nh_(nh){
    ROS_INFO("UavDecision initialized");
    nh_.getParam("/target_points", target_points_flat_);
    // 这些是充电区的参数，和move2Target函数中的参数不一样
    nh_.getParam("/map_north_angle", map_north_angle_);
    nh_.getParam("/speed", speed);
    nh_.getParam("/fly_time", fly_time_);

    for(int i = 0; i < target_points_flat_.size(); i+=3){
        uav_points temp;
        temp.vel_x = target_points_flat_[i];
        temp.angel = target_points_flat_[i+1];
        temp.time = target_points_flat_[i+2];
        target_points_.push_back(temp);
    }

    start_service_ = nh_.advertiseService("start_processing", &UavDecision::startCallback, this);
    flightByVelclient_ = nh_.serviceClient<ttauav_node::flightByVel>("flightByVel");
    gimbalControlclient_ = nh_.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
    takeoffOrLandingclient_ = nh_.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");
    // TODO:这个地方应该用像素点方案，后续会修改
    // aruco_pose_sub_ = nh_.subscribe("aruco_singler/pose", 5, &UavDecision::arucoposeCallback, this);
    aruco_pose_sub_ = nh_.subscribe("/aruco_node/marker_point",5,&UavDecision::arucoposeCallback,this);
    work_thread_ = std::make_shared<std::thread>(&UavDecision::work, this);

}

UavDecision::~UavDecision(){
    if(work_thread_->joinable()){
        work_thread_->join();
    }
}

void UavDecision::work(){
    std::this_thread::sleep_for(std::chrono::seconds(3));

    ros::Rate loop_rate(5); // 10Hz

    while(ros::ok()){
        if(start_processing){
            if(current_state_ == 0){
                Takeoffmode();
            }
            else if(current_state_ == 1){
                move2Target();
            }
            else if(current_state_ == 2){
                Detectmode();
            }
            else if(current_state_ == 3){
                Landingmode();
            }
            else{
                ROS_INFO("Error state");
            }
            start_processing = false;
        }
    }
}

void UavDecision::Detectmode(){
    ROS_INFO("Detect mode");
    // 将相机画面中心移动到aruco的中心的像素点的一定范围内
    if(aruco_point_.point.x < img_width_/2 - 50){
        aruco_speed_.at<float>(0, 0) = -speed;
    }
    else if(aruco_point_.point.x > img_width_/2 + 50){
        aruco_speed_.at<float>(0, 0) = speed;
    }
    if(aruco_point_.point.y < img_height_/2 - 50){
        aruco_speed_.at<float>(1, 0) = -speed;
    }
    else if(aruco_point_.point.y > img_height_/2 + 50){
        aruco_speed_.at<float>(1, 0) = speed;
    }
    float rotation_data[4] = { cos(map_north_angle_), -sin(map_north_angle_),
                               sin(map_north_angle_), cos(map_north_angle_) };
    cv::Mat rotation_matrix = cv::Mat(2, 2, CV_32F, rotation_data).clone();
    cv::Mat vel_rotated = rotation_matrix * aruco_speed_;
    flightbyVel.request.vel_n = vel_rotated.at<float>(0, 0);
    flightbyVel.request.vel_e = vel_rotated.at<float>(1, 0);
    flightbyVel.request.vel_d = 0;
    flightbyVel.request.fly_time = fly_time_;
    flightByVelclient_.call(flightbyVel);
    bool ack = flightByVelclient_.call(flightbyVel);
    if(ack){
        ROS_INFO("move to aruco success!!");
    }
    else{
        ROS_INFO("move to aruco failed!!");
    }
    if(aruco_point_.point.x < img_width_/2 + 50 && aruco_point_.point.x > img_width_/2 - 50 &&
        aruco_point_.point.y < img_height_/2 + 50 && aruco_point_.point.y > img_height_/2 - 50){
        takeofforLanding.request.takeoffOrLanding = 2;
        bool ack = takeoffOrLandingclient_.call(takeofforLanding);
        if(ack){
            ROS_INFO("landing success!!");
            ros::Duration(10).sleep();
        }
        else{
            ROS_INFO("landing faied!!");
        }
        takeofforLanding.request.takeoffOrLanding = 1;
        bool ack1 = takeoffOrLandingclient_.call(takeofforLanding);
        if(ack1){
            ROS_INFO("takeoff success!!");
            ros::Duration(10).sleep();
            current_state_ = 3;
        }
        else{
            ROS_INFO("takeoff faied!!");
        }
        
    }

}

void UavDecision::Takeoffmode(){
    takeofforLanding.request.takeoffOrLanding = 1;
    bool ack = takeoffOrLandingclient_.call(takeofforLanding);

    if(ack){
        ROS_INFO("takeoff success!!");
        ros::Duration(6).sleep();
        current_state_ = 1;
    }
    else{
        ROS_INFO("takeoff faied!!");
    }

}

void UavDecision::Landingmode(){
    takeofforLanding.request.takeoffOrLanding = 2;
    bool ack = takeoffOrLandingclient_.call(takeofforLanding);

    if(ack){
        ROS_INFO("landing success!!");
        current_state_ = 0;
    }
    else{
        ROS_INFO("landing faied!!");
    }

}

void UavDecision::move2Target(){
    for(int i = 0; i < target_points_.size(); i++){
        transposed_points velocity_ = speed_resolver(target_points_[i]);
        flightbyVel.request.vel_n = velocity_.vel_n;
        flightbyVel.request.vel_e = velocity_.vel_e;
        flightbyVel.request.vel_d = velocity_.vel_d;
        flightbyVel.request.fly_time = velocity_.fly_time;
        flightByVelclient_.call(flightbyVel);
        ROS_INFO("Move to target");
        ros::Duration(velocity_.fly_time/1000+2).sleep();
    }
    current_state_ = 2;
}

transposed_points UavDecision::speed_resolver(uav_points target_point){
    float angel_rad = target_point.angel * CV_PI / 180.0;
    float rotation_data[4] = { cos(angel_rad), -sin(angel_rad),
                               sin(angel_rad), cos(angel_rad) };
    cv::Mat rotation_matrix_ = cv::Mat(2, 2, CV_32F, rotation_data).clone();
    cv::Mat vel = (cv::Mat_<float>(2, 1) << target_point.vel_x, 0);
    cv::Mat vel_rotated = rotation_matrix_ * vel;
    transposed_points temp;
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

void UavDecision::arucoposeCallback(const geometry_msgs::PointStamped::ConstPtr& msg){
    aruco_point_ = *msg;
    ROS_INFO("Get aruco point: %f, %f, %f", aruco_point_.point.x, 
        aruco_point_.point.y, aruco_point_.point.z);
}