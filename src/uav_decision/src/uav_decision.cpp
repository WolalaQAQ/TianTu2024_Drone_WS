#include "uav_decision.h"

UavDecision::UavDecision(ros::NodeHandle &nh):nh_(nh){
    ROS_INFO("UavDecision initialized");
    nh_.getParam("/target_points", target_points_flat_);
    nh_.getParam("map_north_angle", map_north_angle_);
    nh_.getParam("speed", speed);
    nh_.getParam("fly_time", fly_time_);

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
                Landingmode();
            }
            else{
                ROS_INFO("Error state");
            }
            start_processing = false;
        }
    }
}

void UavDecision::Takeoffmode(){
    takeofforLanding.request.takeoffOrLanding = 1;
    bool ack = takeoffOrLandingclient_.call(takeofforLanding);

    if(ack){
        ROS_INFO("takeoff success!!");
        ros::Duration(10).sleep();
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
