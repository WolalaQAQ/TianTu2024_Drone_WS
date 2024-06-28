#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
// #include <aruco_ros/marker.h>
// #include <aruco_ros/markerArray.h>
// #include <tf/transform_listener.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>


#include "ttauav_node/flightByVel.h"
#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "std_srvs/Trigger.h" 

class TakeoffLandingSys {
public:
    /**
     * @brief Construct a new Takeoff Landing Sys object 初始化ros节点，初始化服务客户端，订阅标记信息
     * 
     * @param nh 
     */
    TakeoffLandingSys(ros::NodeHandle& nh) : nh_(nh), 
    flightByVelclient_(nh_.serviceClient<ttauav_node::flightByVel>("flightByVel")),
    gimbalControlclient_(nh_.serviceClient<ttauav_node::gimbalControl>("gimbalControl")),
    takeoffOrLandingclient_(nh_.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding")),
    //订阅aruco_marker_publisher节点发布的标记信息   
    aruco_pose_sub_(nh_.subscribe("aruco_singler/pose", 
        5, &TakeoffLandingSys::arucoposeCallback, this)) {

        ROS_INFO("TakeoffLandingSys initialized");
        // 读取参数
        ROS_INFO("TakeoffLandingSys initialized with corridor_angle: %f",
             corridor_angle_);
        nh_.param<float>("corridor_angle", corridor_angle_, 0.0);
        float angle_rad = corridor_angle_ * CV_PI / 180.0; // 将角度转换为弧度
        float rotation_data[4] = { cos(angle_rad), -sin(angle_rad),
                                   sin(angle_rad), cos(angle_rad) };
        rotation_matrix_ = cv::Mat(2, 2, CV_32F, rotation_data).clone();
    }

    void arucoposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->aruco_pose_ = *msg;   
    }
    /**
     * @brief 降落处理函数
     * 
     */
    void landing_processor(){
        // 降落
        takeofforLanding.request.takeoffOrLanding = 2;
        takeoffOrLandingclient_.call(takeofforLanding);
        ROS_INFO("Landing");
    
    }
    /**
     * @brief 起飞处理函数
     * 
     */
    void takeoff_processor(){
        // 起飞
        takeofforLanding.request.takeoffOrLanding = 1;
        takeoffOrLandingclient_.call(takeofforLanding);
        ROS_INFO("Takeoff");
    }
    
    /**
     * @brief 运动控制函数，用于控制无人机运动到aruco上方
     * 
     */
    void move_to_aruco(){
        ttauav_node::flightByVel flightByVel_srv;
        // 由于飞机方向控制使用东南西北坐标系，所以要将东南西北坐标系转换为飞机的相机坐标系
        // 由于aruco_pose_实在相机坐标系下的，所以直接使用aruco_pose_的坐标
        // 实验室场地的正前方是西南245度，所以要发布
        speed_resolver();
        flightbyVel.request.vel_n = velocity_.at<float>(0);
        flightbyVel.request.vel_e = velocity_.at<float>(1);
        flightByVelclient_.call(flightbyVel);
        ROS_INFO("Move to aruco");
    }

    void speed_resolver(){
        displacement_.at<float>(0) = aruco_pose_.pose.position.x;
        displacement_.at<float>(1) = aruco_pose_.pose.position.y;
        time_ = 3;
        velocity_ = displacement_ / time_;
        velocity_ = rotation_matrix_ * velocity_;
    }


private:
    ros::NodeHandle nh_;
    ros::ServiceClient flightByVelclient_;
    ros::ServiceClient gimbalControlclient_;
    ros::ServiceClient takeoffOrLandingclient_;  
    ros::Subscriber aruco_pose_sub_;
    // tf::TransformListener tf_listener_;

    ttauav_node::flightByVel flightbyVel;
    ttauav_node::takeoffOrLanding takeofforLanding;
    ttauav_node::gimbalControl gimbalcontrol;

    
    geometry_msgs::PoseStamped aruco_pose_;
    cv::Mat displacement_ = cv::Mat::zeros(2, 1, CV_32F);
    cv::Mat velocity_ = cv::Mat::zeros(2, 1, CV_32F);
    cv::Mat rotation_matrix_;
    float time_ = 3;
    float corridor_angle_;



    
};
