#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <aruco_ros/marker.h>
#include <aruco_ros/markerArray.h>
#include <tf/transform_listener.h>

#include "ttauav_node/flightByVel.h"
#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "std_srvs/Trigger.h" 

class TakeoffLandingSys {
public:
    TakeoffLandingSys(ros::NodeHandle& nh) : nh_(nh), 
    flightByVelclient_(nh_.serviceClient<ttauav_node::flightByVel>("flightByVel")),
    gimbalControlclient_(nh_.serviceClient<ttauav_node::gimbalControl>("gimbalControl")),
    takeoffOrLandingclient_(nh_.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding")),
    //订阅aruco_marker_publisher节点发布的标记信息   
    aruco_pose_sub_(nh_.subscribe("aruco_singler/pose", 
        5, &TakeoffLandingSys::arucoposeCallback, this)) {
        ROS_INFO("TakeoffLandingSys initialized");
    }

    void arucoposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        this->aruco_pose_ = *msg;   
    }

private:
    ros::NodeHandle nh_;
    ros::ServiceClient flightByVelclient_;
    ros::ServiceClient gimbalControlclient_;
    ros::ServiceClient takeoffOrLandingclient_;  
    ros::Subscriber aruco_pose_sub_;
    tf::TransformListener tf_listener_;

    
    geometry_msgs::PoseStamped aruco_pose_;

    


    
};
