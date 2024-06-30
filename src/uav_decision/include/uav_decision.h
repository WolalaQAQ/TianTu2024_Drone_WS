#ifndef BUILD_SRC_UAV_DECISION_INCLUDE_UAV_DECISION_H
#define BUILD_SRC_UAV_DECISION_INCLUDE_UAV_DECISION_H

#include <thread>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

// #include"fuck_aruco_ros/aruco_detector.h"

#include "ttauav_node/flightByVel.h"
#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "std_srvs/Trigger.h" 

struct uav_points{
    double vel_x;
    double angel;
    double time;
};

struct transposed_points{
    float vel_n;
    float vel_e;
    float vel_d;
    float fly_time;
};

class UavDecision{
    public:
        UavDecision() = delete;
        explicit UavDecision(ros::NodeHandle& nh);
        ~UavDecision();

    private:

      	ros::NodeHandle nh_;

        ros::ServiceClient flightByVelclient_;
        ros::ServiceClient gimbalControlclient_;
        ros::ServiceClient takeoffOrLandingclient_; 
        ros::ServiceServer start_service_; 

        ros::Subscriber aruco_pose_sub_;

        ttauav_node::flightByVel flightbyVel;
        ttauav_node::takeoffOrLanding takeofforLanding;
        ttauav_node::gimbalControl gimbalcontrol;

        std::shared_ptr<std::thread> work_thread_;

    private:

        void work();

        void move2Target();
        void Landingmode();
        void Takeoffmode();
        void Detectmdoe();

        bool startCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

    private:
        // utils
        transposed_points speed_resolver(uav_points target_point);

    private:
        // 动态参数
        double map_north_angle_;
        double speed;
        double fly_time_;
        std::vector<double> target_points_flat_;

        geometry_msgs::PoseStamped aruco_pose_;
        cv::Mat velocity_ = cv::Mat::zeros(2, 1, CV_32F);
        std::vector<uav_points> target_points_;
        
        bool start_processing = false;
        int current_state_ = 0;


};




#endif //BUILD_SRC_UAV_DECISION_INCLUDE_UAV_DECISION_H