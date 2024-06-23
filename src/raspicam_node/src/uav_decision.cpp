#include "uav_decision.h"

ros::ServiceClient flightByVelclient;
ros::ServiceClient gimbalControlclient;
ros::ServiceClient takeoffOrLandingclient;  

int main(int argc, char** argv){
    std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
    cv::VideoCapture cap(rtsp_url);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open RTSP stream");
        return -1;
    }

    ros::init(argc, argv, "uav_decision");
    ros::NodeHandle nh;
    ros::service::waitForService("flightByVel");

    flightByVelclient = nh.serviceClient<ttauav_node::flightByVel>("flightByVel");
    gimbalControlclient = nh.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
    takeoffOrLandingclient= nh.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");
    
    ROS_INFO("Service clients initialized");

    cv::Mat frame;
}