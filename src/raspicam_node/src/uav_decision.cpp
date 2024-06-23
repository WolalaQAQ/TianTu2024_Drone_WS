#include "uav_decision.h"

ros::ServiceClient flightByVelclient;
ros::ServiceClient gimbalControlclient;
ros::ServiceClient takeoffOrLandingclient;  

ttauav_node::flightByVel flightbyVel;
ttauav_node::takeoffOrLanding takeofforLanding;
ttauav_node::gimbalControl gimbalcontrol;

bool start_processing = false; // 全局变量，控制循环是否启动

bool startProcessingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

int main(int argc, char** argv){
    // std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
    // cv::VideoCapture cap(rtsp_url);
    // if (!cap.isOpened())
    // {
    //     ROS_ERROR("Failed to open RTSP stream");
    //     return -1;
    // }

    ros::init(argc, argv, "uav_decision");
    ros::NodeHandle nh;
    // ros::service::waitForService("flightByVel");

    flightByVelclient = nh.serviceClient<ttauav_node::flightByVel>("flightByVel");
    gimbalControlclient = nh.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
    takeoffOrLandingclient= nh.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");
    
    ROS_INFO("Service clients initialized");

    cv::Mat frame;

    // 定义并启动服务服务器
    ros::ServiceServer service = nh.advertiseService("start_processing", startProcessingCallback);

    ros::Rate loop_rate(10); // 10Hz

    while(ros::ok()){

        if(start_processing){
            ros::Duration(1).sleep();
            takeofforLanding.request.takeoffOrLanding = 1;
            bool ack = takeoffOrLandingclient.call(takeofforLanding);

            if(ack){
                    ROS_INFO("takeoff success!!");
                    ros::Duration(10).sleep();
                }
            else{
                    ROS_INFO("takeoff faied!!");
                    continue;
                }

            ros::Duration(10).sleep();
            takeofforLanding.request.takeoffOrLanding = 2;
            bool lan = takeoffOrLandingclient.call(takeofforLanding);
            if(lan)
            {
                ROS_INFO("landing success!!");
            }
            else
            {
                ROS_INFO("landing faied!!");
                continue;
            }
            start_processing = false;
        }
        
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool startProcessingCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    start_processing = true;
    res.success = true;
    ROS_INFO("Start processing set to: %s", start_processing ? "true" : "false");
    return true;
}