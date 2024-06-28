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

    ros::init(argc, argv, "uav_decision");
    ros::NodeHandle nh;

    flightByVelclient = nh.serviceClient<ttauav_node::flightByVel>("flightByVel");
    gimbalControlclient = nh.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
    takeoffOrLandingclient = nh.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");
    
    ROS_INFO("Service clients initialized");

    cv::Mat frame;

    // 定义并启动服务服务器
    ros::ServiceServer service = nh.advertiseService("start_processing", startProcessingCallback);

    ros::Rate loop_rate(10); // 10Hz

    while(ros::ok()){

        if(start_processing){
            // 起飞
            takeofforLanding.request.takeoffOrLanding = 1;
            if (takeoffOrLandingclient.call(takeofforLanding) && takeofforLanding.response.success) {
                ROS_INFO("Takeoff success!!");
            } else {
                ROS_ERROR("Takeoff failed!!");
                start_processing = false; // 停止处理，避免继续执行
                continue;
            }

            ros::Duration(10).sleep(); // 等待起飞完成

            // 想地图正前方前进0.3m
            float angle_rad = 230 * CV_PI / 180.0; // 将角度转换为弧度
            float rotation_data[4] = { cos(angle_rad), -sin(angle_rad), sin(angle_rad), cos(angle_rad) };
            cv::Mat rotation_matrix = cv::Mat(2, 2, CV_32F, rotation_data).clone();
            cv::Mat vel = (cv::Mat_<float>(2, 1) << 0.3, 0);
            cv::Mat vel_rotated = rotation_matrix * vel;
            flightbyVel.request.vel_n = vel_rotated.at<float>(0, 0);
            flightbyVel.request.vel_e = vel_rotated.at<float>(1, 0);
            flightbyVel.request.fly_time = 1;

            if (flightByVelclient.call(flightbyVel) && flightbyVel.response.success) {
                ROS_INFO("Move forward 0.3m");
            } else {
                ROS_ERROR("Move forward failed!!");
                start_processing = false; // 停止处理，避免继续执行
                continue;
            }

            ros::Duration(10).sleep(); // 等待前进完成

            // 降落
            takeofforLanding.request.takeoffOrLanding = 2;
            if (takeoffOrLandingclient.call(takeofforLanding) && takeofforLanding.response.success) {
                ROS_INFO("Landing success!!");
            } else {
                ROS_ERROR("Landing failed!!");
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
