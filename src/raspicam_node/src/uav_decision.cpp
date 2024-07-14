#include "uav_decision.h"


double last_processed_time;
int  work_mode = 0; // 1 is facing boxes qr,2 is facing down car,3 is flight track
int velnumber = 0;
ros::ServiceClient flightByVelclient;
ros::ServiceClient gimbalControlclient;
ros::ServiceClient takeoffOrLandingclient;  
ttauav_node::flightByVel flightbyVel;
ttauav_node::takeoffOrLanding takeofforLanding;
ttauav_node::gimbalControl gimbalcontrol;
double landing_threshold;
double track_threshold;
int X_axis_offset;
int Y_axis_offset;
bool no_has_wifi;
bool first_flag = true;
double vel_n;
double fly_time;

struct Center {
    bool iffind;
    double x;
    double y;
};
void track(cv::Mat frame, int width, int height)
{   
    cv::Mat img;
    cv::cvtColor(frame, img, cv::COLOR_RGB2GRAY); // 将输入图像转换为灰度图像
    cv::threshold(img, img, 127, 255, cv::THRESH_BINARY); // 将图像转换为二值图像

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE); // 轮廓提取

    std::vector<std::vector<cv::Point>> rects; // 存放四边形
    std::vector<cv::Point2d> centers; // 存放中心点

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < 100) { // 过滤掉矩形面积小的
            continue;
        }

        double epsilon = 0.02 * cv::arcLength(contour, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(contour, approx, epsilon, true);

        if (approx.size() == 4 && cv::isContourConvex(approx)) { // 过滤掉非四边形与非凸形
            rects.push_back(approx);
            cv::Point2d center = (approx[0] + approx[1] + approx[2] + approx[3]) / 4.0;
            centers.push_back(center);
        }
    }

    // 以下部分为聚类算法
    std::vector<int> center_iter(centers.size());
    std::iota(center_iter.begin(), center_iter.end(), 0);

    std::vector<int> result;
    double threshold = 20;

    while (!center_iter.empty()) {
        int j = 1;
        std::vector<int> resultnow;

        while (j < center_iter.size()) {
            if (cv::norm(centers[center_iter[0]] - centers[center_iter[j]]) < threshold) {
                resultnow.push_back(center_iter[j]);
                center_iter.erase(center_iter.begin() + j);
                j--;
            }
            j++;
        }

        resultnow.push_back(center_iter[0]);
        center_iter.erase(center_iter.begin());

        if (result.size() < resultnow.size()) {
            result = resultnow;
        }
    }

    std::vector<std::vector<cv::Point>> filteredRects;
    for (int index : result) {
        filteredRects.push_back(rects[index]);
    }

    Center center;
    // 如果嵌套的矩形数量大于2才算提取成功
    if (result.size() > 2) {
        cv::Point2d centerSum;
        for (int index : result) {
            centerSum += centers[index];
        }
        cv::Point2d centerAvg = centerSum / static_cast<double>(result.size());
        
        center.iffind = true;
        center.x = centerAvg.x > 1 ? centerAvg.x : 0;
        center.y = centerAvg.y > 1 ? centerAvg.y : 0;
        std::cout << "Center: ";
        std::cout << "(" << center.x << "," << center.y << ") ";
        std::cout << std::endl;
        double err_x = width/2.0 + X_axis_offset - center.x;
        double err_y = height/2.0 + Y_axis_offset - center.y;
        ROS_INFO_STREAM("state="<<err_x<<" "<<err_y);
        //速度控制
        float velx = 0.0007*err_y;
        float vely = -0.0007*err_x;
        ROS_INFO_STREAM("statevelx="<<velx<<" "<<velx);
        double current_time = ros::Time::now().toSec();
        double time_elapsed = current_time - last_processed_time;
        if(time_elapsed > 6){
            last_processed_time = current_time;
            
            flightbyVel.request.vel_n = velx;
            flightbyVel.request.vel_e = vely;
            flightbyVel.request.vel_d = 0;
            flightbyVel.request.fly_time = 300;
            bool ackvel =flightByVelclient.call(flightbyVel);
            if(ackvel)
            {
                ROS_INFO("cmdvel success!!");
                ros::Duration(1).sleep();
            }
            else
            {
                ROS_INFO("cmdvel faied!!");
                return;
            }
        }
        
        if(abs(velx) < 0.02 && abs(vely) < 0.02){
            std::cout<<velnumber<<std::endl;
            velnumber++;
        }
    }
    else {
        center.iffind = false;
    }
                          
}


int main(int argc, char** argv)
{
    std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
    cv::VideoCapture cap(rtsp_url);
    if (!cap.isOpened())
    {
        ROS_ERROR("Failed to open RTSP stream");
        return -1;
    }

    ros::init(argc, argv, "qr_code_reader");
    ros::NodeHandle nh;
    ros::service::waitForService("flightByVel");

    flightByVelclient = nh.serviceClient<ttauav_node::flightByVel>("flightByVel");
    gimbalControlclient = nh.serviceClient<ttauav_node::gimbalControl>("gimbalControl");
    takeoffOrLandingclient= nh.serviceClient<ttauav_node::takeoffOrLanding>("takeoffOrLanding");

    
    nh.param<bool>("no_has_wifi", no_has_wifi, false);
    nh.param<int>("work_mode", work_mode, 0);
    nh.param<double>("landing_threshold", landing_threshold, 0.05);
    nh.param<double>("track_threshold", track_threshold, 0.10);
    nh.param<double>("/uav_decision/vel_n",vel_n,0.0);
    nh.param<double>("/uav_decision/fly_time",fly_time,0.0);
    nh.param<int>("/uav_decision/X_axis_offset", X_axis_offset, -20);
    nh.param<int>("/uav_decision/Y_axis_offset", Y_axis_offset, 92);

    std::cout<<"no_has_wifi:"<<no_has_wifi<<std::endl;
    std::cout<<"work_mode:"<<work_mode<<std::endl;
    std::cout<<"landing_threshold:"<<landing_threshold<<std::endl;
    std::cout<<"track_threshold:"<<track_threshold<<std::endl;
    std::cout<<"X_axis_offset:"<<X_axis_offset<<std::endl;
    std::cout<<"Y_axis_offset:"<<Y_axis_offset<<std::endl;
    ROS_INFO("服务已经启动....");
    ros::Rate rate(1000);
    cv::Mat frame;

    last_processed_time = ros::Time::now().toSec();
    ros::Rate r(500);
    
    bool cap_open_flag = false;
    
    while (ros::ok())
    {   
        if(work_mode > 0){   
            if(!cap_open_flag){
                cap.open(rtsp_url);     
                while (!cap.isOpened()) {
                // 处理视频流打开失败的情况
                cap.open(rtsp_url);
                if (cap.isOpened()) {
                    break; // 视频流成功打开，跳出循环
                }
                // 处理视频流打开失败的情况
                // 可以进行一些等待操作，或者重试间隔
                // 例如：
                // r.sleep();
                }
                cap_open_flag=true;
            }

            cap.read(frame);
            if (frame.empty()){
                ROS_WARN("Frame is empty");
                continue;
            }
            static double last_time = ros::Time::now().toSec();
            double current_time = ros::Time::now().toSec();
            if(work_mode == 1 && (current_time - last_time >= 3)){
                cap.release();
                cap.open(rtsp_url);

                while (!cap.isOpened()) {
                    // 处理视频流打开失败的情况
                    cap.open(rtsp_url);
                    if (cap.isOpened()) {
                        break; // 视频流成功打开，跳出循环
                    }

                    // 处理视频流打开失败的情况
                    // 可以进行一些等待操作，或者重试间隔
                    // 例如：
                    // r.sleep();
                }
                last_time = current_time;
            
            }
            if((work_mode == 2 || work_mode == 3 || work_mode ==5 || work_mode ==6)
                 && (current_time - last_time >= 8)){
                cap.release();
                cap.open(rtsp_url);
                while (!cap.isOpened()) {
                    // 处理视频流打开失败的情况
                    cap.open(rtsp_url);
                    if (cap.isOpened()) {
                        break; // 视频流成功打开，跳出循环
                    }

                    // 处理视频流打开失败的情况
                    // 可以进行一些等待操作，或者重试间隔
                    // 例如：
                    // r.sleep();
                }
                last_time = current_time;
            
            }
        }
        else if(work_mode == 0){
            std::cout<<"123"<<std::endl;
            work_mode = 1;
            velnumber = 0;
            if(cap_open_flag == true)
                cap.release();
            cap_open_flag = false;
        }

        if(work_mode == 1){
            takeofforLanding.request.takeoffOrLanding = 1;
            bool ack = takeoffOrLandingclient.call(takeofforLanding);

            if (ack) {
                ROS_INFO("takeoff success!!");
                ros::Duration(4).sleep();
            } else {
                ROS_INFO("takeoff faied!!");
            }

            // 云台调整为朝向正下方
            gimbalcontrol.request.pitch = -90;
            bool tiaojian = gimbalControlclient.call(gimbalcontrol);
            ros::Duration(1).sleep();
            gimbalcontrol.request.pitch = -90;
            tiaojian = gimbalControlclient.call(gimbalcontrol);
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
            flightByVelclient.call(flightbyVel);
            bool ack1 = flightByVelclient.call(flightbyVel);
            if (ack1) {
                ROS_INFO("move up success!!");
                ros::Duration(3).sleep();
                work_mode = 2;
            } else {
                ROS_INFO("move up failed!!");
            }

        }
        else if(work_mode == 2){
            flightbyVel.request.vel_n = vel_n;
            flightbyVel.request.vel_e = 0;
            flightbyVel.request.vel_d = 0;
            flightbyVel.request.fly_time = fly_time;
            flightByVelclient.call(flightbyVel);
            bool ack1 = flightByVelclient.call(flightbyVel);
            if (ack1) {
                ROS_INFO("move to target success!!");
                ros::Duration((fly_time/1000)+2).sleep();
                work_mode = 3;
            } else {
                ROS_INFO("move to target failed!!");
            }
        }
        else if(work_mode == 3){   
            cv::Mat img(frame.rows, frame.cols, CV_8UC3, frame.data);
            std::cout<<"track work"<<std::endl;
            track(img, frame.cols, frame.rows); // 寻找矩形
            if(velnumber >= 8){
                std::cout<<"landing"<<std::endl;
                takeofforLanding.request.takeoffOrLanding = 2;
                bool lan = takeoffOrLandingclient.call(takeofforLanding);
                if(lan)
                {
                    ROS_INFO("landing success!!");
                    ros::Duration(9).sleep();
                    work_mode = 4;
                    velnumber = 0;
                }
                else
                {
                    ROS_INFO("landing faied!!");
                    return 0;
                }
            
            }
        }
        else if(work_mode ==4){
            takeofforLanding.request.takeoffOrLanding = 1;
            bool ack = takeoffOrLandingclient.call(takeofforLanding);
            if (ack) {
                ROS_INFO("takeoff success!!");
                ros::Duration(5).sleep();
            } else {
                ROS_INFO("takeoff faied!!");
            }
            gimbalcontrol.request.pitch = -90;
            bool tiaojian = gimbalControlclient.call(gimbalcontrol);
            ros::Duration(1).sleep();
            if (tiaojian) {
                ROS_INFO("gimbalcontrol success!!!!!");
            } else {
                ROS_INFO("gimbalcontrol failed!!!!!!");
            }
            flightbyVel.request.vel_n = 0;
            flightbyVel.request.vel_e = 0;
            flightbyVel.request.vel_d = 0.25;
            flightbyVel.request.fly_time = 2000;
            bool ack1 = flightByVelclient.call(flightbyVel);
            if (ack1) {
                ROS_INFO("move up success!!");
                ros::Duration(3).sleep();
                work_mode = 5;
            } else {
                ROS_INFO("move up failed!!");
            }

        }
        else if(work_mode == 5){
            flightbyVel.request.vel_n = -vel_n;
            flightbyVel.request.vel_e = 0;
            flightbyVel.request.vel_d = 0;
            flightbyVel.request.fly_time = fly_time;
            bool ack1 = flightByVelclient.call(flightbyVel);
            if (ack1) {
                ROS_INFO("move back success!!");
                ros::Duration((fly_time/1000)+2).sleep();
                work_mode = 6;
            } else {
                ROS_INFO("move back failed!!");
            }
        }
        else if(work_mode == 6){
            cv::Mat img(frame.rows, frame.cols, CV_8UC3, frame.data);
            std::cout<<"track work"<<std::endl;
            track(img, frame.cols, frame.rows); // 寻找矩形
            if(velnumber >= 8){
                std::cout<<"landing"<<std::endl;
                takeofforLanding.request.takeoffOrLanding = 2;
                bool lan = takeoffOrLandingclient.call(takeofforLanding);
                if(lan)
                {
                    ROS_INFO("landing success!!");
                    ros::Duration(5).sleep();
                    velnumber = 0;
                }
                else
                {
                    ROS_INFO("landing faied!!");
                }
            
            }
        }
        else{
            std::cout<<"work_mode error"<<std::endl;
        }
        
        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}
