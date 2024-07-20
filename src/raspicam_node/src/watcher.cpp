#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/SetBool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <numeric>

#include "geometry_msgs/Twist.h"
#include "std_msgs/Int32.h"

#include "ttauav_node/flightByVel.h"
#include "ttauav_node/gimbalControl.h"
#include "ttauav_node/takeoffOrLanding.h"
#include "std_srvs/Trigger.h" 

double last_processed_time;
int  work_mode = 0; // 1 is facing boxes qr,2 is facing down car,3 is flight track
int velnumber = 0;
double landing_threshold;
double track_threshold=0.04;
double Kp;
int X_axis_offset=1;
int Y_axis_offset=97;
bool no_has_wifi;
bool first_flag = true;
double vel_n;
double fly_time;
double Time_delay;
int track_time;
bool track_time_out = false;
double back_extra_time;

bool start_processing = false;
struct Center {
    bool iffind;
    double x;
    double y;
};
void track(const cv::Mat &frame, int width, int height,int work_mode)
{
	// static auto count = 0;
	// cv::imwrite("/root/TianTu2024_Drone_WS/image/" + std::to_string(count++) + ".jpg", frame);
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
        float velx = 0.0015*err_y;
        float vely = -0.0015*err_x;
        ROS_INFO_STREAM("statevelx="<<velx<<" "<<velx);
        double current_time = ros::Time::now().toSec();
        double time_elapsed = current_time - last_processed_time;
        if(abs(velx) < track_threshold 
            && abs(vely) < track_threshold){
            std::cout<<velnumber<<std::endl;
            velnumber++;
        }
    }
    else {
        center.iffind = false;
    }
                          
}



int main(int argc, char** argv){
    std::string rtsp_url = "rtsp://127.0.0.1:8554/live";
    cv::VideoCapture cap(rtsp_url);

    ros::init(argc, argv, "watcher");
    ros::NodeHandle nh;
    ros::service::waitForService("flightByVel");
    ROS_INFO("服务已经启动....");
    ros::Rate rate(1000);
    cv::Mat frame;
    ros::Rate r(500);

    bool cap_open_flag = false;
    last_processed_time = ros::Time::now().toSec();
    while(ros::ok()){
        if(!cap_open_flag){
                    ROS_INFO("!cap_open_flag");
                    cap.open(rtsp_url);     
                    while (!cap.isOpened()) {
                    // 处理视频流打开失败的情况
                    cap.open(rtsp_url);
                        if (cap.isOpened()) {
                            break; // 视频流成功打开，跳出循环
                        }
                    }
                    cap_open_flag=true;
                }
         cap.read(frame);
                if (frame.empty()){
                    ROS_WARN("Frame is empty");
                    // cap_open_flag = false;
                    continue;
                }
        static double last_time = ros::Time::now().toSec();
        double current_time = ros::Time::now().toSec();
        if((current_time - last_time >= 8)){
                    ROS_INFO("current_time - last_time >= 3");
                    cap.release();
                    cap.open(rtsp_url);
                    while (!cap.isOpened()) {
                        // 处理视频流打开失败的情况
                        cap.open(rtsp_url);
                        if (cap.isOpened()) {
                            break; // 视频流成功打开，跳出循环
                        }
                    }
                    last_time = current_time;
                
                }
         cv::Mat img = frame.clone();
        std::cout<<"track work"<<std::endl;
        track(img, frame.cols, frame.rows,work_mode); // 寻找矩形
    }



}