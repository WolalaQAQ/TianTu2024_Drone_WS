#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/opencv.hpp>

void publishCameraInfo(ros::Publisher &camera_info_pub, const cv::Mat &frame, ros::NodeHandle &nh) {
    sensor_msgs::CameraInfo camera_info_msg;
    camera_info_msg.header.stamp = ros::Time::now();
    camera_info_msg.header.frame_id = "camera";
    camera_info_msg.width = frame.cols;
    camera_info_msg.height = frame.rows;

    // 读取相机校准参数
    std::vector<double> K, D, R, P;
    nh.getParam("camera_matrix/data", K);
    nh.getParam("distortion_coefficients/data", D);
    nh.getParam("rectification_matrix/data", R);
    nh.getParam("projection_matrix/data", P);

    if (K.size() == 9) {
        for (int i = 0; i < 9; ++i) {
            camera_info_msg.K[i] = K[i];
        }
    }

    if (D.size() > 0) {
        camera_info_msg.D = D;
    }

    if (R.size() == 9) {
        for (int i = 0; i < 9; ++i) {
            camera_info_msg.R[i] = R[i];
        }
    }

    if (P.size() == 12) {
        for (int i = 0; i < 12; ++i) {
            camera_info_msg.P[i] = P[i];
        }
    }

    camera_info_pub.publish(camera_info_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "uav_cam_node");
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("image_raw", 1);
    ros::Publisher camera_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera_info", 1);

    std::string rtsp_url;
    nh.param<std::string>("rtsp_url", rtsp_url, "rtsp://127.0.0.1:8554/live");
    cv::VideoCapture cap(rtsp_url);

    if (!cap.isOpened()) {
        ROS_ERROR("Error opening video stream or file");
        return -1;
    }

    cv::Mat frame;
    sensor_msgs::ImagePtr img_msg;

    ros::Rate loop_rate(30);
    while (nh.ok()) {
        cap >> frame;
        if (frame.empty()) {
            ROS_ERROR("Captured empty frame");
            break;
        }

        img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
        image_pub.publish(img_msg);

        publishCameraInfo(camera_info_pub, frame, nh);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
