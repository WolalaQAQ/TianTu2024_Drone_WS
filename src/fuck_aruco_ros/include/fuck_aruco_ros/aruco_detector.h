//
// Created by wolala on 24-6-26.
//

#ifndef BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_DETECTOR_H
#define BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

class ArucoDetector {
public:
  ArucoDetector();

  void detect(const cv::Mat &src, std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners);

  void estimatePose(const std::vector<int> &ids,
					const std::vector<std::vector<cv::Point2f>> &corners,
					float marker_length,
					const cv::Mat &camera_matrix,
					const cv::Mat &dist_coeffs,
					std::vector<cv::Vec3d> &rvecs,
					std::vector<cv::Vec3d> &tvecs);

private:
  cv::Ptr<cv::aruco::Dictionary> aruco_dict_;
  cv::Ptr<cv::aruco::DetectorParameters> parameters_;

};

#endif //BUILD_SRC_FUCK_ARUCO_ROS_INCLUDE_FUCK_ARUCO_ROS_ARUCO_DETECTOR_H
