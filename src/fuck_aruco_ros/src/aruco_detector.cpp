//
// Created by wolala on 24-6-26.
//
#include "fuck_aruco_ros/aruco_detector.h"

ArucoDetector::ArucoDetector() {
	aruco_dict_ = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
	parameters_ = cv::makePtr<cv::aruco::DetectorParameters>();

//	for (size_t i = 0; i < dictNames.size(); i++) {
//		aruco_dicts.emplace_back(cv::aruco::getPredefinedDictionary(i));
//	}
}

void ArucoDetector::detect(const cv::Mat &src, std::vector<int> &ids, std::vector<std::vector<cv::Point2f>> &corners) {
	cv::Mat gray_image;
	cvtColor(src, gray_image, cv::COLOR_BGR2GRAY);
	GaussianBlur(gray_image, gray_image, cv::Size(5, 5), 0);
	equalizeHist(gray_image, gray_image);
	cv::aruco::detectMarkers(src, aruco_dict_, corners, ids, parameters_);
}

void ArucoDetector::estimatePose(
	const std::vector<int> &ids,
	const std::vector<std::vector<cv::Point2f>> &corners,
	float marker_length,
	const cv::Mat &camera_matrix,
	const cv::Mat &dist_coeffs,
	std::vector<cv::Vec3d> &rvecs,
	std::vector<cv::Vec3d> &tvecs) {
	if (corners.empty()) {
		std::cout << "Corners is empty!" << std::endl;
		return;
	}

	cv::aruco::estimatePoseSingleMarkers(corners, marker_length, camera_matrix, dist_coeffs, rvecs, tvecs);

}