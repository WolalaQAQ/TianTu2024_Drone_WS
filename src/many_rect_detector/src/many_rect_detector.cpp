//
// Created by root on 7/8/24.
//
#include "many_rect_detector/many_rect_detector.h"

#include <numeric>

Center ManyRectDetector::detect(const cv::Mat &src) {
	cv::Mat img;
	cv::cvtColor(src, img, cv::COLOR_RGB2GRAY); // 将输入图像转换为灰度图像

	cv::threshold(img, img, 127, 255, cv::THRESH_BINARY); // 将图像转换为二值图像

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(img, contours, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE); // 轮廓提取

	std::vector<std::vector<cv::Point>> rects; // 存放四边形
	std::vector<cv::Point2d> centers; // 存放中心点

	for (const auto &contour : contours) {
		if (cv::contourArea(contour) < 100) { // 过滤掉矩形面积小的
			continue;
		}

		double epsilon = 0.02*cv::arcLength(contour, true);
		std::vector<cv::Point> approx;
		cv::approxPolyDP(contour, approx, epsilon, true);

		if (approx.size()==4 && cv::isContourConvex(approx)) { // 过滤掉非四边形与非凸形
			rects.push_back(approx);
			cv::Point2d center = (approx[0] + approx[1] + approx[2] + approx[3])/4.0;
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

	Center center{};
	if (result.size() > 2) {
		cv::Point2d centerSum;
		for (int index : result) {
			centerSum += centers[index];
		}
		cv::Point2d centerAvg = centerSum/static_cast<double>(result.size());

		center.if_find = true;
		center.x = centerAvg.x > 1 ? centerAvg.x : 0;
		center.y = centerAvg.y > 1 ? centerAvg.y : 0;
//		std::cout << "Center: ";
//		std::cout << "(" << center.x << "," << center.y << ") ";
//		std::cout << std::endl;
		// 下面注释掉的部分将画出提取出的轮廓
		cv::polylines(src, filteredRects, true, cv::Scalar(0, 0, 255), 2);
		cv::circle(src, cv::Point(center.x, center.y), 5, cv::Scalar(0, 255, 0), -1);
		cv::putText(src, "Center", cv::Point(center.x, center.y), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
//         cv::imshow("w", src);
//         cv::waitKey(1);

	} else {
		center.if_find = false;
	}

	return center;
}
