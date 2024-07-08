//
// Created by root on 7/8/24.
//

#ifndef BUILD_MANY_RECT_DETECTOR_H
#define BUILD_MANY_RECT_DETECTOR_H

#include <opencv2/opencv.hpp>
#include <string>

struct Center {
    bool if_find;
    double x;
    double y;
};

class ManyRectDetector {
public:
    ManyRectDetector() = default;

    static Center detect(const cv::Mat &src);

private:

};

#endif //BUILD_MANY_RECT_DETECTOR_H
