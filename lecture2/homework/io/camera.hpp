#pragma once
#include <opencv2/opencv.hpp>

class Camera
{
public:
    Camera();

    ~Camera();

    void read(cv::Mat& img);

private:
    void* handle_;

};