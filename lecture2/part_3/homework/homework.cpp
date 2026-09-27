#include <iostream>
#include <opencv2/opencv.hpp>

// ======================= 作业 =======================
// 1. 读取 ../assets/demo.jpg
// 2. 使用 cvtColor 把图像转为灰度图（颜色空间：2. 使用 cvtColor 把图像转为灰度图（颜色空间：BGR2GRAY））
// 3. 使用 imwrite 把灰度图保存为 gray.jpg
// 4. 在灰度图上用 circle 画一个圆，标记你要"瞄准"的位置
// 5. 显示灰度图，按任意键退出
// ====================================================

int main()
{
    cv::Mat img = cv::imread("assets/demo.jpg");
    if (img.empty())
    {
        std::cout << "读取图片失败！请检查路径" << std::endl;
        return -1;
    }
    cv::Mat gray;
    cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY);
    cv::imwrite("gray.jpg", gray);
    cv::Point center(gray.cols / 2, gray.rows / 2);
    cv::circle(gray, center, 20, cv::Scalar(255), 2);
    cv::imshow("gray picture", gray);
    cv::waitKey(0);
    return 0;
}
