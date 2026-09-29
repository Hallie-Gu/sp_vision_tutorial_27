#include "io/camera.hpp"
#include "tasks/yolo.hpp"
#include "opencv2/opencv.hpp"
#include "tools/img_tools.hpp"
#include "tasks/apriltag_detector.hpp"

int main()
{
    // 初始化相机、yolo类
    Camera camera;

    std::string config_path = "./configs/yolo.yaml";
    auto_aim::YOLO yolo(config_path);
    auto_charge::AprilTagDetector apriltag_detector(config_path);

     int frame_count = 0;

     while (1) {

        // 调用相机读取图像
        cv::Mat img;
        camera.read(img);
        auto tags = apriltag_detector.detect(img);

        //apriltag detect
        for (const auto& tag : tags){
     tools::draw_points(
        img,
        tag.corners,
        cv::Scalar(0, 255, 0),
        2
    );

    std::string text = "ID:" + std::to_string(tag.id);

    cv::Point text_point(
        static_cast<int>(tag.corners[0].x),
        static_cast<int>(tag.corners[0].y)
    );

    tools::draw_text(
        img,
        text,
        text_point,
        cv::Scalar(0, 255, 0),
        1.0,
        2
    );
}

        // 调用yolo识别装甲板
        auto armors = yolo.detect(img, frame_count);
        for (const auto& armor : armors){
            tools::draw_points(img,armor.points,cv::Scalar(0, 255, 0),2);
            std::string text =auto_aim::COLORS[armor.color] +auto_aim::ARMOR_NAMES[armor.name];
            tools::draw_text(img,text,armor.box.tl(),cv::Scalar(0, 255, 0),1.0,2);
        }
        
        frame_count++;

        // 显示图像
        cv::resize(img, img , {},1.5,1.5);
        cv::imshow("img", img);
        if (cv::waitKey(1) == 'q') {
         break;
         }
    }

    return 0;
}