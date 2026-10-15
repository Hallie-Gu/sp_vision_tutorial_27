#include <chrono>
#include <opencv2/opencv.hpp>
#include <limits>
#include <cmath>
#include <optional>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{@config-path   | | yaml配置文件路径 }";

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>("@config-path");
  if (cli.has("help") || !cli.has("@config-path")) {
    cli.printMessage();
    return 0;
  }

  // 初始化工具类
  tools::Exiter exiter;
  tools::Plotter plotter;   // 注意plotter工具的使用

  // 初始化io类
  io::Camera camera(config_path);
  io::Gimbal gimbal(config_path);

  // 初始化auto_aim类
  auto_aim::YOLO yolo(config_path, true);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;


  while (!exiter.exit()) {
    // Your code start
    camera.read(img,t);
    if (img.empty()) {
      tools::logger()->warn("Empty camera frame!");
      continue;
    }
    q=gimbal.q(t);
    solver.set_R_gimbal2world(q);

    auto armors = yolo.detect(img);
    double min_distance = std::numeric_limits<double>::infinity();
    auto_aim::Armor * selected_armor = nullptr;

    for (auto & armor : armors) {
      solver.solve(armor);
      double current_distance = armor.ypd_in_world[2];

      if (!std::isfinite(current_distance) || current_distance <= 0 ||
        !std::isfinite(armor.ypd_in_world[0]) ||
        !std::isfinite(armor.ypd_in_world[1])) {
        continue;
      }

      if (current_distance < min_distance) {
        min_distance = current_distance;
        selected_armor = &armor;
      }
    }
    

    if (selected_armor != nullptr) {
      auto gimbal_state = gimbal.state();
      nlohmann::json data;
      data["target_found"] = 1;
      auto gimbal_ypr_world =
      tools::eulers(solver.R_gimbal2world(), 2, 1, 0);
      double yaw_error = tools::limit_rad(
      selected_armor->ypd_in_world[0] - gimbal_ypr_world[0]);

      double pitch_error =
      selected_armor->ypd_in_world[1] - gimbal_ypr_world[1];

      data["x"] = selected_armor->xyz_in_world[0];
      data["y"] = selected_armor->xyz_in_world[1];
      data["z"] = selected_armor->xyz_in_world[2];

      data["target_yaw_rad"] = selected_armor->ypd_in_world[0];
      data["target_pitch_rad"] = selected_armor->ypd_in_world[1];

      

      data["yaw_error_deg"] = yaw_error * 180.0 / CV_PI;
      data["pitch_error_deg"] = pitch_error * 180.0 / CV_PI;

      data["gimbal_yaw_raw"] = gimbal_state.yaw;
      data["gimbal_pitch_raw"] = gimbal_state.pitch;

      data["gimbal_yaw_world_rad"] = gimbal_ypr_world[0];
      data["gimbal_pitch_world_rad"] = gimbal_ypr_world[1];

      
      data["yaw_error_rad"] = yaw_error;
      data["pitch_error_rad"] = pitch_error;

      plotter.plot(data);
    }
    else {
      nlohmann::json data;
      data["target_found"] = 0;
      plotter.plot(data);
    }

    if (selected_armor != nullptr) {
      tools::draw_points(img, selected_armor->points, {0, 0, 255});
    }

    cv::imshow("selected armor", img);
    cv::waitKey(1);
    // Your code end
    
  }
  
  return 0;
}