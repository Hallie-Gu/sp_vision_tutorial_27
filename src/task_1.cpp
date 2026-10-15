#include <chrono>
#include <cmath>
#include <exception>
#include <iostream>
#include <limits>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "io/gimbal/gimbal.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"

namespace
{
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

// 暂定范围，单位 m：排除近原点和极端远解，需按现场调整；不是机械安全限位。
constexpr double MIN_DISTANCE_M = 0.1;
constexpr double MAX_DISTANCE_M = 20.0;
constexpr auto CAMERA_TIMEOUT = 100ms;
constexpr double MIN_QUATERNION_NORM = 1e-6;  // 无量纲，仅防止零范数归一化。

enum Status { NO_TARGET = 0, TARGET_FOUND = 1, DATA_ERROR = 2, EXITING = 3, NOT_AUTO_AIM = 4 };

struct Command
{
  bool valid = false;
  bool control = false;
  float yaw = 0.0f;
  float pitch = 0.0f;
  // fire 固定为 false，不作为可变的控制状态。
};

// q(t) 仍可能阻塞；这里只检查返回值的数值，无法验证姿态包新鲜度。
bool read_pose(
  io::Gimbal & gimbal, auto_aim::Solver & solver, Clock::time_point t, Eigen::Vector3d & ypr)
{
  auto q = gimbal.q(t);
  const auto norm = q.norm();
  if (!q.coeffs().allFinite() || !std::isfinite(norm) || norm <= MIN_QUATERNION_NORM) return false;
  q.normalize();
  solver.set_R_gimbal2world(q);
  const auto rotation = solver.R_gimbal2world();
  if (!rotation.allFinite()) return false;
  const auto angles = tools::eulers(rotation, 2, 1, 0);
  if (!angles.allFinite()) return false;
  ypr = angles;
  return true;
}

nlohmann::json empty_telemetry()
{
  // 每轮重新创建，0 是占位，须结合 target_found / command_valid / pose_valid 读取。
  return {
    {"t", 0.0}, {"status", NO_TARGET}, {"target_found", 0}, {"command_valid", 0},
    {"pose_valid", 0}, {"gimbal_mode", -1},
    {"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"distance_m", 0.0},
    {"target_geometric_pitch_rad", 0.0},
    {"command_yaw_rad", 0.0f}, {"command_pitch_rad", 0.0f}, {"control", 0}, {"fire", 0},
    {"send_enabled", 0}, {"send_attempted", 0},
    {"gimbal_yaw_world_rad", 0.0}, {"gimbal_pitch_world_rad", 0.0},
    {"yaw_error_rad", 0.0}, {"pitch_error_rad", 0.0}
  };
}
}  // namespace

const std::string keys =
  "{help h usage ? | | 输出命令行参数说明}"
  "{enable-send    | false | 真实发送开关，默认关闭，现场负责人确认后才开启}"
  "{@config-path   | | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  try {
    cv::CommandLineParser cli(argc, argv, keys);
    const auto config_path = cli.get<std::string>("@config-path");
    const auto send_enabled = cli.get<bool>("enable-send");
    if (cli.has("help")) {
      cli.printMessage();
      return 0;
    }
    if (!cli.check() || config_path.empty()) {
      cli.printMessage();
      cli.printErrors();
      return 1;
    }

    tools::Exiter exiter;
    tools::Plotter plotter;
    const auto start = Clock::now();
    io::Gimbal gimbal(config_path);  // 首包等待仍可能阻塞，保留老师接口。

    // 唯一发送入口，退出也复用；记录的就是实际 float 接口参数。
    const auto publish = [&](const Command & command, nlohmann::json & data) {
      data["t"] = tools::delta_time(Clock::now(), start);
      data["command_valid"] = command.valid ? 1 : 0;
      data["command_yaw_rad"] = command.yaw;
      data["command_pitch_rad"] = command.pitch;
      data["control"] = command.control ? 1 : 0;
      data["fire"] = 0;
      data["send_enabled"] = send_enabled ? 1 : 0;
      data["send_attempted"] = 0;
      if (send_enabled) {
        data["send_attempted"] = 1;
        gimbal.send(command.control, false, command.yaw, command.pitch);
      }
      // attempted 只表示调用了 void send()，不能证明下位机收到或执行。
      plotter.plot(data);
    };
    const auto cancel_control = [&](Status status) noexcept {
      try {
        auto data = empty_telemetry();
        data["status"] = status;
        // 有限的 0,0 是 mode 0 占位，不是 control=true 回零；mode 0 行为待现场确认。
        publish(Command{}, data);
      } catch (...) {
        // 清理只尽力取消控制，不再抛出异常。
      }
    };

    try {
      io::Camera camera(config_path);
      auto_aim::YOLO yolo(config_path, true);
      auto_aim::Solver solver(config_path);

      while (!exiter.exit()) {
        Command command;  // 本轮从无效、control=false 开始，不保留上一轮目标。
        auto data = empty_telemetry();
        cv::Mat img;

        try {
          // 1. 读取图像与请求图像时刻的姿态。
          Clock::time_point t;
          Eigen::Vector3d gimbal_ypr = Eigen::Vector3d::Zero();
          if (!camera.try_read_for(img, t, CAMERA_TIMEOUT) || img.empty()) {
            data["status"] = DATA_ERROR;
          } else if (!read_pose(gimbal, solver, t, gimbal_ypr)) {
            data["status"] = DATA_ERROR;
          } else {
            data["pose_valid"] = 1;
            data["gimbal_yaw_world_rad"] = gimbal_ypr[0];
            data["gimbal_pitch_world_rad"] = gimbal_ypr[1];

            // 2. 检测、调用原 Solver，并在通过基础检查的装甲板中选最近的一块。
            auto armors = yolo.detect(img);
            auto min_distance = std::numeric_limits<double>::infinity();
            auto_aim::Armor * selected_armor = nullptr;
            for (auto & armor : armors) {
              bool points_valid = armor.points.size() == 4;
              for (const auto & point : armor.points) {
                points_valid = points_valid && std::isfinite(point.x) && std::isfinite(point.y);
              }
              if (!points_valid) continue;
              try {
                solver.solve(armor);
              } catch (const cv::Exception &) {
                continue;
              }
              // void solve() 丢弃了 PnP 返回值，也不暴露原始相机系深度。
              // 此处只能检查输出数值，不能据此宣称 PnP 成功或正深度已验证。
              if (!armor.xyz_in_world.allFinite() || !armor.ypd_in_world.allFinite()) continue;
              const auto distance = armor.ypd_in_world[2];
              if (distance < MIN_DISTANCE_M || distance > MAX_DISTANCE_M) continue;
              if (distance < min_distance) {
                min_distance = distance;
                selected_armor = &armor;
              }
            }

            // 3. 世界系绝对角度，单位 rad。几何 pitch 向上为正，命令 pitch 向下为正。
            if (selected_armor == nullptr) {
              data["status"] = armors.empty() ? NO_TARGET : DATA_ERROR;
            } else {
              const auto geometric_pitch = selected_armor->ypd_in_world[1];
              command.yaw = static_cast<float>(tools::limit_rad(selected_armor->ypd_in_world[0]));
              command.pitch = static_cast<float>(-geometric_pitch);
              command.valid = true;  // 上面已检查有限性，atan2 角度在 float 范围内。
              data["status"] = TARGET_FOUND;
              data["target_found"] = 1;
              data["x"] = selected_armor->xyz_in_world[0];
              data["y"] = selected_armor->xyz_in_world[1];
              data["z"] = selected_armor->xyz_in_world[2];
              data["distance_m"] = min_distance;
              data["target_geometric_pitch_rad"] = geometric_pitch;
              // 反馈来自 q(t)，不是最新 state()；使用与发送一致的 float 角度计算误差。
              data["yaw_error_rad"] = tools::limit_rad(command.yaw - gimbal_ypr[0]);
              data["pitch_error_rad"] = command.pitch - gimbal_ypr[1];
              tools::draw_points(img, selected_armor->points, {0, 0, 255});
            }
          }

          // 4. 显示与退出；按键退出时不再发送本轮的控制请求。
          if (!img.empty()) cv::imshow("selected armor", img);
          const auto key = cv::waitKey(1);
          if (key == 'q' || key == 27 || exiter.exit()) break;

          // 5. 自瞄模式且目标有效时才请求控制。dry-run 也计算同一份候选命令。
          const auto mode = gimbal.mode();  // 缓存模式，接口未暴露其接收时间。
          data["gimbal_mode"] = static_cast<int>(mode);
          command.control = command.valid && mode == io::GimbalMode::AUTO_AIM;
          if (command.valid && !command.control) data["status"] = NOT_AUTO_AIM;
          publish(command, data);  // 无目标/空图/异常数值均发送 control=false（若已开启发送）。
        } catch (const std::exception & e) {
          cancel_control(DATA_ERROR);
          std::cerr << "[Task1] Processing error: " << e.what() << '\n';
          return 1;
        }
      }

      cancel_control(EXITING);  // 先请求取消控制，再销毁相机等对象。
      return 0;
    } catch (...) {
      cancel_control(DATA_ERROR);
      throw;
    }
  } catch (const std::exception & e) {
    std::cerr << "[Task1] Fatal error: " << e.what() << '\n';
  }
  return 1;
}
