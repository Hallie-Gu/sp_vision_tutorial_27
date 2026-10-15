#include <iostream>
#include <cmath>

int main() {
    const double pi = std::acos(-1.0);

    double target_yaw = -179.0 * pi / 180.0;
    double gimbal_yaw = 179.0 * pi / 180.0;

    double yaw_error = target_yaw - gimbal_yaw;

    while (yaw_error > pi) yaw_error -= 2 * pi;
    while (yaw_error <= -pi) yaw_error += 2 * pi;

    std::cout << yaw_error * 180.0 / pi << std::endl;

    double target_pitch = 15.0 * pi / 180.0;
    double gimbal_pitch = 5.0 * pi / 180.0;

    double pitch_error = target_pitch - gimbal_pitch;

    std::cout << pitch_error * 180.0 / pi << std::endl;

    return 0;
}