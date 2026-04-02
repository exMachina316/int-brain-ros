#ifndef INT_BRAIN_HARDWARE_IMU_HPP
#define INT_BRAIN_HARDWARE_IMU_HPP

#include <string>

class Imu {
   public:
    std::string name_;
    double orientation_[4];          // Quaternion representation: w, x, y, z
    double angular_velocity_[3];     // rad/s
    double linear_acceleration_[3];  // m/s^2

    Imu() = default;
    Imu(const std::string& name) : name_(name) { reset_state(); }

    void reset_state() {
        orientation_[0] = 0.0f;
        orientation_[1] = 0.0f;
        orientation_[2] = 0.0f;
        orientation_[3] = 1.0f;  // Default to no rotation

        angular_velocity_[0] = 0.0f;
        angular_velocity_[1] = 0.0f;
        angular_velocity_[2] = 0.0f;

        linear_acceleration_[0] = 0.0f;
        linear_acceleration_[1] = 0.0f;
        linear_acceleration_[2] = 0.0f;
    }
};

#endif  // INT_BRAIN_HARDWARE_IMU_HPP
