#ifndef int_brain_hardware_MOTOR_HPP
#define int_brain_hardware_MOTOR_HPP

#include <string>

class Motor {
   public:
    std::string name_;
    int64_t enc_;
    double pos_;
    double vel_;
    float kp_;
    float ki_;
    float kd_;
    float kd_filter_coeff_;  // Derivative Filter Coefficient
    float ff_param_[2];      // Feedforward parameters
    double rpm_desired_;     // In rad/s
    double current_;

    Motor() = default;

    void setup(const std::string& name, float kp, float ki, float kd,
               float kd_filter_coeff, float ff0, float ff1);

    void set_encoder(int64_t enc);

    void reset_state();
};

#endif  // int_brain_hardware_MOTOR_HPP