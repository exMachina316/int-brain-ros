#include "int_brain_hardware/motor.hpp"

#include <cmath>
#include <vector>

void Motor::setup(const std::string& name, float kp, float ki, float kd,
                  float kd_filter_coeff, float ff0, float ff1) {
    name_ = name;
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    kd_filter_coeff_ = kd_filter_coeff;
    ff_param_[0] = ff0;
    ff_param_[1] = ff1;

    enc_ = 0.0;
    pos_ = 0.0;
    vel_ = 0.0;
    rpm_desired_ = 0.0;
    current_ = 0.0;
}

void Motor::set_encoder(int64_t enc) {
    enc_ = enc;
    pos_ = 2 * M_PI * ((enc % 1320) / 1320.0);
}

void Motor::reset_state() {
    pos_ = 0.0;
    vel_ = 0.0;
    current_ = 0.0;
}