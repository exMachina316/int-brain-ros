#ifndef int_brain_hardware_MOTOR_HPP
#define int_brain_hardware_MOTOR_HPP

#include <string>
#include <vector>

class Motor
{
public:
    std::string name_;
    int64_t enc_;
    double pos_;
    double vel_;
    float kp_;
    float ki_;
    float kd_;
    float kd_filter_coeff_;     // Derivative Filter Coefficient
    float ff_param_[2];         // Feedforward parameters
    double rpm_desired_;        // In rad/s
    double current_;

    Motor() = default;

    void setup(const std::string &name, float kp, float ki, float kd, float kd_filter_coeff, float ff0, float ff1)
    {
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

    void set_encoder(int64_t enc)
    {
        enc_ = enc;
        pos_ = 2 * M_PI * ((enc % 1320) / 1320.0);
    }

    void reset_state()
    {
        pos_ = 0.0;
        vel_ = 0.0;
        current_ = 0.0;
    }
};

#endif // int_brain_hardware_MOTOR_HPP