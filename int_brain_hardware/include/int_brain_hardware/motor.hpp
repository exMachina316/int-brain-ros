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
    double rpm_desired_;
    double current_;

    Motor() = default;

    Motor(const std::string name)
        : name_(name),
          enc_(0.0), pos_(0.0), vel_(0.0),
          rpm_desired_(30.0),
          current_(0.0) {}

    void setup(const std::string &name)
    {
        name_ = name;
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