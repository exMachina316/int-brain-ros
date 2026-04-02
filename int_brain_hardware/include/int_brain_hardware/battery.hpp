#ifndef INT_BRAIN_HARDWARE_BATTERY_HPP
#define INT_BRAIN_HARDWARE_BATTERY_HPP

#include <string>

class Battery {
   public:
    std::string name_;
    double voltage_;

    Battery() = default;

    Battery(const std::string name) : name_(name), voltage_(0.0) {}

    void reset_state() { voltage_ = 0.0; }
};

#endif  // INT_BRAIN_HARDWARE_BATTERY_HPP
