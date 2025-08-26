#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
#include <cmath>

class ImuFeedbackNode : public rclcpp::Node
{
public:
    ImuFeedbackNode() : Node("imu_feedback_node")
    {
        // Declare parameters for the two-point spline
        this->declare_parameter<double>("alpha", 1.0);
        this->declare_parameter<double>("mid_point", 5.0);

        // Read parameters
        this->get_parameter("alpha", alpha_);
        this->get_parameter("mid_point", mid_point_);

        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu_sensor_broadcaster/imu", 10, std::bind(&ImuFeedbackNode::imu_callback, this, std::placeholders::_1));

        feedback_pub_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("/joy/set_feedback", 10);
    }

private:
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        // Use orientation to estimate gravity vector
        double qx = msg->orientation.x;
        double qy = msg->orientation.y;
        double qz = msg->orientation.z;
        double qw = msg->orientation.w;

        // Components of gravity vector in sensor frame
        // http://www.ros.org/reps/rep-0103.html#axis-orientation
        // The gravity vector in the world frame is (0, 0, -g)
        // We rotate it to the sensor frame using the orientation quaternion
        double g = -9.81;
        double gravity_x = 2 * (qx * qz - qw * qy) * g;
        double gravity_y = 2 * (qy * qz + qw * qx) * g;

        // Remove gravity from linear acceleration
        float x_accel = msg->linear_acceleration.x - gravity_x;
        float y_accel = msg->linear_acceleration.y - gravity_y;

        // Use linear acceleration in x and y to calculate magnitude
        float magnitude = std::sqrt(x_accel * x_accel + y_accel * y_accel);

        float intensity = 1.0f / (1.0f + std::exp(-alpha_ * (magnitude - mid_point_)));

        // Clamp intensity to the range [0, 1]
        intensity = std::max(0.0f, std::min(1.0f, intensity));

        auto feedback_msg = std::make_unique<sensor_msgs::msg::JoyFeedback>();
        feedback_msg->type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
        feedback_msg->id = 0; // Assuming the first rumble motor
        feedback_msg->intensity = intensity;

        feedback_pub_->publish(std::move(feedback_msg));
    }

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr feedback_pub_;
    double alpha_;
    double mid_point_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImuFeedbackNode>());
    rclcpp::shutdown();
    return 0;
}