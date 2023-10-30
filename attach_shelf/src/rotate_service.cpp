#include "attach_shelf/srv/rotate.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class MoveRB1 : public rclcpp::Node
{
public:
    MoveRB1() : Node("rotate_robot_node")
    {
        // ROS Services
        service = create_service<attach_shelf::srv::Rotate>(
            "rotate_robot",
            std::bind(&MoveRB1::rotateServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        // ROS Publishers
        vel_pub = create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

         // Odom Sub 
        odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options2;
        options2.callback_group = odom_callback_group_;

        subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&MoveRB1::odometryCallback, this, std::placeholders::_1),
        options2);

        RCLCPP_INFO(get_logger(), "Service Server Configured");

    }

    void rotate_robot(float vel)
    {
        vel_msg_.angular.z = vel;
        vel_pub->publish(vel_msg_);
    }

private:
    void rotateServiceCallback(
        const std::shared_ptr<attach_shelf::srv::Rotate::Request> req,
        std::shared_ptr<attach_shelf::srv::Rotate::Response> res)
    {
        RCLCPP_INFO(get_logger(), "The Service /rotate_robot has been called");

        // Obtiene el número de grados desde la solicitud
        float degrees = req->degrees;
        float radians = ((degrees/2) * M_PI) / 180;
        float initial_theta = current_theta_;  

        // Calculus
        float target_theta = initial_theta + radians;
        float error_percent = 0.05; // 5%
        float max_value_degree = target_theta + (target_theta * error_percent);
        float min_value_degree = target_theta - (target_theta * error_percent);

        rclcpp::Rate rate(10); // Control loop rate

        while (rclcpp::ok()) {
            RCLCPP_INFO(get_logger(), "Current theta: %f | Target theta: %f | Initial theta: %f | Min_value: %f | Max_value: %f ",
                        current_theta_, target_theta, initial_theta, min_value_degree, max_value_degree);
            float orientation_error = target_theta - current_theta_;  
            if (fabs(orientation_error) < error_percent) {
                rotate_robot(0.0);
                float current_theta_degree = ((current_theta_ * 180) / M_PI);
                float target_theta_degree = (target_theta * 180 / M_PI);
                RCLCPP_INFO(get_logger(), "Current theta in degree: %f ||| Target theta in degree: %f", current_theta_degree, target_theta_degree);
                RCLCPP_INFO(get_logger(), "Current theta equal to Target theta");
                res->result = "Rotation completed successfully";
                return;
            }

            if (orientation_error > 0.0) {
                rotate_robot(0.2);
            } else {
                rotate_robot(-0.2);
            }
            rate.sleep();
        }

        res->result = "Rotation NOT complete";
        RCLCPP_INFO(get_logger(), "Finished service /rotate_robot");
    }

    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_theta_ = msg->pose.pose.orientation.z;  // Actualizar el valor al que apunta el puntero
        current_degrees_ = ((current_theta_ * 2) * 180) / M_PI;
        RCLCPP_WARN(get_logger(), "Current Degrees: %f", current_degrees_);
    }

private:
    rclcpp::Service<attach_shelf::srv::Rotate>::SharedPtr service;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    // Define the subscription to the Odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

    geometry_msgs::msg::Twist vel_msg_;
    float current_theta_;  
    float current_degrees_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<MoveRB1> MoveRB1_node = std::make_shared<MoveRB1>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(MoveRB1_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
