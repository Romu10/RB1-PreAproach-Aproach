#include "rclcpp/subscription.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

class VelParam: public rclcpp::Node
{
  public:
    VelParam()
      : Node("param_vel_node")
    {
      // Velocity Parameter Configuration
        auto lin_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        lin_vel_param_desc.description = "Sets the velocity (in m/s) of the robot.";
        this->declare_parameter<std::double_t>("linear_velocity", 0.0, lin_vel_param_desc);

      // Turning Parameter Configuration
        auto ang_vel_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        ang_vel_param_desc.description = "Sets the angular velocity (in r/s) of the robot.";
        this->declare_parameter<std::double_t>("angular_velocity", 0.0, ang_vel_param_desc);
      
      // Timer Configuration
        timer_ = this->create_wall_timer(
        1000ms, std::bind(&VelParam::timer_callback, this));
      
      // Publisher Configuration 
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);
    
      // Laser Sub
        laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&VelParam::Laser_callback, this, std::placeholders::_1),
        options1);


    }
    
    void timer_callback()
    {
        // Getting linear velocity parameter data
        this->get_parameter("linear_velocity", lin_vel_parameter_);
        RCLCPP_INFO(this->get_logger(), "Linear Velocity parameter is: %f", lin_vel_parameter_);

        // Gettin angular velocity parameter data 
        this->get_parameter("angular_velocity", ang_vel_parameter_);
        RCLCPP_INFO(this->get_logger(), "Angular Velocity parameter is: %f", ang_vel_parameter_);

        // Printing Front Distance 
        RCLCPP_INFO(this->get_logger(), "Front Robot Distance: %f", front_distance);
        
        // Publishing to robot cmd_vel
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = lin_vel_parameter_;
        publisher_->publish(message);
    }
    
    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        // Message received
        int range_total = msg->ranges.size();
        //RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);
        
        // Getiting Front Distance 
        front_distance = msg->ranges[540];
    }

  
  private:

    // Define the parameter
    std::double_t lin_vel_parameter_;
    std::double_t ang_vel_parameter_;

    // Define Timer 
    rclcpp::TimerBase::SharedPtr timer_;

    // Define the Publisher for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Variables 
    float front_distance = 0.00;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<VelParam> VelParam_node = std::make_shared<VelParam>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(VelParam_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
