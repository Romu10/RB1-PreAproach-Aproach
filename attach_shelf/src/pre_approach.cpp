#include "rclcpp/subscription.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace std::chrono_literals;

class VelParam: public rclcpp::Node
{
  public:
    VelParam()
      : Node("param_vel_node")
    {
      // Velocity Parameter Configuration
        auto obstacle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        obstacle_param_desc.description = "Sets the distance (in m) from the robot to the wall.";
        this->declare_parameter<std::double_t>("obstacle", 0.0, obstacle_param_desc);

      // Turning Parameter Configuration
        auto degrees_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        degrees_param_desc.description = "Sets the degrees for rotate the robot.";
        this->declare_parameter<std::double_t>("degrees", 0.0, degrees_param_desc);
      
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

      // Odom Sub 
        odom_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options2;
        options2.callback_group = odom_callback_group_;

        subscription2_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10,
        std::bind(&VelParam::Odom_callback, this, std::placeholders::_1),
        options2);

    }
    
    void timer_callback()
    {
        // Getting linear velocity parameter data
        this->get_parameter("obstacle", obs_parameter_);
        RCLCPP_INFO(this->get_logger(), "Obstacle parameter is: %f", obs_parameter_);

        // Gettin angular velocity parameter data 
        this->get_parameter("degrees", dgs_parameter_);
        RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %f", dgs_parameter_);

        // Printing Front Distance 
        RCLCPP_INFO(this->get_logger(), "Front Robot Distance: %f", front_distance);
        
        if (front_distance > obs_parameter_ && obs_parameter_ > 0.00)
        {
            message.linear.x = 0.2;
            RCLCPP_INFO(this->get_logger(), "Moving Forward");
        }
        else if (dgs_parameter_ > 0.00)
        {
            message.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Max Distance Reached");
            rotate_robot_degrees(dgs_parameter_);
        }
        publisher_->publish(message);
    }
    
    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {
        // Message received
        int range_total = msg->ranges.size();
        //RCLCPP_INFO(this->get_logger(), "Total Laser Received: %i", range_total);
        
        // Getiting Front Distance 
        front_distance = msg->ranges[540];
    }

    void Odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_theta = msg->pose.pose.orientation.z;
        //RCLCPP_INFO(this->get_logger(), "Received Odometry: z = %f", current_theta);
    }

    void rotate_robot(float vel)
    {
        message.angular.z = vel;
        publisher_->publish(message);
    }

    void rotate_robot_degrees(float degrees_to)
    {
      // Obtiene el número de grados desde la solicitud
        float degrees = degrees_to;
        float radians = degrees * (M_PI/180);
        float initial_theta = current_theta;

      // Calculus
        float target_theta = initial_theta + radians; 
        float error_percet = 0.05; // 5%
        float max_value_degree = target_theta+(target_theta*error_percet);
        float min_value_degree =  target_theta-(target_theta*error_percet);

        while (rclcpp::ok())
            {
                RCLCPP_INFO(this->get_logger(), "Current theta: %f | Target theta: %f | Initial theta: %f | Min_value: %f | Max_value: %f ", current_theta, target_theta, initial_theta, min_value_degree, max_value_degree);
                float orientation_error = target_theta - current_theta;
                if (fabs(orientation_error) < error_percet)
                {
                    rotate_robot(0.0);
                    float current_theta_degree = ((current_theta*180)/M_PI);
                    float target_theta_degree = ((target_theta*180)/M_PI);
                    RCLCPP_INFO(this->get_logger(), "Current theta in degree: %f ||| Target theta in degree: %f", current_theta_degree, target_theta_degree); 
                    RCLCPP_INFO(this->get_logger(), "Current theta equal to Target theta");
                }    

                if (orientation_error > 0.0)
                {
                    rotate_robot(0.1);
                }
                else 
                {
                    rotate_robot(-0.1);
                }

        }
        RCLCPP_INFO(this->get_logger(), "Finished Rotate Movement");
    }

  
  private:

    // Define the parameter
    std::double_t obs_parameter_;
    std::double_t dgs_parameter_;

    // Define Timer 
    rclcpp::TimerBase::SharedPtr timer_;

    // Define Twist 
    geometry_msgs::msg::Twist message;

    // Define the Publisher for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Define the subscription to the Odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

    // Variables 
    float front_distance = 0.00;
    float current_theta = 0.0;

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
