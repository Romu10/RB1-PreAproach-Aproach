#include "rclcpp/logging.hpp"
#include "rclcpp/subscription.hpp"
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include "attach_shelf/srv/rotate.hpp"

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
        500ms, std::bind(&VelParam::timer_callback, this));
      
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

      // Servie Client
        rotate_client_ = create_client<attach_shelf::srv::Rotate>("/rotate_robot");

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
        
        // Parameters Set Info
        if (obs_parameter_ <= 0.0 && dgs_parameter_ <= 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Please Set Obstacle & Deegres Parameters");
        }

        if (obs_parameter_ != 0.0 && task_1 == false)
        {
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Moving Forward");
        }
        
        if (obs_parameter_ != 0.0 && front_distance <= obs_parameter_)
        {
            task_1 = true;
            message.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Max Distance Reached");
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "Rotating Robot");

            // Construct the request
            auto request = std::make_shared<attach_shelf::srv::Rotate::Request>();
            request->degrees = dgs_parameter_;

            // Send the request to the service server
            auto result = rotate_client_->async_send_request(request);
            result.wait(); // Esperar a que la solicitud se complete (de forma sincrónica)

            if (result.get()->result == "Rotation completed successfully")
            {
                RCLCPP_INFO(this->get_logger(), "Service call succeeded");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service call failed");
            }
            
        }
        
    }
    
    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {   
        // Getiting Front Distance 
        //if (front_distance > obs_parameter_)
        //{
        front_distance = msg->ranges[540];
        RCLCPP_WARN(this->get_logger(), "Front Distance: %f", front_distance);
        //} 
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

    // Define Service Client
    rclcpp::Client<attach_shelf::srv::Rotate>::SharedPtr rotate_client_;

    // Variables 
    float front_distance = 0.00;
    float current_theta = 0.0;
    float request_angle = 0.0;
    bool task_1 = false;
    bool task_2 = false;
    

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
