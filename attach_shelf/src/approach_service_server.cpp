#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <functional>

class MoveRB1 : public rclcpp::Node
{
public:
    MoveRB1() : Node("attach_robot_node")
    {
        // ROS Services
        service = create_service<attach_shelf::srv::GoToLoading>(
            "approach_shelf",
            std::bind(&MoveRB1::approachServiceCallback, this, std::placeholders::_1, std::placeholders::_2)
        );

        RCLCPP_INFO(get_logger(), "Approach Service Server Configured");

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


        // Laser Sub
        laser_callback_group_ = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options1;
        options1.callback_group = laser_callback_group_;

        subscription1_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 10,
        std::bind(&MoveRB1::Laser_callback, this, std::placeholders::_1),
        options1);

    }

    void rotate_robot(float vel)
    {
        vel_msg_.angular.z = vel;
        vel_pub->publish(vel_msg_);
    }

private:
    void approachServiceCallback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res)
    {
        RCLCPP_INFO(get_logger(), "The Service /approach_robot has been called");

        bool signal = req->attach_to_shelf;
        
        if (signal == true)
        {
            res->complete = true; 
            RCLCPP_INFO(get_logger(), "Complete -> True");
        }
        else
        {
            res->complete = false;
            RCLCPP_INFO(get_logger(), "Complete -> False");
        }

        RCLCPP_INFO(get_logger(), "Finished service /approach_robot");
        //rclcpp::shutdown();
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_theta_ = msg->pose.pose.orientation.z;  
        current_degrees_ = ((current_theta_ * 2) * 180) / M_PI;
        //RCLCPP_WARN(get_logger(), "Current Degrees: %f", current_degrees_);
    }

    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {   
    
        front_distance = msg->ranges[540];
        //RCLCPP_WARN(this->get_logger(), "Front Distance: %f", front_distance);

        const std::vector<float>& intensities = msg->intensities;
        float threshold = 7500.0;

        for (size_t i = 0; i < intensities.size(); ++i)
        {
            if (intensities[i] > threshold)
            {
                //RCLCPP_INFO(get_logger(), "Índice: %zu - Intensidad: %f", i, intensities[i]);
            }
        }
    
    }

private:

    // Define Service Message
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service;
    
    // Define the Publisher to cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub;

    // Define the subscription to the Odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    geometry_msgs::msg::Twist vel_msg_;
    float current_theta_;  
    float current_degrees_;
    float front_distance = 0.00;
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
