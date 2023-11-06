#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/detail/rotate__struct.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/subscription.hpp"
#include <rclcpp/rclcpp.hpp>

#include <cmath>
#include <chrono>
#include <functional>

#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <string>
#include "attach_shelf/srv/rotate.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"
#include "rclcpp/wait_result_kind.hpp"

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
        this->declare_parameter<double>("obstacle", 0.0, obstacle_param_desc);

      // Turning Parameter Configuration
        auto degrees_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        degrees_param_desc.description = "Sets the degrees for rotate the robot.";
        this->declare_parameter<int>("degrees", 0, degrees_param_desc);
      
      // Turning Parameter Configuration
        auto final_approach_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
        final_approach_param_desc.description = "Sets if you want to final approach.";
        this->declare_parameter<bool>("final_approach", "", final_approach_param_desc);
      
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

      // Service Client for /approach_shelf  
        approach_client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
      
      // Servie Client for /rotate_robot
        rotate_client_ = create_client<attach_shelf::srv::Rotate>("/rotate_robot");

    }
    
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "=========================================");
        // Getting linear velocity parameter data
        this->get_parameter("obstacle", obs_parameter_);
        RCLCPP_INFO(this->get_logger(), "Obstacle parameter is: %f", obs_parameter_);

        // Getting angular velocity parameter data 
        this->get_parameter("degrees", dgs_parameter_);
        RCLCPP_INFO(this->get_logger(), "Degrees parameter is: %i", dgs_parameter_);

        // Getting final approach parameter data 
        this->get_parameter("final_approach", fa_parameter_);
        if (fa_parameter_ == true) 
        {
            RCLCPP_INFO(this->get_logger(), "Final Approach parameter is set to: true");
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Final Approach parameter is set to: false");
        }
        RCLCPP_INFO(this->get_logger(), "=========================================");

        // Printing Front Distance 
        RCLCPP_INFO(this->get_logger(), "- Front Robot Distance: %f", front_distance);
        
        // Parameters Set Info
        if (obs_parameter_ < 0.1 && dgs_parameter_ < 1 )
        {
            RCLCPP_WARN(this->get_logger(), "Please Set Obstacle & Deegres Parameters & Final Approach");
        }

        if (obs_parameter_ != 0.0 && task_1 == false)
        {
            message.linear.x = 0.2;
            publisher_->publish(message);
            RCLCPP_INFO(this->get_logger(), "- Moving Forward");
        }
        
        if (obs_parameter_ != 0.0 && front_distance <= obs_parameter_)
        {
            task_1 = true;
            message.linear.x = 0.0;
            RCLCPP_INFO(this->get_logger(), "Max Distance Reached");
            publisher_->publish(message);

            // Wait until rotate service server is ready.
            while (!rotate_client_->wait_for_service(1s) && rotate_srv_done_ == false) 
            {
                if (!rclcpp::ok()) 
                {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Rotate service - Interrupted while waiting for the service. Exiting.");
                    return;
                }       
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rotate service not available, waiting again...");
                
            }
            
            if (rotate_srv_done_ == true)
            {
                RCLCPP_INFO(this->get_logger(), "Rotate Service Completed");
                RCLCPP_INFO(this->get_logger(), "Rotate Result: %s", glb_rotate_result_.c_str());
            }
            else if (rotate_requiered == false)
            {
                // Construct the rotate request
                auto rotate_request = std::make_shared<attach_shelf::srv::Rotate::Request>();
                rotate_request->degrees = dgs_parameter_;

                // Send to the rotate service server the request
                auto result_future = rotate_client_->async_send_request(rotate_request, std::bind(&VelParam::rotate_response_callback, this, std::placeholders::_1)); 

                // Send just once
                rotate_requiered = true;
            }

        }
        else
        {
            if (glb_rotate_result_ != "Rotation complete")
            {
                RCLCPP_INFO_ONCE(this->get_logger(), "Rotate result Not Received Properly");
            }
            else if (glb_rotate_result_.empty())
            {
                RCLCPP_INFO(this->get_logger(), "Working in Rotate result");
            }
        }

        if (rotate_srv_done_ == true) 
        {
            RCLCPP_WARN(this->get_logger(), "- Rotate Service Complete");
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "- Rotate Service Incomplete");
        }

        if (rotate_srv_done_ == true)
        {
            if (fa_parameter_ == true)
            {
                // Wait until approach service server is ready.
                while (!approach_client_->wait_for_service(1s)) 
                {
                    if (!rclcpp::ok() && approach_srv_done_ == false) 
                    {
                        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Approach Service - Interrupted while waiting for the service. Exiting.");
                        return;
                    }
                    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Approach service not available, waiting again...");
                    approach_client_ = create_client<attach_shelf::srv::GoToLoading>("/approach_shelf");
                }

                if (approach_srv_done_ == true) 
                {
                    RCLCPP_WARN(this->get_logger(), "- Approach Service Complete");
                }
                else if (approach_required == false)
                {
                    // Construct the approach request
                    auto approach_request = std::make_shared<attach_shelf::srv::GoToLoading::Request>();
                    approach_request->attach_to_shelf = fa_parameter_;

                    // Send to the approach service server the request
                    auto approach_result_future = approach_client_->async_send_request(approach_request, std::bind(&VelParam::approach_response_callback, this, std::placeholders::_1));

                    // send just once
                    approach_required = true;
                }
                else 
                {
                    RCLCPP_WARN(this->get_logger(), "- Waiting for Approach Service to Complete");
                }
            }
            else 
            {
                RCLCPP_WARN(this->get_logger(), "- Approach Parameter setted to FALSE");
            }

        }
        
    }

    void rotate_response_callback(rclcpp::Client<attach_shelf::srv::Rotate>::SharedFuture future) 
    {
        auto status = future.wait_for(1s);
        auto rotate_result_ = future.get()->result;
        glb_rotate_result_ = rotate_result_;
        if (status == std::future_status::ready) 
        {
            rotate_srv_done_ = true;
            RCLCPP_INFO(this->get_logger(), "Rotate Result: %s", rotate_result_.c_str());
            RCLCPP_INFO(this->get_logger(), "Rotation completed successfully");
        } 
        else 
        {
            RCLCPP_INFO(this->get_logger(), "Rotate Service In-Progress...");
        }
    }

    void approach_response_callback(rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedFuture future) 
    {
        auto status = future.wait_for(1s);
        auto approach_result_ = future.get()->complete;
        glb_approach_result_ = approach_result_;
        if (status == std::future_status::ready) 
        {
            if (approach_result_ == true) 
            {
                approach_srv_done_ = true;
                RCLCPP_WARN(this->get_logger(), "Approach complete");
            } 
            else 
            {
                RCLCPP_WARN(this->get_logger(), "Approach incomplete");
            }
        } 
        else 
        {
            RCLCPP_WARN(this->get_logger(), "Approach Service In-Progress...");
        }
    }

    
    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {   
        front_distance = msg->ranges[540];
        //RCLCPP_WARN(this->get_logger(), "Front Distance: %f", front_distance);
    }
  
  private:

    // Define the parameter
    double obs_parameter_;
    int dgs_parameter_;
    bool fa_parameter_;

    // Define Timer 
    rclcpp::TimerBase::SharedPtr timer_;

    // Define Twist 
    geometry_msgs::msg::Twist message;

    // Define the Publisher for cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Define Service Client for Rotate
    rclcpp::Client<attach_shelf::srv::Rotate>::SharedPtr rotate_client_;

    // Define Service Client for Approach
    rclcpp::Client<attach_shelf::srv::GoToLoading>::SharedPtr approach_client_;

    // Variables 
    float front_distance = 0.00;
    float current_theta = 0.0;
    float request_angle = 0.0;
    bool task_1 = false;
    bool task_2 = false;
    bool rotate_srv_done_ = false; 
    bool approach_srv_done_ = false;
    std::string glb_rotate_result_;
    bool glb_approach_result_; 
    bool rotate_requiered = false;
    bool approach_required = false;

};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    std::shared_ptr<VelParam> VelParam_node = std::make_shared<VelParam>();
    //rclcpp::executors::MultiThreadedExecutor executor;
    //executor.add_node(VelParam_node);
    //executor.spin();

    rclcpp::spin(VelParam_node);

    rclcpp::shutdown();
    return 0;
}
