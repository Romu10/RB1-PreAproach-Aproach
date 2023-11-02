#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

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

        // TransformBroadCaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

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

    double roundToNearestHundred(double number) 
    {
        double multiple = 100.0;
        double rounded = std::round(number / multiple) * multiple;
        return (rounded > number) ? (rounded - multiple) : rounded;
    }

    void calcularCoordenadas(double theta, double d, double *x, double *y) 
    {
        *x = d * cos(theta);
        *y = d * sin(theta);
    }

    double calcularDistancia(double x1, double y1, double x2, double y2) 
    {
        double dx = x1 - x2;
        double dy = y1 - y2;
        return sqrt(dx * dx + dy * dy);
    }

    geometry_msgs::msg::Point calcularPuntoMedio(double x1, double y1, double x2, double y2)
    {
        geometry_msgs::msg::Point puntoMedio;
        puntoMedio.x = (x1 + x2) / 2.0;
        puntoMedio.y = (y1 + y2) / 2.0;
        return puntoMedio;
    }

    void publishTransformBroadcasterMsg(const geometry_msgs::msg::Point& center_point)
    {
        tf2_ros::TransformBroadcaster tf_broadcaster(this->shared_from_this());

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = "robot_base_link"; // Reemplaza con tu marco base
        transformStamped.child_frame_id = "cart_frame";
        transformStamped.transform.translation.x = center_point.x + 0.2;
        transformStamped.transform.translation.y = center_point.y - 0.1;
        transformStamped.transform.translation.z = 0.0; // Suponiendo una transformación 2D
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster.sendTransform(transformStamped);
    }

private:
    void approachServiceCallback(
    const std::shared_ptr<attach_shelf::srv::GoToLoading::Request> req,
    std::shared_ptr<attach_shelf::srv::GoToLoading::Response> res)
    {
        RCLCPP_INFO(get_logger(), "The Service /approach_robot has been called");

        signal = req->attach_to_shelf;
        
        if (signal == true)
        {
            
            RCLCPP_INFO(get_logger(), "Calculating position for shelf's legs");
            std::this_thread::sleep_for(std::chrono::seconds(1));
            if (detected_leg_1_ == true && detected_leg_2_ == true)
            {
                res->complete = true; 
                RCLCPP_INFO(get_logger(), "Service detected both leg");

                // Calculating distance between points
                dist_p1_to_p2 = calcularDistancia(x_point_leg_1, y_point_leg_1, x_point_leg_2, y_point_leg_2);
                RCLCPP_INFO(get_logger(), "Distance between points is: %f", dist_p1_to_p2);

                // Calculating middle point
                midPoint = calcularPuntoMedio(x_point_leg_1, y_point_leg_1, x_point_leg_2, y_point_leg_2);
                RCLCPP_INFO(get_logger(), "Middle Point X: %f", midPoint.x);
                RCLCPP_INFO(get_logger(), "Middle Point Y: %f", midPoint.y);

                // Create and Publish the Transform
                publishTransformBroadcasterMsg(midPoint);
            }
            else if (detected_leg_1_ == true && detected_leg_2_ == false)  
            {
                res->complete = false;
                RCLCPP_INFO(get_logger(), "Service fail because it only detected one leg");
            }
            else if (detected_leg_1_ == false && detected_leg_2_ == true)
            {
                res->complete = false;
                RCLCPP_INFO(get_logger(), "Service fail because it only detected one leg");
            }

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
            if (intensities[i] > threshold && divided == false)
            {
                //RCLCPP_INFO(get_logger(), "Índice: %zu - Intensidad: %f", i, intensities[i]);
                
                // Round index 
                double rounded_ind = roundToNearestHundred(i);
                
                // Calcula el ángulo correspondiente al índice 'i'
                double angle_in_radians = msg->angle_min + i * msg->angle_increment;
                
                if (ref_ind == 0.0)
                {
                    ref_ind = rounded_ind;
                }

                // Verifica si la diferencia de índice con respecto al índice anterior 
                if (prev_index != -1 && rounded_ind == ref_ind) 
                {
                    group1_ind.push_back(i);
                    group1_ind_angle.push_back(angle_in_radians);
                } 
                else 
                {
                    group2_ind.push_back(i);
                    group2_ind_angle.push_back(angle_in_radians);
                }
                prev_index = rounded_ind;
            }
        }
        
        if (divided == false)
        {
            group2_ind.erase(group2_ind.begin());
            
            // Print results
            std::cout << "Grupo 1: ";
            for (int i : group1_ind) {
                std::cout << i << " ";
            }
            std::cout << std::endl;

            std::cout << "Grupo 2: ";
            for (int i : group2_ind) {
                std::cout << i << " ";
            }
            std::cout << std::endl;

            // Calculating average distance for leg 1
            for (size_t i : group1_ind) 
            {
                avg_distance_leg_1 += msg->ranges[i];
            }
            avg_distance_leg_1 /= group1_ind.size();
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Average Distance is: %f", avg_distance_leg_1);

            // Calculating average angle for leg 1
            angle_leg_1 = group1_ind_angle[group1_ind_angle.size()/2];
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Angle is: %f", angle_leg_1);

            // Calculating coordinates point for leg 1
            calcularCoordenadas(angle_leg_1, avg_distance_leg_1, &x_point_leg_1, &y_point_leg_1);
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 X: %f", x_point_leg_1);
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Y: %f", y_point_leg_1);

            // Calculating average distance for leg 1
            for (size_t i : group2_ind) 
            {
                avg_distance_leg_2 += msg->ranges[i];
            }
            avg_distance_leg_2 /= group2_ind.size();
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Average Distance is: %f", avg_distance_leg_2);

            // Calculating average angle for leg 1
            angle_leg_2 = group2_ind_angle[group2_ind_angle.size()/2];
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Angle is: %f", angle_leg_2);

            // Calculating coordinates point for leg 1
            calcularCoordenadas(angle_leg_2, avg_distance_leg_2, &x_point_leg_2, &y_point_leg_2);
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 X: %f", x_point_leg_2);
            RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Y: %f", y_point_leg_2);

            // Calculating middle point
            midPoint = calcularPuntoMedio(x_point_leg_1, y_point_leg_1, x_point_leg_2, y_point_leg_2);
            RCLCPP_INFO(get_logger(), "Middle Point X: %f", midPoint.x);
            RCLCPP_INFO(get_logger(), "Middle Point Y: %f", midPoint.y);

        }

        // Create and Publish the Transform
        //RCLCPP_INFO(get_logger(), "Publishing Transform");
        publishTransformBroadcasterMsg(midPoint);

        // Make run the legs detection just once 
        divided = true;

        if (signal == true)
        {
            // Verifing if first leg was detected
            if (avg_distance_leg_1 > 0)
            {
                detected_leg_1_ = true; 
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Distance Detected!");
            }
            else 
            {
                detected_leg_1_ = false;
                RCLCPP_INFO_ONCE(get_logger(), "No Distance Detected in for first shelf leg");
            }

            // Verifing if second leg was detected
            if (avg_distance_leg_2 > 0)
            {
                detected_leg_2_ = true;
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Distance Detected!");
            }
            else 
            {
                RCLCPP_INFO_ONCE(get_logger(), "No Distance Detected in for second shelf leg");
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

    // Define TransformBroadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    geometry_msgs::msg::Twist vel_msg_;
    geometry_msgs::msg::Point midPoint;
    float current_theta_;  
    float current_degrees_;
    float front_distance = 0.00;

    // First Group Vector
    std::vector<int> group1_ind;
    std::vector<double> group1_ind_angle;
    float avg_distance_leg_1;
    float angle_leg_1;
    double x_point_leg_1;
    double y_point_leg_1;

    // Second Group Vector
    std::vector<int> group2_ind;
    std::vector<double> group2_ind_angle;
    float avg_distance_leg_2;
    float angle_leg_2;
    double x_point_leg_2;
    double y_point_leg_2;

    // First Leg
    bool detected_leg_1_;

    // Second Leg
    bool detected_leg_2_; 

    // Service Call Variable
    bool signal;
    bool divided = false;
    int prev_index = -1;
    double ref_ind = 0.0;

    // Coordinate points
    double theta;
    double d;
    double x, y;
    double dist_p1_to_p2;
    

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
