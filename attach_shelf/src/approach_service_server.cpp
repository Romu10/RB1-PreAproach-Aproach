#include "attach_shelf/srv/detail/go_to_loading__struct.hpp"
#include "attach_shelf/srv/go_to_loading.hpp"

#include "geometry_msgs/msg/detail/point__struct.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2_ros/transform_broadcaster.h"
#include "tf2/exceptions.h"
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/transform_datatypes.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/static_transform_broadcaster.h"

#include "rclcpp/executors.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/qos.hpp>
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

    
        error_yaw_ = 0.0;
        kp_distance_ = 0.2;
        kp_yaw_ = 0.2;

        RCLCPP_INFO(get_logger(), "Approach Service Server Configured");

        // TransformBroadCaster
        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Inicializa el buffer y el oyente TF2
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this);

        // ROS Publishers
        publisher_ = create_publisher<geometry_msgs::msg::Twist>("/robot/cmd_vel", 10);

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
        publisher_->publish(vel_msg_);
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

    void publishTransformBroadcasterMsg(const geometry_msgs::msg::Point& center_point , std::string frame_child_name, std::string frame_header_name = "robot_front_laser_link")
    {
        //tf2_ros::TransformBroadcaster tf_broadcaster(this->shared_from_this());

        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.stamp = this->now();
        transformStamped.header.frame_id = frame_header_name; // Reemplaza con tu marco base
        transformStamped.child_frame_id = frame_child_name;
        transformStamped.transform.translation.x = center_point.x;
        transformStamped.transform.translation.y = center_point.y;
        transformStamped.transform.translation.z = 0.0; // Suponiendo una transformación 2D
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.0;
        transformStamped.transform.rotation.w = 1.0;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    void goToTransform()
    {
        // Intenta obtener la transformación más reciente
        geometry_msgs::msg::TransformStamped transform;
        try 
        {
            transform = tf_buffer_->lookupTransform("robot_front_laser_link", "cart_frame", tf2::TimePoint());
        } 
        catch (tf2::TransformException &ex) 
        {
            RCLCPP_ERROR(get_logger(), "Error al obtener la transformación: %s", ex.what());
        return;
        }

        // Calcula la distancia y el error angular
        double dx = transform.transform.translation.x;
        double dy = transform.transform.translation.y;
        error_distance_ = sqrt(dx * dx + dy * dy);
        RCLCPP_INFO(get_logger(), "Distance: %f m", error_distance_);

        // Convierte la rotación de geometry_msgs::msg::Quaternion a tf2::Quaternion
        tf2::Quaternion rotation(
        transform.transform.rotation.w, transform.transform.rotation.x,
        transform.transform.rotation.y, transform.transform.rotation.z);

        // Calcula el error angular
        double q_x = transform.transform.rotation.x;
        double q_y = transform.transform.rotation.y;
        double q_z = transform.transform.rotation.z;
        double q_w = transform.transform.rotation.w;

        double yaw = atan2(2.0 * (q_x * q_y + q_w * q_z), q_w * q_w + q_x * q_x - q_y * q_y - q_z * q_z);

        // Calculate the orientation error
        error_yaw_ = desired_yaw_ - yaw;
        RCLCPP_INFO(get_logger(), "Yaw: %f", yaw);

        // Calcula la velocidad lineal y angular
        double linear_velocity = kp_distance_ * error_distance_;
        RCLCPP_INFO(get_logger(), "Linear Velocity: %f m", linear_velocity);
        double angular_velocity = kp_yaw_ * error_yaw_;
        RCLCPP_INFO(get_logger(), "Angular Velocity: %f m", angular_velocity);

        // Publish the linear and angular velocity
        auto twist_msg = std::make_unique<geometry_msgs::msg::Twist>();
        if (error_distance_ < 0.1)
        {
            twist_msg->linear.x = 0.00;
            twist_msg->angular.z = 0.00;
            RCLCPP_INFO(get_logger(), "RB1 in Position");
        }
        else 
        {
            twist_msg->linear.x = linear_velocity;
            twist_msg->angular.z = angular_velocity;
            RCLCPP_INFO(get_logger(), "RB1 Moving to Position");
        }
        publisher_->publish(std::move(twist_msg));

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
            //std::this_thread::sleep_for(std::chrono::seconds(3));
            if (detected_leg_1_ == true && detected_leg_2_ == true)
            {
                RCLCPP_INFO(get_logger(), "Service detected both leg");

                /*
                // Make the robot move to the new transform 
                while (rclcpp::ok() && error_distance_ > 0.1)
                {
                    RCLCPP_WARN(get_logger(), "- Going to transform");
                    // goToTransform();
                }
                */

                if (error_distance_ < 0.1)
                {
                    res->complete = true; 
                }
                else if (error_distance_ > midPoint.x) 
                {
                    RCLCPP_INFO(get_logger(), "Robot Moving in wrong way");
                }
            }
            else if (detected_leg_1_ == true && detected_leg_2_ == false)  
            {
                RCLCPP_INFO(get_logger(), "Service fail because it only detected one leg");
                res->complete = false;
            }
            else if (detected_leg_1_ == false && detected_leg_2_ == true)
            {
                RCLCPP_INFO(get_logger(), "Service fail because it only detected one leg");
                res->complete = false;
            }

        }

        RCLCPP_INFO(get_logger(), "Finished service /approach_robot");
        //rclcpp::shutdown();
    }
    
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        current_theta_ = msg->pose.pose.orientation.z;  
        current_x_ = msg->pose.pose.orientation.x; 
        current_y_ = msg->pose.pose.orientation.y; 
        current_degrees_ = ((current_theta_ * 2) * 180) / M_PI;
        //RCLCPP_WARN(get_logger(), "Current Degrees: %f", current_degrees_);
    }

    void Laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) 
    {   
    
        front_distance = msg->ranges[540];
        //RCLCPP_WARN(this->get_logger(), "Front Distance: %f", front_distance);

        const std::vector<float>& intensities = msg->intensities;
        float threshold = 7500.0;
        //signal = true;
        
        if (signal == true)
        {

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
                group2_ind_angle.erase(group2_ind_angle.begin());
                
                // Print results
                std::cout << "Index Group 1: ";
                for (int i : group1_ind) {
                    std::cout << i << " ";
                }
                std::cout << std::endl;

                std::cout << "Index Group 2: ";
                for (double i : group1_ind_angle) {
                    std::cout << i << " ";
                }
                std::cout << std::endl;

                std::cout << "Angles Group 1: ";
                for (int i : group2_ind) {
                    std::cout << i << " ";
                }
                std::cout << std::endl;

                std::cout << "Angles Group 2: ";
                for (double i : group2_ind_angle) {
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
                for (size_t i : group1_ind_angle) 
                {
                    avg_angle_leg_1 += group1_ind_angle[i];
                }
                avg_angle_leg_1 /= group1_ind_angle.size();
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Average Angle is: %f", avg_angle_leg_1);

                // Calculating average angle for leg 1
                angle_leg_1 = avg_angle_leg_1 * 1;
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Angle is: %f", angle_leg_1);

                // Calculating coordinates point for leg 1
                calcularCoordenadas(angle_leg_1, avg_distance_leg_1, &x_point_leg_1, &y_point_leg_1);
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 X: %f", x_point_leg_1);
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 1 Y: %f", y_point_leg_1);
                pointLeg1.x = x_point_leg_1;
                pointLeg1.y = y_point_leg_1;

                // Calculating average distance for leg 2
                for (size_t i : group2_ind) 
                {
                    avg_distance_leg_2 += msg->ranges[i];
                }
                avg_distance_leg_2 /= group2_ind.size();
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Average Distance is: %f", avg_distance_leg_2);

                // Calculating average angle for leg 2
                for (size_t i : group2_ind_angle) 
                {
                    avg_angle_leg_2 += group2_ind_angle[i];
                }
                avg_angle_leg_2 /= group2_ind_angle.size();
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Average Angle is: %f", avg_angle_leg_2);

                // Calculating average angle for leg 2
                angle_leg_2 = avg_angle_leg_2 * 1;
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Angle is: %f", angle_leg_2);

                // Calculating coordinates point for leg 2
                calcularCoordenadas(angle_leg_2, avg_distance_leg_2, &x_point_leg_2, &y_point_leg_2);
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 X: %f", x_point_leg_2);
                RCLCPP_INFO_ONCE(get_logger(), "Shelf Leg 2 Y: %f", y_point_leg_2);
                pointLeg2.x = x_point_leg_2;
                pointLeg2.y = y_point_leg_2;

                // Calculating distance between points
                dist_p1_to_p2 = calcularDistancia(x_point_leg_2, y_point_leg_2, x_point_leg_1, y_point_leg_1);
                RCLCPP_INFO(get_logger(), "Distance between points is: %f", dist_p1_to_p2);

                // Calculating middle point
                midPoint = calcularPuntoMedio(x_point_leg_2, y_point_leg_2, x_point_leg_1, y_point_leg_1);
                // midPoint.x = midPoint.x + 0.20;
                RCLCPP_INFO(get_logger(), "Middle Point X: %f", midPoint.x);
                RCLCPP_INFO(get_logger(), "Middle Point Y: %f", midPoint.y);

                // Calculating distance between middle point and robot
                dist_rb1_to_midpoint = calcularDistancia(current_x_, current_y_, midPoint.x, midPoint.y);
                RCLCPP_INFO(get_logger(), "Distance between Robot and Middle Point is: %f", dist_rb1_to_midpoint);
                error_distance_ = dist_rb1_to_midpoint;

                // Create and Publish the Transforms
                publishTransformBroadcasterMsg(midPoint, "cart_frame");
                publishTransformBroadcasterMsg(pointLeg1, "leg1_frame");
                publishTransformBroadcasterMsg(pointLeg2, "leg2_frame");
                // worldPoint.x = current_x_ + midPoint.x;
                // worldPoint.y = current_y_ + midPoint.y;
                // publishTransformBroadcasterMsg(worldPoint, "cart_frame2", "robot_odom");
                RCLCPP_INFO_ONCE(get_logger(), "Transform Published");
            
            }

            // Make run the legs detection just once 
            divided = true;
        
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

            if (detected_leg_1_ == true && detected_leg_2_ == true)
            {
                // goToTransform();
            }
        }
    
    }

private:

    // Define Service Attach Service Message
    rclcpp::Service<attach_shelf::srv::GoToLoading>::SharedPtr service;
    
    // Define the Publisher to cmd_vel
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

    // Define the subscription to the Odom
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription2_;
    rclcpp::CallbackGroup::SharedPtr odom_callback_group_;

    // Define the subscription to the Laser Scan 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription1_;
    rclcpp::CallbackGroup::SharedPtr laser_callback_group_;

    // Define TransformBroadcaster
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
    
    // Define Transfor Listener
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // PID Variables for following the transform
    double error_distance_;
    double error_yaw_;
    double kp_distance_;
    double kp_yaw_;
    double desired_yaw_ = 0.00;

    geometry_msgs::msg::Twist vel_msg_;
    geometry_msgs::msg::Point worldPoint;
    geometry_msgs::msg::Point midPoint;
    geometry_msgs::msg::Point pointLeg1;
    geometry_msgs::msg::Point pointLeg2;
    float dist_rb1_to_midpoint;
    float current_theta_;  
    float current_x_;  
    float current_y_;  
    float current_degrees_;
    float front_distance = 0.00;

    // First Group Vector
    std::vector<int> group1_ind;
    std::vector<double> group1_ind_angle;
    float avg_distance_leg_1;
    float avg_angle_leg_1;
    float angle_leg_1;
    double x_point_leg_1;
    double y_point_leg_1;

    // Second Group Vector
    std::vector<int> group2_ind;
    std::vector<double> group2_ind_angle;
    float avg_distance_leg_2;
    float avg_angle_leg_2;
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
    // executor.add_node(MoveRB1_node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
