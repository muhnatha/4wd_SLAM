// include ros cpp necessary 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include <functional>
#include <memory>
#include <vector>

class controller: public rclcpp::Node
{
    public:
        // constructor
        controller(): Node("controller_node")
        {
            // get log info node starting
            RCLCPP_INFO(this->get_logger(), "Controller node started");

            // setup parameter (wheels radius and separation)
            this->declare_parameter<double>("wheel_radius", 0.05);
            this->declare_parameter<double>("wheel_separation", 0.34);
            this->get_parameter("wheel_radius", wheel_radius_);
            this->get_parameter("wheel_separation", wheel_separation_);

            // create a subscriber to /cmd_vel topic
            subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
                "/cmd_vel", 10, std::bind(&controller::cmd_vel_callback, this, std::placeholders::_1)
            );
            // create a subscriber that listen to "/cmd_vel" topic that has geometry_msgs::msg::Twist message type
            // If the object get the message, it will start the cmd_vel_callback function as the first argument (max 10)

            // create publisher for the wheels velocity
            fl_pub_ = this->create_publisher<std_msgs::msg::Float64>("/front_left_wheel_joint_velocity_controller/commands", 10);
            fr_pub_ = this->create_publisher<std_msgs::msg::Float64>("/front_right_wheel_joint_velocity_controller/commands", 10);
            bl_pub_ = this->create_publisher<std_msgs::msg::Float64>("/back_left_wheel_joint_velocity_controller/commands", 10);
            br_pub_ = this->create_publisher<std_msgs::msg::Float64>("/back_right_wheel_joint_velocity_controller/commands", 10);
        }
    
    private:
        void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
            // inverse kinematic
            double left_wheel_rad_s = (msg->linear.x - msg->angular.z * wheel_separation_ /2.0)/ wheel_radius_;
            double right_wheel_rad_s = (msg->linear.x + msg->angular.z * wheel_separation_/2.0)/wheel_radius_;

            // create messages
            auto left_vel_msg = std_msgs::msg::Float64();
            left_vel_msg.data = left_wheel_rad_s;

            auto right_vel_msg = std_msgs::msg::Float64();
            right_vel_msg.data = right_wheel_rad_s;

            // publish messages
            fl_pub_->publish(left_vel_msg);
            bl_pub_->publish(left_vel_msg);
            fr_pub_->publish(right_vel_msg);
            br_pub_->publish(right_vel_msg);
        }

        // variables
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fl_pub_, fr_pub_, bl_pub_, br_pub_;
        double wheel_radius_;
        double wheel_separation_;
};

int main(int argc, char*argv[])
{   
    // initialize ROS2
    rclcpp::init(argc, argv);

    // create node and keep it running
    rclcpp::spin(std::make_shared<controller>());

    // shutdown ROS2 when done
    rclcpp::shutdown();
    return 0;
}