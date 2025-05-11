#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <SerialPort.h>
#include <string>

class SerialBridge : public rclcpp::Node {
public:
    SerialBridge() : Node("serial_bridge") {
        serial_.Open("/dev/ttyUSB0");  // Adjust port
        serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        if (!serial_.IsOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            rclcpp::shutdown();
        }
        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&SerialBridge::cmd_callback, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&SerialBridge::read_odom, this));
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::string command = std::to_string(msg->linear.x) + "," + std::to_string(msg->angular.z) + "\n";
        serial_.Write(command);
        RCLCPP_INFO(this->get_logger(), "Sent: %s", command.c_str());
    }

    void read_odom() {
        std::string line;
        if (serial_.IsDataAvailable()) {
            serial_.ReadLine(line);
            if (line.find("ODOM:") == 0) {
                line = line.substr(5);
                float x = std::stof(line.substr(0, line.find(",")));
                line = line.substr(line.find(",") + 1);
                float y = std::stof(line.substr(0, line.find(",")));
                float yaw = std::stof(line.substr(line.find(",") + 1));

                nav_msgs::msg::Odometry odom_msg;
                odom_msg.header.stamp = this->now();
                odom_msg.header.frame_id = "odom";
                odom_msg.child_frame_id = "base_link";
                odom_msg.pose.pose.position.x = x;
                odom_msg.pose.pose.position.y = y;
                tf2::Quaternion q;
                q.setRPY(0, 0, yaw);
                odom_msg.pose.pose.orientation.x = q.x();
                odom_msg.pose.pose.orientation.y = q.y();
                odom_msg.pose.pose.orientation.z = q.z();
                odom_msg.pose.pose.orientation.w = q.w();
                odom_pub_->publish(odom_msg);
            }
        }
    }

    LibSerial::SerialPort serial_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialBridge>());
    rclcpp::shutdown();
    return 0;
}
