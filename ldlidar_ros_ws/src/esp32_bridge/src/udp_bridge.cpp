#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>

class UDPBridge : public rclcpp::Node {
public:
    UDPBridge() : Node("udp_bridge") {
        // Command socket (send to ESP32)
        cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (cmd_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create cmd socket");
            rclcpp::shutdown();
        }
        cmd_addr_.sin_family = AF_INET;
        cmd_addr_.sin_port = htons(12347);
        inet_pton(AF_INET, "192.168.121.xxx", &cmd_addr_.sin_addr);  // Replace xxx with ESP32 IP

        // Odometry socket (receive from ESP32)
        odom_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (odom_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create odom socket");
            rclcpp::shutdown();
        }
        odom_addr_.sin_family = AF_INET;
        odom_addr_.sin_port = htons(12346);
        odom_addr_.sin_addr.s_addr = INADDR_ANY;
        if (bind(odom_sock_, (struct sockaddr*)&odom_addr_, sizeof(odom_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind odom socket");
            rclcpp::shutdown();
        }

        cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&UDPBridge::cmd_callback, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&UDPBridge::read_odom, this));
    }

    ~UDPBridge() {
        close(cmd_sock_);
        close(odom_sock_);
    }

private:
    void cmd_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::string command = std::to_string(msg->linear.x) + "," + std::to_string(msg->angular.z);
        sendto(cmd_sock_, command.c_str(), command.size(), 0, (struct sockaddr*)&cmd_addr_, sizeof(cmd_addr_));
        RCLCPP_INFO(this->get_logger(), "Sent: %s", command.c_str());
    }

    void read_odom() {
        char buffer[64];
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        int len = recvfrom(odom_sock_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                          (struct sockaddr*)&sender_addr, &addr_len);
        if (len > 0) {
            buffer[len] = '\0';
            std::string data(buffer);
            float x = std::stof(data.substr(0, data.find(",")));
            data = data.substr(data.find(",") + 1);
            float y = std::stof(data.substr(0, data.find(",")));
            float yaw = std::stof(data.substr(data.find(",") + 1));

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

    int cmd_sock_, odom_sock_;
    struct sockaddr_in cmd_addr_, odom_addr_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UDPBridge>());
    rclcpp::shutdown();
    return 0;
}
