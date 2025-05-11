#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <string>
#include <cerrno>

class ESP32Bridge : public rclcpp::Node {
public:
    ESP32Bridge() : Node("esp32_bridge") {
        cmd_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (cmd_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create cmd socket: %s", strerror(errno));
            rclcpp::shutdown();
        }
        cmd_addr_.sin_family = AF_INET;
        cmd_addr_.sin_port = htons(12347);
        inet_pton(AF_INET, "192.168.121.1", &cmd_addr_.sin_addr);  // Update IP if needed

        odom_sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (odom_sock_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create odom socket: %s", strerror(errno));
            rclcpp::shutdown();
        }
        struct sockaddr_in odom_addr;
        odom_addr.sin_family = AF_INET;
        odom_addr.sin_port = htons(12346);
        odom_addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(odom_sock_, (struct sockaddr*)&odom_addr, sizeof(odom_addr)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to bind odom socket: %s", strerror(errno));
            rclcpp::shutdown();
        }

        cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&ESP32Bridge::cmdVelCallback, this, std::placeholders::_1));
        odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&ESP32Bridge::odomCallback, this));
        RCLCPP_INFO(this->get_logger(), "ESP32 Bridge initialized");
    }

    ~ESP32Bridge() {
        close(cmd_sock_);
        close(odom_sock_);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        std::string packet = std::to_string(msg->linear.x) + "," + std::to_string(msg->angular.z);
        if (sendto(cmd_sock_, packet.c_str(), packet.size(), 0, (struct sockaddr*)&cmd_addr_, sizeof(cmd_addr_)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to send cmd_vel: %s", strerror(errno));
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent cmd_vel: %s", packet.c_str());
        }
    }

    void odomCallback() {
        char buffer[64];
        struct sockaddr_in sender_addr;
        socklen_t addr_len = sizeof(sender_addr);
        int len = recvfrom(odom_sock_, buffer, sizeof(buffer) - 1, MSG_DONTWAIT,
                          (struct sockaddr*)&sender_addr, &addr_len);
        if (len > 0) {
            buffer[len] = '\0';
            float x, y, yaw;
            if (sscanf(buffer, "%f,%f,%f", &x, &y, &yaw) == 3) {
                RCLCPP_INFO(this->get_logger(), "Received odom: %s", buffer);

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

                geometry_msgs::msg::TransformStamped tf;
                tf.header.stamp = this->now();
                tf.header.frame_id = "odom";
                tf.child_frame_id = "base_link";
                tf.transform.translation.x = x;
                tf.transform.translation.y = y;
                tf.transform.rotation = odom_msg.pose.pose.orientation;
                tf_broadcaster_->sendTransform(tf);
            } else {
                RCLCPP_WARN(this->get_logger(), "Invalid odom format: %s", buffer);
            }
        } else if (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            RCLCPP_ERROR(this->get_logger(), "UDP recvfrom error: %s", strerror(errno));
        }
    }

    int cmd_sock_, odom_sock_;
    struct sockaddr_in cmd_addr_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ESP32Bridge>());
    rclcpp::shutdown();
    return 0;
}
