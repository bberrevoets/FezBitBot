#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <deque>  // Include the header for deque
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <iostream>

class FezBitBotBase : public rclcpp::Node
{
public:
    FezBitBotBase(const std::string &device_ip, int device_port, const std::string &twist_topic)
        : Node("fezbitbot_base_node"), device_ip_(device_ip), device_port_(device_port), twist_topic_(twist_topic)
    {

        START_OF_MESSAGE = std::vector<uint8_t>{0xAA, 0x55};
        HEARTBEAT_MESSAGE = std::vector<uint8_t>{0x00};
        TWIST_MESSAGE_PREFIX = std::vector<uint8_t>{0x01};

        data_queue_ = std::deque<geometry_msgs::msg::Twist>();

        // Subscribe to the Twist topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            twist_topic_, 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
            { twist_callback(msg); });

        // Initialize the TCP socket
        tcp_socket_ = socket(AF_INET, SOCK_STREAM, 0);
        if (tcp_socket_ == -1)
        {
            perror("Error creating socket");
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        struct sockaddr_in server_addr;
        server_addr.sin_family = AF_INET;
        server_addr.sin_port = htons(device_port_);
        inet_pton(AF_INET, device_ip_.c_str(), &server_addr.sin_addr);

        // Connect to the device
        if (connect(tcp_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
        {
            perror("Error connecting to device");
            rclcpp::shutdown();
            close(tcp_socket_);
            exit(EXIT_FAILURE);
        }

        RCLCPP_INFO(this->get_logger(), "Connected to %s:%d over TCP.", device_ip_.c_str(), device_port_);

        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FezBitBotBase::send_data, this));
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        try
        {
            // Put the message in the queue
            data_queue_.push_back(*msg);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error putting data into the queue: %s", e.what());
        }
    }

    void send_data()
    {
        try
        {
            if (!data_queue_.empty())
            {
                // If there's data in the queue, send it
                geometry_msgs::msg::Twist data_to_send = data_queue_.front();
                data_queue_.pop_front();
                float linear_x = data_to_send.linear.x;
                float angular_z = data_to_send.angular.z;

                std::vector<uint8_t> twist_bytes;
                twist_bytes.insert(twist_bytes.end(), START_OF_MESSAGE.begin(), START_OF_MESSAGE.end());
                twist_bytes.insert(twist_bytes.end(), TWIST_MESSAGE_PREFIX.begin(), TWIST_MESSAGE_PREFIX.end());
                uint8_t linear_x_bytes[4];
                uint8_t angular_z_bytes[4];
                memcpy(linear_x_bytes, &linear_x, sizeof(linear_x));
                memcpy(angular_z_bytes, &angular_z, sizeof(angular_z));
                twist_bytes.insert(twist_bytes.end(), linear_x_bytes, linear_x_bytes + sizeof(linear_x));
                twist_bytes.insert(twist_bytes.end(), angular_z_bytes, angular_z_bytes + sizeof(angular_z));

                ssize_t sent_bytes = send(tcp_socket_, twist_bytes.data(), twist_bytes.size(), 0);

                if (sent_bytes == -1)
                {
                    perror("Error sending data over TCP");
                }
                else
                {
                    // Receive and process the server's response
                    uint8_t response_data[1024];
                    ssize_t received_bytes = recv(tcp_socket_, response_data, sizeof(response_data), 0);
                    if (received_bytes > 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Received TWIST response");
                    }
                    else if (received_bytes == 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Connection closed by remote host");
                        rclcpp::shutdown();
                    }
                    else
                    {
                        perror("Error receiving response data");
                    }
                }
            }
            else
            {
                // If the queue is empty, send a heartbeat message
                std::vector<uint8_t> heartbeat_bytes;
                heartbeat_bytes.insert(heartbeat_bytes.end(), START_OF_MESSAGE.begin(), START_OF_MESSAGE.end());
                heartbeat_bytes.insert(heartbeat_bytes.end(), HEARTBEAT_MESSAGE.begin(), HEARTBEAT_MESSAGE.end());
                ssize_t sent_bytes = send(tcp_socket_, heartbeat_bytes.data(), heartbeat_bytes.size(), 0);

                if (sent_bytes == -1)
                {
                    perror("Error sending heartbeat over TCP");
                }
                else
                {
                    // Receive and process the server's response
                    uint8_t response_data[1024];
                    ssize_t received_bytes = recv(tcp_socket_, response_data, sizeof(response_data), 0);
                    if (received_bytes > 0)
                    {
                        RCLCPP_INFO(this->get_logger(), "Received HEARTBEAT response");
                    }
                    else if (received_bytes == 0)
                    {
                        RCLCPP_ERROR(this->get_logger(), "Connection closed by remote host");
                        rclcpp::shutdown();
                    }
                    else
                    {
                        perror("Error receiving response data");
                    }
                }
            }
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Error sending data over TCP: %s", e.what());
        }
    }

private:
    std::string device_ip_;
    int device_port_;
    std::string twist_topic_;
    std::vector<uint8_t> START_OF_MESSAGE;
    std::vector<uint8_t> HEARTBEAT_MESSAGE;
    std::vector<uint8_t> TWIST_MESSAGE_PREFIX;
    std::deque<geometry_msgs::msg::Twist> data_queue_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    int tcp_socket_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::string hostname = "Robot01.local";
    try
    {
        std::string device_ip = "192.168.180.132"; // Change this to your device's IP
        int device_port = 12345;
        std::string twist_topic = "cmd_vel";

        auto comms = std::make_shared<FezBitBotBase>(device_ip, device_port, twist_topic);
        rclcpp::spin(comms);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FezBitBotBase"), "%s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}