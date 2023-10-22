#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <queue> // Include the header for deque
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#include <iostream>
#include <cstring>

class FezBitBotBase : public rclcpp::Node
{
public:
    FezBitBotBase()
        : Node("fezbitbot_base_node")
    {
        this->declare_parameter("hostname", "Robot01.local");
        std::string hostname = this->get_parameter("hostname").as_string();

        this->declare_parameter("wheel_radius", 0.02168);
        wheel_radius = this->get_parameter("wheel_radius").as_double();

        this->declare_parameter("wheel_distance", 0.04873);
        wheel_distance = this->get_parameter("wheel_distance").as_double();

        struct hostent *hostInfo;
        struct in_addr **addrList;
        hostInfo = gethostbyname(hostname.c_str());
        if (hostInfo == nullptr)
        {
            RCLCPP_ERROR(rclcpp::get_logger("FezBitBotBase"), "Can not find %s on the net", hostname.c_str());
            rclcpp::shutdown();
            exit(EXIT_FAILURE);
        }

        addrList = (struct in_addr **)hostInfo->h_addr_list;
        std::string device_ip = inet_ntoa(*addrList[0]);
        int device_port = 12345;

        // Subscribe to the Twist topic
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, [this](const geometry_msgs::msg::Twist::SharedPtr msg)
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
        server_addr.sin_port = htons(device_port);
        inet_pton(AF_INET, device_ip.c_str(), &server_addr.sin_addr);

        // Connect to the device
        if (connect(tcp_socket_, (struct sockaddr *)&server_addr, sizeof(server_addr)) == -1)
        {
            perror("Error connecting to device");
            rclcpp::shutdown();
            close(tcp_socket_);
            exit(EXIT_FAILURE);
        }

        RCLCPP_INFO(this->get_logger(), "Connected to %s:%d over TCP.", device_ip.c_str(), device_port);

        send_timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&FezBitBotBase::send_data, this));
    }

    void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        try
        {
            // Convert the Twist message to wheel velocities
            float linear_velocity = static_cast<float>(msg->linear.x);
            float angular_velocity = static_cast<float>(msg->angular.z);

            float left_wheel_speed = (2 * linear_velocity - angular_velocity * wheel_distance) / (2 * wheel_radius);
            float right_wheel_speed = (2 * linear_velocity + angular_velocity * wheel_distance) / (2 * wheel_radius);

            // Create a byte array starting with 0xAA, 0x55
            std::vector<uint8_t> byteArray(11);
            byteArray[0] = 0xAA;
            byteArray[1] = 0x55;
            byteArray[2] = 0x01;

            // Convert the float values to byte arrays
            std::memcpy(&byteArray[3], &left_wheel_speed, sizeof(float));
            std::memcpy(&byteArray[7], &right_wheel_speed, sizeof(float));

            // Put the message in the queue
            data_queue_.push(byteArray);
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

                // Pop the first element from the queue
                std::vector<uint8_t> data = data_queue_.front();
                data_queue_.pop();

                // Identify the message type based on the third byte
                uint8_t msg_type = data[2];

                // Take action based on the message type
                switch (msg_type)
                {
                case 0x01: // Wheel speed message handling
                {
                    ssize_t sent_bytes = send(tcp_socket_, data.data(), data.size(), 0);

                    if (sent_bytes == -1)
                    {
                        perror("Error sending Wheel speed over TCP");
                    }
                    else
                    {
                        // Receive and process the server's response
                        uint8_t response_data[1024];
                        ssize_t received_bytes = recv(tcp_socket_, response_data, sizeof(response_data), 0);
                        if (received_bytes > 0)
                        {
                            RCLCPP_DEBUG(this->get_logger(), "Received Wheel speed response");
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
                break;

                default:
                    break;
                }
            }
            else
            {
                // If the queue is empty, send a heartbeat message
                std::vector<uint8_t> heartbeat_bytes = {0xAA, 0x55, 0x00};
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
                        RCLCPP_DEBUG(this->get_logger(), "Received HEARTBEAT response");
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
    std::vector<uint8_t> START_OF_MESSAGE;
    std::vector<uint8_t> HEARTBEAT_MESSAGE;
    std::vector<uint8_t> TWIST_MESSAGE_PREFIX;
    std::queue<std::vector<uint8_t>> data_queue_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr send_timer_;
    int tcp_socket_;
    float wheel_radius;
    float wheel_distance;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto comms = std::make_shared<FezBitBotBase>();
        rclcpp::spin(comms);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(rclcpp::get_logger("FezBitBotBase"), "%s", e.what());
    }

    rclcpp::shutdown();
    return 0;
}