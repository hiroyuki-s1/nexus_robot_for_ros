#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include <chrono>
#include <memory>
#include <thread>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

using namespace std::chrono_literals;

class NexusDriver : public rclcpp::Node
{
public:
    NexusDriver()
    : Node("nexus_driver_node")
    {
        // UARTポートを開く
        open_serial("/dev/ttyUSB0", B115200);

        // cmd_velの購読
        cmd_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&NexusDriver::cmd_vel_callback, this, std::placeholders::_1)
        );

        // odomの配信
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // UART読み取り用スレッド
        reader_thread_ = std::thread([this]() { this->uart_reader(); });

        RCLCPP_INFO(this->get_logger(), "nexus_driver_node started");
    }

    ~NexusDriver()
    {
        running_ = false;
        if (reader_thread_.joinable()) reader_thread_.join();
        if (fd_ >= 0) close(fd_);
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::thread reader_thread_;
    int fd_ = -1;
    bool running_ = true;

    void open_serial(const std::string &device, speed_t baud)
    {
        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to open serial port: %s", device.c_str());
            rclcpp::shutdown();
            return;
        }

        struct termios tty;
        tcgetattr(fd_, &tty);
        cfsetospeed(&tty, baud);
        cfsetispeed(&tty, baud);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8; // 8bit
        tty.c_iflag = IGNBRK;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = 10; // 1秒
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD); // no parity
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;

        tcsetattr(fd_, TCSANOW, &tty);
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 例：TwistメッセージをUARTに文字列で送信
        std::string command = "V " +
            std::to_string(msg->linear.x) + " " +
            std::to_string(msg->angular.z) + "\n";
        write(fd_, command.c_str(), command.size());
    }
    void uart_reader()
    {
        constexpr size_t PACKET_SIZE = 22;
        uint8_t buffer[PACKET_SIZE];
        size_t index = 0;
        
        while (running_) {
            uint8_t byte;
            int n = read(fd_, &byte, 1);
            if (n <= 0) {
                std::this_thread::sleep_for(10ms);
                continue;
            }
    
            if (index == 0 && byte != 0x02) continue;  // STX待ち
            buffer[index++] = byte;
    
            if (index < PACKET_SIZE) continue;
            index = 0;
    
            // チェック：ETX & チェックサム
            if (buffer[21] != 0x03) continue;
    
            uint8_t checksum = 0;
            for (int i = 1; i < 1 + 1 + 2 + 16; i++) {
                checksum += buffer[i];
            }
            if (checksum != buffer[20]) continue;
    
            // データ種別チェック
            if (buffer[1] != 0xA0) continue;
    
            // longデータ読み取り（リトルエンディアン）
            auto read_long = [&](int offset) -> int32_t {
                return static_cast<int32_t>(buffer[offset]) |
                       (static_cast<int32_t>(buffer[offset + 1]) << 8) |
                       (static_cast<int32_t>(buffer[offset + 2]) << 16) |
                       (static_cast<int32_t>(buffer[offset + 3]) << 24);
            };
    
            long pulse_ul = read_long(4);
            long pulse_ll = read_long(8);
            long pulse_lr = read_long(12);
            long pulse_ur = read_long(16);
    
            // オドメトリに変換・publish
            publish_odometry(pulse_ul, pulse_ll, pulse_lr, pulse_ur);
        }
    }

    void publish_odometry(long pul_ul, long pul_ll, long pul_lr, long pul_ur)
    {
        nav_msgs::msg::Odometry msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "odom";
        msg.child_frame_id = "base_link";

        // 仮の距離変換（例：1パルス = 1mm）
        double mm_per_pulse = 1.0;

        double x = ((pul_ul + pul_ll + pul_lr + pul_ur) / 4.0) * mm_per_pulse / 1000.0;
        double y = 0.0;      // 実際はエンコーダの向きとレイアウトに依存
        double theta = 0.0;

        msg.pose.pose.position.x = x;
        msg.pose.pose.position.y = y;
        msg.pose.pose.orientation.w = 1.0;

        odom_pub_->publish(msg);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NexusDriver>());
    rclcpp::shutdown();
    return 0;
}
