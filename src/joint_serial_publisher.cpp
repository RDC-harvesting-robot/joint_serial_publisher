#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <thread>

class JointSerialPublisher : public rclcpp::Node {
public:
    JointSerialPublisher()
        : Node("joint_serial_publisher")
    {
        joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
        joint_state_.name = {"joint_0", "joint_1", "joint_2", "joint_3", "joint_4"};
        joint_state_.position.resize(5, 0.0);

        if (!initSerial("/dev/ttyACM0", B115200)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
            rclcpp::shutdown();
            return;
        }

        running_ = true;
        read_thread_ = std::thread(&JointSerialPublisher::readLoop, this);
    }

    ~JointSerialPublisher() {
        running_ = false;
        if (read_thread_.joinable()) {
            read_thread_.join();
        }
        if (fd_ >= 0) close(fd_);
    }

private:
    bool initSerial(const char* portname, int baudrate) {
        fd_ = open(portname, O_RDWR | O_NOCTTY | O_NONBLOCK); // 非ブロッキングモード
        if (fd_ < 0) return false;

        struct termios tty{};
        if (tcgetattr(fd_, &tty) != 0) return false;

        cfsetospeed(&tty, baudrate);
        cfsetispeed(&tty, baudrate);

        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
        tty.c_iflag = 0;
        tty.c_lflag = 0;
        tty.c_oflag = 0;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 1;

        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

        return tcsetattr(fd_, TCSANOW, &tty) == 0;
    }

    void readLoop() {
        char ch;
        char buffer[256];
        int idx = 0;

        while (rclcpp::ok() && running_) {
            ssize_t n = read(fd_, &ch, 1);
            if (n > 0) {
                if (ch == '\n') {
                    buffer[idx] = '\0';

                    char* endptr = nullptr;
                    double value = std::strtod(buffer, &endptr);
                    if (endptr != buffer) {
                        int joint_id = static_cast<int>(value / 1000);
                        double angle = value - joint_id * 1000;

                        if (joint_id >= 0 && joint_id < 5) {
                            if (angle != 0.0) {
                                joint_state_.position[joint_id] = angle;
                            }

                            joint_state_.header.stamp = this->get_clock()->now();
                            joint_pub_->publish(joint_state_);
                        }
                    }

                    idx = 0;
                } else if (ch != '\r' && idx < static_cast<int>(sizeof(buffer) - 1)) {
                    buffer[idx++] = ch;
                }
            } else {
                std::this_thread::sleep_for(std::chrono::microseconds(100));
            }
        }
    }

    int fd_;
    bool running_;
    std::thread read_thread_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    sensor_msgs::msg::JointState joint_state_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JointSerialPublisher>());
    rclcpp::shutdown();
    return 0;
}