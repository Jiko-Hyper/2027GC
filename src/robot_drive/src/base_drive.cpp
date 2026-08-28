#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <memory>
#include <string>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace
{
constexpr uint8_t kDataSize = 7;
constexpr std::size_t kFrameSize = 2 + 1 + kDataSize + 1;

int16_t to_int16(double value)
{
  const auto rounded = std::round(value);
  const auto minimum = static_cast<double>(std::numeric_limits<int16_t>::min());
  const auto maximum = static_cast<double>(std::numeric_limits<int16_t>::max());
  return static_cast<int16_t>(std::clamp(rounded, minimum, maximum));
}

void put_int16_le(std::array<uint8_t, kFrameSize> & frame, std::size_t offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  frame[offset] = static_cast<uint8_t>(raw & 0xffU);
  frame[offset + 1] = static_cast<uint8_t>((raw >> 8U) & 0xffU);
}
}  // namespace

class BaseDrive : public rclcpp::Node
{
public:
  BaseDrive()
  : Node("base_drive"),
    serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
    ctrl_(static_cast<uint8_t>(declare_parameter<int>("initial_ctrl", 0) & 0x03))
  {
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        vx_cm_s_ = to_int16(message->linear.x * 100.0);
        vy_cm_s_ = to_int16(message->linear.y * 100.0);
        wz_rad_s_ = to_int16(message->angular.z);
      });

    ctrl_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "drive_ctrl", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        ctrl_ = static_cast<uint8_t>(message->data & 0x03U);
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(20), [this]() {send_frame();});

    if (!open_serial()) {
      RCLCPP_WARN(
        get_logger(), "串口 %s 暂不可用，将在发送时自动重连", serial_port_.c_str());
    }
  }

  ~BaseDrive() override
  {
    close_serial();
  }

private:
  bool open_serial()
  {
    if (serial_fd_ >= 0) {
      return true;
    }

    serial_fd_ = ::open(serial_port_.c_str(), O_WRONLY | O_NOCTTY | O_NONBLOCK);
    if (serial_fd_ < 0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "打开串口 %s 失败: %s",
        serial_port_.c_str(), std::strerror(errno));
      return false;
    }

    termios tty{};
    if (tcgetattr(serial_fd_, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "读取串口配置失败: %s", std::strerror(errno));
      close_serial();
      return false;
    }

    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag |= CLOCAL | CREAD;
    tty.c_cflag &= ~(PARENB | CSTOPB | CRTSCTS);
    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON);
    tty.c_oflag &= ~OPOST;
    tty.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(serial_fd_, TCSANOW, &tty) != 0) {
      RCLCPP_ERROR(get_logger(), "设置串口为 115200 8N1 失败: %s", std::strerror(errno));
      close_serial();
      return false;
    }

    tcflush(serial_fd_, TCIOFLUSH);
    RCLCPP_INFO(get_logger(), "串口 %s 已打开（115200 8N1）", serial_port_.c_str());
    return true;
  }

  void close_serial()
  {
    if (serial_fd_ >= 0) {
      ::close(serial_fd_);
      serial_fd_ = -1;
    }
  }

  std::array<uint8_t, kFrameSize> make_frame() const
  {
    std::array<uint8_t, kFrameSize> frame{};
    frame[0] = 0xaa;
    frame[1] = 0x55;
    frame[2] = kDataSize;
    put_int16_le(frame, 3, vx_cm_s_);
    put_int16_le(frame, 5, vy_cm_s_);
    put_int16_le(frame, 7, wz_rad_s_);
    frame[9] = ctrl_;

    uint8_t checksum = 0;
    // SUM8 includes the length byte and all data bytes, but not the frame header.
    for (std::size_t index = 2; index < 10; ++index) {
      checksum = static_cast<uint8_t>(checksum + frame[index]);
    }
    frame[10] = checksum;
    return frame;
  }

  void send_frame()
  {
    if (!open_serial()) {
      return;
    }

    const auto frame = make_frame();
    std::size_t sent = 0;
    while (sent < frame.size()) {
      const auto result = ::write(serial_fd_, frame.data() + sent, frame.size() - sent);
      if (result > 0) {
        sent += static_cast<std::size_t>(result);
        continue;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }

      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "串口 %s 写入失败: %s",
        serial_port_.c_str(), std::strerror(errno));
      close_serial();
      return;
    }
  }

  std::string serial_port_;
  int serial_fd_{-1};
  int16_t vx_cm_s_{0};
  int16_t vy_cm_s_{0};
  int16_t wz_rad_s_{0};
  uint8_t ctrl_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseDrive>());
  rclcpp::shutdown();
  return 0;
}
