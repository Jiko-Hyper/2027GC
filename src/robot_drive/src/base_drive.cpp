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
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"

namespace
{
constexpr uint8_t kTxDataSize = 7;
constexpr std::size_t kTxFrameSize = 2 + 1 + kTxDataSize + 1;
constexpr uint8_t kRxDataSize = 0x1b;
constexpr std::size_t kRxFrameSize = 2 + 1 + kRxDataSize + 1;
constexpr double kPi = 3.14159265358979323846;

int16_t to_int16(double value)
{
  const auto rounded = std::round(value);
  const auto minimum = static_cast<double>(std::numeric_limits<int16_t>::min());
  const auto maximum = static_cast<double>(std::numeric_limits<int16_t>::max());
  return static_cast<int16_t>(std::clamp(rounded, minimum, maximum));
}

void put_int16_le(
  std::array<uint8_t, kTxFrameSize> & frame, std::size_t offset, int16_t value)
{
  const auto raw = static_cast<uint16_t>(value);
  frame[offset] = static_cast<uint8_t>(raw & 0xffU);
  frame[offset + 1] = static_cast<uint8_t>((raw >> 8U) & 0xffU);
}

int16_t get_int16_le(const std::array<uint8_t, kRxFrameSize> & frame, std::size_t offset)
{
  const auto raw = static_cast<uint16_t>(
    static_cast<uint16_t>(frame[offset]) |
    (static_cast<uint16_t>(frame[offset + 1]) << 8U));
  const auto value = (raw & 0x8000U) == 0U ?
    static_cast<int32_t>(raw) : static_cast<int32_t>(raw) - 0x10000;
  return static_cast<int16_t>(value);
}

int32_t get_int32_le(const std::array<uint8_t, kRxFrameSize> & frame, std::size_t offset)
{
  const auto raw = static_cast<uint32_t>(frame[offset]) |
    (static_cast<uint32_t>(frame[offset + 1]) << 8U) |
    (static_cast<uint32_t>(frame[offset + 2]) << 16U) |
    (static_cast<uint32_t>(frame[offset + 3]) << 24U);
  const auto value = (raw & 0x80000000U) == 0U ?
    static_cast<int64_t>(raw) : static_cast<int64_t>(raw) - 0x100000000LL;
  return static_cast<int32_t>(value);
}
}  // namespace

class BaseDrive : public rclcpp::Node
{
public:
  BaseDrive()
  : Node("base_drive"),
    serial_port_(declare_parameter<std::string>("serial_port", "/dev/ttyUSB0")),
    odom_frame_id_(declare_parameter<std::string>("odom_frame_id", "odom")),
    base_frame_id_(declare_parameter<std::string>("base_frame_id", "base_link")),
    ctrl_(static_cast<uint8_t>(declare_parameter<int>("initial_ctrl", 0) & 0x03))
  {
    cmd_vel_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist::SharedPtr message) {
        vx_cm_s_ = to_int16(message->linear.x * 100.0);
        vy_cm_s_ = to_int16(message->linear.y * 100.0);
        wz_millirad_s_ = to_int16(message->angular.z * 1000.0);
      });

    ctrl_subscription_ = create_subscription<std_msgs::msg::UInt8>(
      "drive_ctrl", rclcpp::QoS(10),
      [this](const std_msgs::msg::UInt8::SharedPtr message) {
        ctrl_ = static_cast<uint8_t>(message->data & 0x03U);
      });

    odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    motor_errors_publisher_ =
      create_publisher<std_msgs::msg::UInt8MultiArray>("motor_errors", rclcpp::QoS(10));
    drive_state_publisher_ =
      create_publisher<std_msgs::msg::UInt8>("drive_state", rclcpp::QoS(10));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      [this]() {
        receive_frames();
        send_frame();
      });

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

    serial_fd_ = ::open(serial_port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    tty.c_iflag &=
      ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL | IXON | IXOFF | IXANY);
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
      rx_buffer_.clear();
    }
  }

  std::array<uint8_t, kTxFrameSize> make_frame() const
  {
    std::array<uint8_t, kTxFrameSize> frame{};
    frame[0] = 0xaa;
    frame[1] = 0x55;
    frame[2] = kTxDataSize;
    put_int16_le(frame, 3, vx_cm_s_);
    put_int16_le(frame, 5, vy_cm_s_);
    put_int16_le(frame, 7, wz_millirad_s_);
    frame[9] = ctrl_;

    uint8_t checksum = 0;
    // SUM8 includes the length byte and all data bytes, but not the frame header.
    for (std::size_t index = 2; index < 10; ++index) {
      checksum = static_cast<uint8_t>(checksum + frame[index]);
    }
    frame[10] = checksum;
    return frame;
  }

  void receive_frames()
  {
    if (!open_serial()) {
      return;
    }

    std::array<uint8_t, 256> input{};
    while (true) {
      const auto result = ::read(serial_fd_, input.data(), input.size());
      if (result > 0) {
        rx_buffer_.insert(rx_buffer_.end(), input.begin(), input.begin() + result);
        continue;
      }
      if (result == 0 || (result < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))) {
        break;
      }
      if (result < 0 && errno == EINTR) {
        continue;
      }

      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "串口 %s 读取失败: %s",
        serial_port_.c_str(), std::strerror(errno));
      close_serial();
      return;
    }

    parse_rx_buffer();
  }

  void parse_rx_buffer()
  {
    while (rx_buffer_.size() >= 2) {
      if (rx_buffer_[0] != 0x55U || rx_buffer_[1] != 0xaaU) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() < 3) {
        return;
      }
      if (rx_buffer_[2] != kRxDataSize) {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }
      if (rx_buffer_.size() < kRxFrameSize) {
        return;
      }

      uint8_t checksum = 0;
      for (std::size_t index = 2; index < kRxFrameSize - 1; ++index) {
        checksum = static_cast<uint8_t>(checksum + rx_buffer_[index]);
      }
      if (checksum != rx_buffer_[kRxFrameSize - 1]) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "STM32 上报帧校验失败: 计算值=0x%02x, 接收值=0x%02x",
          static_cast<unsigned int>(checksum),
          static_cast<unsigned int>(rx_buffer_[kRxFrameSize - 1]));
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      std::array<uint8_t, kRxFrameSize> frame{};
      std::copy_n(rx_buffer_.begin(), kRxFrameSize, frame.begin());
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kRxFrameSize);
      publish_stm32_frame(frame);
    }
  }

  void publish_stm32_frame(const std::array<uint8_t, kRxFrameSize> & frame)
  {
    const int32_t x_cm = get_int32_le(frame, 3);
    const int32_t y_cm = get_int32_le(frame, 7);
    const int32_t yaw_centidegrees = get_int32_le(frame, 11);
    const int16_t vx_cm_s = get_int16_le(frame, 15);
    const int16_t vy_cm_s = get_int16_le(frame, 17);
    const int16_t wz_millirad_s = get_int16_le(frame, 19);
    const double yaw_rad = static_cast<double>(yaw_centidegrees) * kPi / 18000.0;

    nav_msgs::msg::Odometry odometry;
    odometry.header.stamp = now();
    odometry.header.frame_id = odom_frame_id_;
    odometry.child_frame_id = base_frame_id_;
    odometry.pose.pose.position.x = static_cast<double>(x_cm) / 100.0;
    odometry.pose.pose.position.y = static_cast<double>(y_cm) / 100.0;
    odometry.pose.pose.orientation.z = std::sin(yaw_rad / 2.0);
    odometry.pose.pose.orientation.w = std::cos(yaw_rad / 2.0);
    odometry.twist.twist.linear.x = static_cast<double>(vx_cm_s) / 100.0;
    odometry.twist.twist.linear.y = static_cast<double>(vy_cm_s) / 100.0;
    odometry.twist.twist.angular.z = static_cast<double>(wz_millirad_s) / 1000.0;
    odom_publisher_->publish(odometry);

    std_msgs::msg::UInt8MultiArray motor_errors;
    motor_errors.data = {frame[25], frame[26], frame[27], frame[28]};
    motor_errors_publisher_->publish(motor_errors);

    std_msgs::msg::UInt8 drive_state;
    drive_state.data = frame[29];
    drive_state_publisher_->publish(drive_state);
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
  std::string odom_frame_id_;
  std::string base_frame_id_;
  int serial_fd_{-1};
  int16_t vx_cm_s_{0};
  int16_t vy_cm_s_{0};
  int16_t wz_millirad_s_{0};
  uint8_t ctrl_{0};
  std::vector<uint8_t> rx_buffer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr ctrl_subscription_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr motor_errors_publisher_;
  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drive_state_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseDrive>());
  rclcpp::shutdown();
  return 0;
}
