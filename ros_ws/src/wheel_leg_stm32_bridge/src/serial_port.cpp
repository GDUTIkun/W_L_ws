#include "wheel_leg_stm32_bridge/serial_port.hpp"

#include <cerrno>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace wheel_leg_stm32_bridge {
namespace {

bool baudConstant(int baud_rate, speed_t &speed) {
  switch (baud_rate) {
    case 115200:
      speed = B115200;
      return true;
#ifdef B230400
    case 230400:
      speed = B230400;
      return true;
#endif
#ifdef B460800
    case 460800:
      speed = B460800;
      return true;
#endif
#ifdef B921600
    case 921600:
      speed = B921600;
      return true;
#endif
#ifdef B1000000
    case 1000000:
      speed = B1000000;
      return true;
#endif
#ifdef B2000000
    case 2000000:
      speed = B2000000;
      return true;
#endif
    default:
      return false;
  }
}

}  // namespace

SerialPort::~SerialPort() { close(); }

bool SerialPort::open(
    const std::string &device, int baud_rate, std::string &error) {
  close();
  speed_t speed{};
  if (!baudConstant(baud_rate, speed)) {
    error = "unsupported baud rate: " + std::to_string(baud_rate);
    return false;
  }

  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (fd_ < 0) {
    error = std::strerror(errno);
    return false;
  }

  termios config{};
  if (tcgetattr(fd_, &config) != 0) {
    error = std::strerror(errno);
    close();
    return false;
  }
  cfmakeraw(&config);
  config.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
  config.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
  config.c_cflag &= static_cast<tcflag_t>(~PARENB);
  config.c_cflag &= static_cast<tcflag_t>(~CSIZE);
  config.c_cflag |= CS8;
#ifdef CRTSCTS
  config.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
#endif
  config.c_cc[VMIN] = 0;
  config.c_cc[VTIME] = 0;
  cfsetispeed(&config, speed);
  cfsetospeed(&config, speed);
  if (tcsetattr(fd_, TCSANOW, &config) != 0) {
    error = std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  error.clear();
  return true;
}

void SerialPort::close() {
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

std::ptrdiff_t SerialPort::read(
    std::uint8_t *data, std::size_t capacity, std::string &error) {
  const auto result = ::read(fd_, data, capacity);
  if (result >= 0) {
    error.clear();
    return result;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    error.clear();
    return 0;
  }
  error = std::strerror(errno);
  return -1;
}

std::ptrdiff_t SerialPort::write(
    const std::uint8_t *data, std::size_t size, std::string &error) {
  const auto result = ::write(fd_, data, size);
  if (result >= 0) {
    error.clear();
    return result;
  }
  if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
    error.clear();
    return 0;
  }
  error = std::strerror(errno);
  return -1;
}

}  // namespace wheel_leg_stm32_bridge
