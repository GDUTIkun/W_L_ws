#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace wheel_leg_stm32_bridge {

class SerialPort {
 public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort &operator=(const SerialPort &) = delete;

  bool open(const std::string &device, int baud_rate, std::string &error);
  void close();
  bool isOpen() const { return fd_ >= 0; }
  std::ptrdiff_t read(std::uint8_t *data, std::size_t capacity, std::string &error);
  std::ptrdiff_t write(
    const std::uint8_t *data, std::size_t size, std::string &error);

 private:
  int fd_{-1};
};

}  // namespace wheel_leg_stm32_bridge
