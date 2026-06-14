// Minimal blocking serial port + line reader for the UGV ESP32 link.
//
// Mirrors the framing used by the legacy Python BaseController/ReadLine
// (ugv_bringup.py): newline-delimited JSON at 115200 8N1. POSIX termios only,
// no external dependencies.
#ifndef UGV_HARDWARE__SERIAL_PORT_HPP_
#define UGV_HARDWARE__SERIAL_PORT_HPP_

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

namespace ugv_hardware
{

class SerialPort
{
public:
  SerialPort() = default;

  ~SerialPort() { close_port(); }

  // Open the device and configure it as raw 8N1 at the given baud.
  // Returns false (with errno set) on failure rather than throwing, so the
  // hardware component can fail cleanly via CallbackReturn::ERROR.
  bool open_port(const std::string & device, int baud)
  {
    close_port();
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      return false;
    }

    struct termios tty;
    std::memset(&tty, 0, sizeof(tty));
    if (::tcgetattr(fd_, &tty) != 0) {
      close_port();
      return false;
    }

    speed_t speed = baud_to_speed(baud);
    ::cfsetospeed(&tty, speed);
    ::cfsetispeed(&tty, speed);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;  // 8 data bits
    tty.c_cflag |= (CLOCAL | CREAD);             // ignore modem ctrl, enable read
    tty.c_cflag &= ~(PARENB | PARODD);           // no parity
    tty.c_cflag &= ~CSTOPB;                       // 1 stop bit
    tty.c_cflag &= ~CRTSCTS;                       // no HW flow control

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);       // no SW flow control
    tty.c_iflag &= ~(ICRNL | INLCR | IGNCR);      // raw, do not translate CR/LF
    tty.c_lflag = 0;                              // raw input (no canonical/echo)
    tty.c_oflag = 0;                              // raw output

    tty.c_cc[VMIN] = 0;                           // non-blocking-ish reads
    tty.c_cc[VTIME] = 1;                          // 0.1 s read timeout

    if (::tcsetattr(fd_, TCSANOW, &tty) != 0) {
      close_port();
      return false;
    }
    ::tcflush(fd_, TCIOFLUSH);
    return true;
  }

  bool is_open() const { return fd_ >= 0; }

  void close_port()
  {
    if (fd_ >= 0) {
      ::close(fd_);
      fd_ = -1;
    }
    buffer_.clear();
  }

  // Write a raw string (caller appends the trailing '\n'). Returns bytes written.
  ssize_t write_str(const std::string & data)
  {
    if (fd_ < 0) {
      return -1;
    }
    return ::write(fd_, data.data(), data.size());
  }

  // Read one newline-terminated line into `line` (without the newline).
  // Returns true if a complete line was produced. Non-blocking-friendly: when no
  // full line is available yet it returns false so the caller can keep looping.
  bool read_line(std::string & line)
  {
    // Drain any already-buffered complete line first.
    auto nl = buffer_.find('\n');
    if (nl == std::string::npos) {
      char chunk[256];
      ssize_t n = ::read(fd_, chunk, sizeof(chunk));
      if (n > 0) {
        buffer_.append(chunk, static_cast<size_t>(n));
      }
      nl = buffer_.find('\n');
      if (nl == std::string::npos) {
        return false;
      }
    }
    line = buffer_.substr(0, nl);
    // Strip a trailing CR if the firmware sends CRLF.
    if (!line.empty() && line.back() == '\r') {
      line.pop_back();
    }
    buffer_.erase(0, nl + 1);
    return true;
  }

private:
  static speed_t baud_to_speed(int baud)
  {
    switch (baud) {
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      case 230400: return B230400;
      case 460800: return B460800;
      case 921600: return B921600;
      default: return B115200;
    }
  }

  int fd_{-1};
  std::string buffer_;
};

}  // namespace ugv_hardware

#endif  // UGV_HARDWARE__SERIAL_PORT_HPP_
