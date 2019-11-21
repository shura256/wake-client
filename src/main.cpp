#include <iostream>
#include <memory>
#include <string>

#include <cassert>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

typedef struct uart_port_options {
  uart_port_options(std::string device, uint32_t baud_rate, uint8_t word_length,
                    uint8_t stop_bits, std::string parity)
      : device(device),
        baud_rate(baud_rate),
        word_length(word_length),
        stop_bits(stop_bits),
        parity(parity) {}
  std::string device;
  uint32_t baud_rate;
  uint8_t word_length;
  uint8_t stop_bits;
  std::string parity;
} uart_port_options_t;

std::ostream& operator<<(std::ostream& out, const uart_port_options_t* value) {
  out << "Port: " << value->device << std::endl
      << "BaudRate: " << value->baud_rate << std::endl
      << "WordLength: " << (int)value->word_length << std::endl
      << "StopBits: " << (int)value->stop_bits << std::endl
      << "Parity: " << value->parity << std::endl;
  return out;
}

std::unique_ptr<uart_port_options_t> get_device_config() {
  return std::make_unique<uart_port_options_t>("/dev/ttyACM1", 115200, 8, 1,
                                               "none");
}

int setup_port(int port_fd) {
  struct termios tty;
  int speed = B115200;

  if (tcgetattr(port_fd, &tty) != 0) {
    std::cerr << "error " << errno << ", from tcgetattr(), " << strerror(errno) << std::endl;
    return -1;
  }

  if (cfsetispeed(&tty, speed) == -1) {
    std::cerr << "error " << errno << ", from cfsetispeed(), " << strerror(errno) << std::endl;
    return -1;
  }

  if (cfsetospeed(&tty, speed) == -1) {
    std::cerr << "error " << errno << ", from cfsetospeed(), " << strerror(errno) << std::endl;
    return -1;
  }

  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= (CS8);
  tty.c_cflag |= CRTSCTS;
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_lflag &= ~(IXON | IXOFF | IXANY);
  tty.c_oflag &= ~OPOST;

  if (tcsetattr(port_fd, TCSANOW, &tty) != 0) {
    std::cerr << "error " << errno << ", setting term attributes, " << strerror(errno) << std::endl;
    return -1;
  }

  return 0;
}

void print_menu() {
  std::cout << "Available actions:\n"
            << "\t1 - get blinking status\n"
            << "\t2 - turn on blinking\n"
            << "\t3 - turn off blinking\n"
            << "\t? - print this menu\n"
            << "\tq - quit\n";
}

int main() {
  auto port_options = get_device_config();
  std::cout << port_options.get();

  int ret;
  int serial_fd =
      open(port_options->device.c_str(), O_RDWR | O_NOCTTY);
  if (serial_fd < 0) {
    std::cerr << "error " << errno << ", openning " << port_options->device
              << ", " << strerror(errno) << std::endl;
    return -1;
  }

  ret = setup_port(serial_fd);
  if (ret < 0)
    return ret;

  uint8_t port_buffer[1];
  size_t buffer_size;
  ssize_t completed_size;
  bool continue_main_loop = true;
  bool continue_choise_loop;
  char user_choise;
  while (continue_main_loop) {
    print_menu();

    continue_choise_loop = true;
    while (continue_choise_loop) {
      std::cout << "Your choise: ";
      std::cin >> user_choise;

      switch (user_choise) {
        case '1':
        case '2':
        case '3':
        case '?':
        case 'q':
          continue_choise_loop = false;
          break;

        default:
          break;
      }
    }

    switch (user_choise) {
      case '1':
        port_buffer[0] = 1;
        buffer_size = 1;

        std::cout << "send blinking status request" << std::endl;
        completed_size = write(serial_fd, port_buffer, buffer_size);
        if (completed_size == -1)
          std::cerr << "send error " << errno << ", " << strerror(errno) << std::endl;
        assert((ssize_t)buffer_size == completed_size);

        buffer_size = 1;
        completed_size = read(serial_fd, (void*)port_buffer, buffer_size);
        if (completed_size == -1)
          std::cerr << "receive error " << errno << ", " << strerror(errno) << std::endl;
        assert((ssize_t)buffer_size == completed_size);
        std::cout << "blinking status: " << (int)port_buffer[0] << std::endl;

        break;

      case '2':
        port_buffer[0] = 2;
        buffer_size = 1;

        std::cout << "send turn on blinking request" << std::endl;
        completed_size = write(serial_fd, port_buffer, 1);
        if (completed_size == -1)
          std::cerr << "send error " << errno << ", " << strerror(errno) << std::endl;
        assert((ssize_t)buffer_size == completed_size);

        break;

      case '3':
        port_buffer[0] = 3;
        buffer_size = 1;

        std::cout << "send turn off blinking request" << std::endl;
        completed_size = write(serial_fd, port_buffer, 1);
        if (completed_size == -1)
          std::cerr << "send error " << errno << ", " << strerror(errno) << std::endl;
        assert((ssize_t)buffer_size == completed_size);

        break;

      case '?':
        break;

      case 'q':
        continue_main_loop = false;
        break;

      default:
        break;
    }
  }

  close(serial_fd);

  return 0;
}
