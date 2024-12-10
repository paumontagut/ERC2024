// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

#include <unistd.h>       // Used for UART
#include <sys/fcntl.h>    // Used for UART
#include <termios.h>      // Used for UART

const char *uart_target = "/dev/ttyACM0";
#define     NSERIAL_CHAR   256
#define     VMINX          1
#define     BAUDRATE       115200

// SETUP SERIAL WORLD
int fid = -1;
struct termios  port_options;   // Create the structure
unsigned char tx_buffer[NSERIAL_CHAR];

void openSerialPort(){
    tcgetattr(fid, &port_options);      // Get the current attributes of the Serial port

    //------------------------------------------------
    //  OPEN THE UART
    //------------------------------------------------
    // The flags (defined in fcntl.h):
    //  Access modes (use 1 of these):
    //          O_RDONLY - Open for reading only.
    //          O_RDWR   - Open for reading and writing.
    //          O_WRONLY - Open for writing only.
    //      O_NDELAY / O_NONBLOCK (same function)
    //               - Enables nonblocking mode. When set read requests on the file can return immediately with a failure status
    //                 if there is no input immediately available (instead of blocking). Likewise, write requests can also return
    //                             immediately with a failure status if the output can't be written immediately.
    //                 Caution: VMIN and VTIME flags are ignored if O_NONBLOCK flag is set.
    //      O_NOCTTY - When set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.fid = open("/dev/ttyTHS1", O_RDWR | O_NOCTTY | O_NDELAY);          //Open in non blocking read/write mode

    fid = open(uart_target, O_RDWR | O_NOCTTY );

    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    //usleep(1000000);  // 1 sec delay

    if (fid == -1)
    {
        printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
    }

//------------------------------------------------
    // CONFIGURE THE UART
    //------------------------------------------------
    // flags defined in /usr/include/termios.h - see http://pubs.opengroup.org/onlinepubs/007908799/xsh/termios.h.html
    //  Baud rate:
    //         - B1200, B2400, B4800, B9600, B19200, B38400, B57600, B115200,
    //           B230400, B460800, B500000, B576000, B921600, B1000000, B1152000,
    //           B1500000, B2000000, B2500000, B3000000, B3500000, B4000000
    //  CSIZE: - CS5, CS6, CS7, CS8
    //  CLOCAL - Ignore modem status lines
    //  CREAD  - Enable receiver
    //  IGNPAR = Ignore characters with parity errors
    //  ICRNL  - Map CR to NL on input (Use for ASCII comms where you want to auto correct end of line characters - don't use for bianry comms!)
    //  PARENB - Parity enable
    //  PARODD - Odd parity (else even)
    port_options.c_cflag &= ~PARENB;            // Disables the Parity Enable bit(PARENB),So No Parity
    port_options.c_cflag &= ~CSTOPB;            // CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit
    port_options.c_cflag &= ~CSIZE;                 // Clears the mask for setting the data size
    port_options.c_cflag |=  CS8;               // Set the data bits = 8
    port_options.c_cflag &= ~CRTSCTS;           // No Hardware flow Control
    port_options.c_cflag |=  CREAD | CLOCAL;                  // Enable receiver,Ignore Modem Control lines
    port_options.c_iflag &= ~(IXON | IXOFF | IXANY);          // Disable XON/XOFF flow control both input & output
    port_options.c_iflag &= ~(ICANON | ECHO | ECHOE | ISIG);  // Non Cannonical mode
    port_options.c_oflag &= ~OPOST;                           // No Output Processing

    port_options.c_lflag = 0;               //  enable raw input instead of canonical,

    port_options.c_cc[VMIN]  = VMINX;       // Read at least 1 character
    port_options.c_cc[VTIME] = 0;           // Wait indefinetly

    cfsetispeed(&port_options,BAUDRATE);    // Set Read  Speed
    cfsetospeed(&port_options,BAUDRATE);    // Set Write Speed

    // Set the attributes to the termios structure
    int att = tcsetattr(fid, TCSANOW, &port_options);

    if (att != 0 )
    {
        printf("\nERROR in Setting port attributes");
    }
    else
    {
        printf("\nSERIAL Port Good to Go.\n");
    }

    // Flush Buffers
    tcflush(fid, TCIFLUSH);
    tcflush(fid, TCIOFLUSH);

    usleep(500000);   // 0.5 sec delay
}

void sendStringToSerialPort(unsigned char* tx_buffer){
    // std::cout << "Length: " << std::strlen(reinterpret_cast<const char *>(tx_buffer)) << std::endl;    //unsigned char *p_tx_buffer;
    int length = std::strlen(reinterpret_cast<const char *>(tx_buffer));

    // printf("fid 1=%d\n", fid );

    if (fid != -1)
    {
        //int count = write(fid, &tx_buffer[0], (p_tx_buffer - &tx_buffer[0]));         //Filestream, bytes to write, number of bytes to write
        int count = write(fid, &tx_buffer[0], length);          //Filestream, bytes to write, number of bytes to write
        tcflush(fid, TCIOFLUSH);

        // usleep(1000);   // .001 sec delay

        // printf("Count = %d\n", count);

        if (count < 0)  printf("UART TX error\n");
    }

    // usleep(1000000);  // 1 sec delay

}

std::string upper(std::string text){
    std::string upperCase;
    for(int it : text){
        if(it>96&&it<123){
            upperCase += char(it-32);
        }else{
            upperCase += char(it);
        }
    }
    return upperCase;
}

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minicernbot2_server")
  {
    std::cout << "test_serial_port" << std::endl;
    openSerialPort();

    unsigned char tx_buffer[NSERIAL_CHAR] = "={\"LINVELX\":0.01, \"LINVELY\": 0.01, \"ANGVELZ\":0.0}\n";
    sendStringToSerialPort(tx_buffer);

    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
  {
    float x=msg->linear.x, y=msg->linear.y, z=msg->angular.z;
    std::string commandToSerial = "={\"LINVELX\":"+ std::to_string(x)+", \"LINVELY\":"+std::to_string(y)+", \"ANGVELZ\":"+std::to_string(z)+"}\n";
    // std::cout << commandToSerial;
    strcpy(reinterpret_cast<char *>(tx_buffer), commandToSerial.c_str());
    sendStringToSerialPort(tx_buffer);
  }
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
