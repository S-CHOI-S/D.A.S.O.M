// #include <string>
// #include <ros/ros.h>
// #include <serial/serial.h>
// #include <std_msgs/String.h>
// #include <std_msgs/Empty.h>
// #include <std_msgs/UInt16.h>
// #include <std_msgs/Float32.h>
// #include <sensor_msgs/JointState.h>

// serial::Serial _serial;				// serial object

// int main(int argc, char** argv)
// {	
// 	ros::init(argc, argv, "mirobot_write_node");//초기화, 노드이름(初始化，节点名称) "Mirobot_write_node"
// 	ros::NodeHandle nh;

// 	ros::Rate loop_rate(20);//주파수를 지정했습니다(指定了频率为)20Hz

// 	try//로봇팔의 직렬 연결 시도(尝试连接机械臂的串口)
// 	{
// 		_serial.setPort("/dev/ttyUSB0");
// 		_serial.setBaudrate(57600);
// 		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
// 		_serial.setTimeout(to);
// 		_serial.open();
// 		_serial.write("M50\r\n");
// 		ROS_INFO_STREAM("Port has been open successfully");
// 	}
// 	catch (serial::IOException& e)
// 	{
// 		ROS_ERROR_STREAM("Unable to open port");
// 		return -1;
// 	}
	
// 	if (_serial.isOpen())
// 	{
// 		ros::Duration(1).sleep();				
// 		ROS_INFO_STREAM("Attach and wait for commands");
// 	}

// 	while (ros::ok())
// 	{
// 		ros::spinOnce();
// 		loop_rate.sleep();
// 	}
	
// 	return 0;
// }

// #include <iostream>
// #include <string>
// #include <boost/asio.hpp>

// using namespace std;
// namespace asio = boost::asio;
// using asio::serial_port;
// using asio::serial_port_base;
// using asio::system_error;
// using boost::system::error_code;
// using std::cerr;
// using std::cout;
// using std::endl;

// int main(int argc, char** argv) {
//     if (argc < 3) {
//         cerr << "Usage: " << argv[0] << " port my_adr" << endl;
//         return 1;
//     }

//     const std::string serialPort = argv[1];
//     int my_address = std::stoi(argv[2]);

//     try {
//         asio::io_service io;
//         serial_port port(io, serialPort);

//         // Configure serial port settings
//         port.set_option(serial_port_base::baud_rate(57600));
//         port.set_option(serial_port_base::character_size(8));
//         port.set_option(serial_port_base::stop_bits(serial_port_base::stop_bits::one));
//         port.set_option(serial_port_base::parity(serial_port_base::parity::none));

//         // Connect to the XBee
//         std::string cmd = "+++";
//         port.write_some(asio::buffer(cmd, cmd.size()));
//         usleep(1000000); // Sleep for 1 second

//         // Check if connected to the XBee
//         char response[3];
//         port.read_some(asio::buffer(response, sizeof(response)));
//         if (response[0] == 'O' && response[1] == 'K') {
//             cout << "Connected to the XBee" << endl;
//         } else {
//             cerr << "Failed to connect to the XBee" << endl;
//             return 1;
//         }

//         cmd = "AT";
//         cmd += "MY" + std::to_string(my_address) + ",";
//         cmd += "BD3,"; // Baud rate 57600
//         cmd += "ID" + std::to_string(1331) + ","; // Pan ID
//         cmd += "CH" + std::string("0D") + ",";
//         cmd += "DL0,";
//         cmd += "RN1,";
//         cmd += "RO5,";
//         cmd += "WR";

//         // Send AT command
//         port.write_some(asio::buffer(cmd, cmd.size()));

//         // Reset the XBee
//         cmd = "ATRE";
//         port.write_some(asio::buffer(cmd, cmd.size()));

//         // Close the serial port
//         port.close();
//         cout << "XBee successfully programmed!" << endl;

//     } catch (system_error& e) {
//         cerr << "Error: " << e.what() << endl;
//         return 1;
//     }

//     return 0;
// }

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

using namespace std;

bool sendATCommand(int fd, const string& command) {
    string fullCommand = "AT" + command + "\r";
    ssize_t bytesWritten = write(fd, fullCommand.c_str(), fullCommand.size());
    if (bytesWritten == -1) {
        cerr << "Error writing to serial port." << endl;
        return false;
    }
    return true;
}

bool setSerialPortAttributes(int fd, int speed) {
    struct termios tty;
    if (tcgetattr(fd, &tty) < 0) {
        cerr << "Error getting serial port attributes." << endl;
        return false;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        cerr << "Error setting serial port attributes." << endl;
        return false;
    }

    return true;
}

int main(int argc, char* argv[]) {
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <serial_port> <my_adr>" << endl;
        return 1;
    }

    const string serialPort = argv[1];
    const int myAddress = atoi(argv[2]);

    int fd = open(serialPort.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd == -1) {
        cerr << "Error opening serial port." << endl;
        return 1;
    }

    // Set serial port attributes (57600 baud rate)
    if (!setSerialPortAttributes(fd, B57600)) {
        close(fd);
        return 1;
    }

    // Switch to command mode
    if (!sendATCommand(fd, "+++")) {
        close(fd);
        return 1;
    }

    // Wait for command mode
    usleep(1000000);  // 1 second

    // Check if connected to the XBee
    char response[3];
    ssize_t bytesRead = read(fd, response, sizeof(response));
    if (bytesRead == -1) {
        cerr << "Error reading from serial port." << endl;
        close(fd);
        return 1;
    }
	ROS_INFO("%lf, %lf", response[0], response[1]);
    if (bytesRead == 2 && response[0] == 'O' && response[1] == 'K') {
        cout << "Connected to the XBee" << endl;
    } else {
        cerr << "Failed to connect to the XBee" << endl;
        close(fd);
        return 1;
    }

    // Configure XBee settings (replace with your desired settings)
    if (!sendATCommand(fd, "MY" + to_string(myAddress) + ",")) {
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "BD7,")) {  // 57600 baud rate
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "ID1331,")) {  // Pan ID (replace with your desired ID)
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "CH0D,")) {  // Channel (replace with your desired channel)
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "DL0,")) {
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "RN1,")) {
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "RO5,")) {
        close(fd);
        return 1;
    }
    if (!sendATCommand(fd, "WR")) {  // Write settings to non-volatile memory
        close(fd);
        return 1;
    }

    // Reset the XBee
    if (!sendATCommand(fd, "RE")) {
        close(fd);
        return 1;
    }

    // Close the serial port
    close(fd);

    cout << "XBee successfully programmed!" << endl;

    return 0;
}

