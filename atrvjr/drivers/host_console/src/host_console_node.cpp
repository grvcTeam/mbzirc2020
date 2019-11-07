#include <string>

#include "ros/ros.h"
#include "serial/serial.h"
#include "rosgraph_msgs/Log.h"

// Subscriber
ros::Subscriber sub;
// Serial port
serial::Serial *serial_port_;
// String to be sent
std::string str_to_send;
// Severity from which message is sent
int severity = 1;
// Create severity map
std::map<int, std::string> severirty_map =
{
    { 1, "DEBUG" },
    { 2, "INFO" },
    { 4, "WARN" },
    { 8, "ERROR" },
    { 16, "FATAL" }
};

void callback(const rosgraph_msgs::Log::ConstPtr& log)
{
    // Severity level constants
    // byte DEBUG=1 #debug level
    // byte INFO=2  #general level
    // byte WARN=4  #warning level
    // byte ERROR=8 #error level
    // byte FATAL=16 #fatal/critical level
    // Fields
    // Header header
    // byte level
    // string name # name of the node
    // string msg # message
    // string file # file the message came from
    // string function # function the message came from
    // uint32 line # line the message came from
    // string[] topics # topic names that the node publishes
    if(log->level >= severity)
    {
        str_to_send = "[" + severirty_map[log->level] + "]" + "[" + log->name + "] " + log->msg + "\n";
        serial_port_->write(str_to_send);
        std::cout << str_to_send;
    }
    return;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "host_console_node");
    ros::NodeHandle node_handle("~");

    // Get parameters
    std::string port;
    node_handle.param<std::string>("port", port, std::string("/dev/ttyUSB0"));
    int baudrate;
    node_handle.param<int>("baudrate", baudrate, 9600);
    node_handle.param<int>("severity", severity, 1);

    // Create port
    long timeout = 50;
    serial_port_ = new serial::Serial(port, baudrate, serial::Timeout::simpleTimeout(timeout));
    // Check status
    if (!serial_port_->isOpen()){
		std::cout << "Serial port: " << port << " failed to open." << std::endl;
		delete serial_port_;
		serial_port_ = NULL;
		return -1;
	} else {
		std::cout << "Serial port: " << port << " opened successfully." << std::endl;
	}

    // Subscribe to topic
    // It is prefered to subscribe to rosout_agg
    sub = node_handle.subscribe<rosgraph_msgs::Log>("/rosout_agg", 10, callback);

    //
    std::string read_data;
    ros::Rate r(10); // 10 hz
    while (ros::ok())
    {
        ros::spinOnce();
        r.sleep();
        if(serial_port_->read(read_data,1000))
        {
            char cxxx = read_data.c_str()[0];
            if(int(cxxx) == 13)
            {
                std::cout<<"Shuting all nodes down."<< std::endl;
                std::system("rosnode kill -a");
                ros::shutdown();
            }
        }
    }
    // Close serial connection
    std::cout << "Serial port: " << port << " closing." << std::endl;
    serial_port_->close();
    return 0;
}


