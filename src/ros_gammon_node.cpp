//
// Created by lydakis on 05/07/18.
//

#include "ros/ros.h"
#include "RS485Slave.h"
#include "RS485Slave.cpp"
#include <std_msgs/Int8.h>

using namespace serial;
using namespace std;

int main(int argc, char **argv) {


    ros::init(argc, argv, "ros_gammon_node");
    ros::NodeHandle nh;

    // Open, setup and test a communications port
    Serial *arduino_port;
    try {
        arduino_port = new serial::Serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(1000));
    } catch (serial::SerialException &se) {
        cout << "Serial Exception (Is the adapter plugged in ?)\n";
        cout << se.what() << endl;
        return -1;
    } catch (serial::IOException &ioe) {
        cerr << "IO Exception (Is the adapter plugged in ?)\n";
        cerr << ioe.what() << endl;
        return -1;
    }
    ROS_INFO("Initialized port");
//    SubscribeOptions subscribeOptions{};
//    subscribeOptions.topic = "test";
//    subscribeOptions.queue_length = 10;
//    AdvertiseOptions advertiseOptions{};
//    advertiseOptions.topic = "response";
//    advertiseOptions.queue_length = 10;
//
//    std_msgs_Int8_Slave slave(nh, 1, 500, *arduino_port,
//                              subscribeOptions, advertiseOptions);
//    ROS_INFO("Initialized slave");

    GammonSlave<std_msgs::Int8, std_msgs::Int8> gs(nh, 1, 2000, *arduino_port, 255, 255);
    while (ros::ok()) {
        ros::spinOnce();
    }
    return EXIT_SUCCESS;
}