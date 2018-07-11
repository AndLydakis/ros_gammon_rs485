//
// Created by Andreas Lydakis on 05/07/18.
//

#include "ros/ros.h"
#include "RS485Slave.h"
#include "RS485Slave.cpp"
#include <std_msgs/Int8.h>

using namespace serial;
using namespace std;

std::vector<unique_ptr<RS485Slave>> createSlaves(const ros::NodeHandle &nh, serial::Serial *arduino_port) {
    std_msgs::Int8 msg1;
    int cnt = 0;
    std::vector<unique_ptr<RS485Slave>> slaves;
    std::string slave_name;
    std::string slave_("/slave");
    std::string id_ = "/id";
    std::string query = slave_ + std::to_string(cnt + 1) + id_;
    query = slave_ + std::to_string(cnt + 1) + "/sub_type";
    while (nh.hasParam(query.c_str())) {
        ROS_INFO("Creating Slave: %s%d", "/slave", cnt + 1);
        std::string sub_type;
        std::string pub_type;
        std::string sub_topic;
        std::string pub_topic;
        int id;
        int mtl;
        int mrl;
        int timeout;
        if (!nh.getParam(query.c_str(), sub_type))return {};
        ROS_INFO("Found sub type");
        query = slave_ + std::to_string(cnt + 1) + "/pub_type";
        if (!nh.getParam(query.c_str(), pub_type))return {};
        ROS_INFO("Found pub type");
        query = slave_ + std::to_string(cnt + 1) + "/id";
        if (!nh.getParam(query.c_str(), id))return {};
        ROS_INFO("Found id");
        query = slave_ + std::to_string(cnt + 1) + "/timeout";
        if (!nh.getParam(query.c_str(), timeout))return {};
        ROS_INFO("Found timeout");
        query = slave_ + std::to_string(cnt + 1) + "/sub_topic";
        if (!nh.getParam(query.c_str(), sub_topic))return {};
        ROS_INFO("Found sub topic");
        query = slave_ + std::to_string(cnt + 1) + "/pub_topic";
        if (!nh.getParam(query.c_str(), pub_topic))return {};
        ROS_INFO("Found pub topic");
        query = slave_ + std::to_string(cnt + 1) + "/mrl";
        if (!nh.getParam(query.c_str(), mrl))return {};
        ROS_INFO("Found mrl");
        query = slave_ + std::to_string(cnt + 1) + "/mtl";
        if (!nh.getParam(query.c_str(), mtl))return {};
        ROS_INFO("Found mtl");
        slaves.emplace_back(
                createGammonTopicSlave(sub_type, pub_type, nh, id, timeout, *arduino_port, pub_topic, sub_topic, mtl,
                                       mrl));
        slaves[slaves.size() - 1]->init_sub();
        cnt++;
        query = slave_ + std::to_string(cnt + 1) + "/sub_type";
    }
    return slaves;
}

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

    auto slaves = createSlaves(nh, arduino_port);

    ROS_INFO("Found %ld slaves", slaves.size());
    if (slaves.empty())throw std::invalid_argument("No slave configurations were found");

    ROS_INFO("Created Slaves");

    ros::Rate r(500);
    while (ros::ok()) {
        for (size_t i = 0; i < slaves.size(); ++i)
            slaves[i]->makeExchange();
        ros::spinOnce();
        r.sleep();
    }
    return EXIT_SUCCESS;
}