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

    std_msgs::Int8 msg1;
    int cnt = 0;
    std::vector<unique_ptr<RS485Slave>> slaves;
    std::string slave_name;
    std::string slave_("/slave");
    std::string id_ = "/id";
    std::string query = slave_ + std::to_string(cnt + 1) + id_;
    cout << slave_ << endl;
    cout << id_ << endl;
    cout << cnt + 1 << endl;
    cout << query << endl;
    ROS_INFO("Looking for %s", query.c_str());
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
        cout << query.c_str() << endl;
        if (!nh.getParam(query.c_str(), sub_type))return EXIT_FAILURE;
        ROS_INFO("Found sub type");
        query = slave_ + std::to_string(cnt + 1) + "/pub_type";
        if (!nh.getParam(query.c_str(), pub_type))return EXIT_FAILURE;
        ROS_INFO("Found pub type");
        query = slave_ + std::to_string(cnt + 1) + "/id";
        if (!nh.getParam(query.c_str(), id))return EXIT_FAILURE;
        ROS_INFO("Found id");
        query = slave_ + std::to_string(cnt + 1) + "/timeout";
        if (!nh.getParam(query.c_str(), timeout))return EXIT_FAILURE;
        ROS_INFO("Found timeout");
        query = slave_ + std::to_string(cnt + 1) + "/sub_topic";
        if (!nh.getParam(query.c_str(), sub_topic))return EXIT_FAILURE;
        ROS_INFO("Found sub topic");
        query = slave_ + std::to_string(cnt + 1) + "/pub_topic";
        if (!nh.getParam(query.c_str(), pub_topic))return EXIT_FAILURE;
        ROS_INFO("Found pub topic");
        query = slave_ + std::to_string(cnt + 1) + "/mrl";
        if (!nh.getParam(query.c_str(), mrl))return EXIT_FAILURE;
        ROS_INFO("Found mrl");
        query = slave_ + std::to_string(cnt + 1) + "/mtl";
        if (!nh.getParam(query.c_str(), mtl))return EXIT_FAILURE;
        ROS_INFO("Found mtl");
        slaves.emplace_back(
                createGammonTopicSlave(sub_type, pub_type, nh, id, timeout, *arduino_port, pub_topic, sub_topic, mtl,
                                       mrl));

        cnt++;
        query = slave_ + std::to_string(cnt + 1) + "/sub_type";
    }
    ROS_INFO("Found %d slaves", cnt);
    if (cnt == 0) return EXIT_FAILURE;



//    slaves.emplace_back(
//            createGammonTopicSlave(type1, type1, nh, 1, 500, *arduino_port, pub_topic0, sub_topic0, 10, 10));
//    slaves.emplace_back(
//            createGammonTopicSlave(type1, type1, nh, 2, 500, *arduino_port, pub_topic1, sub_topic1, 10, 10));

    ROS_INFO("Created Slaves");
//    cout << decltype(msg1);
//    GammonTopicSlave<decltype(msg1), decltype(msg1)> gts(nh, 1, 500, *arduino_port, "test", "res", 10, 10);
//    GammonTopicSlave<std_msgs::Int8, std_msgs::Int8> (nh, 1, 500, *arduino_port, "test", "res", 10, 10);
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


    ros::Rate r(500);
    while (ros::ok()) {
        for (size_t i = 0; i < slaves.size(); ++i)
            slaves[i]->makeExchange();
        ros::spinOnce();
        r.sleep();
    }
    return EXIT_SUCCESS;
}