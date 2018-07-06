//
// Created by lydakis on 04/07/18.
//

#ifndef ROS_GAMMON_RS485_RS485SLAVE_H
#define ROS_GAMMON_RS485_RS485SLAVE_H

#include <serial/serial.h>
#include <RS485_port.cpp>
#include <ros_gammon_rs485/GammonExchange.h>
#include <std_msgs/Int8.h>

typedef uint8_t byte;

struct AdvertiseOptions {
    std::string topic;
    size_t queue_length;
};

struct SubscribeOptions {
    std::string topic;
    size_t queue_length;
};

class RS485Slave {
public:
//    RS485Slave() {};

//    virtual ~RS485Slave() {};

    RS485Slave(ros::NodeHandle &nh_, uint8_t id_, unsigned long timeOut_, serial::Serial &port_);

    virtual bool recvMsg();

    virtual bool sendMsg();

    uint8_t getID();

    void setID(uint8_t id);

    unsigned long getTimeOut();

    void setTimeout(unsigned long timeout);

protected:
    ros::NodeHandle nh;

    uint8_t ID;

    unsigned long timeOut;

    serial::Serial &port;
private:

};

class GammonBroker {
public:
    GammonBroker(ros::NodeHandle &nh, size_t id_ = 0);

    static bool exchangeService(std::function<bool()> clientExchangeFunction);

    void addClient(RS485Slave slave);

    void removeClient(RS485Slave slave);

protected:
private:
    std::vector<RS485Slave> slaves;
    ros::NodeHandle nh;
    ros::ServiceServer service;
    size_t broker_id;
};

template<typename T, typename R>
class GammonSlave : public RS485Slave {

public:
    GammonSlave(ros::NodeHandle &nh, size_t id, unsigned long timeout, serial::Serial &port, size_t mx_msg_ln = 1,
                size_t mx_res_len = 1);

    void setMaxMsgLength(size_t len);

    void setMaxResLength(size_t len);

    bool exchange();

    bool exchangeService(ros_gammon_rs485::GammonExchange::Request &req,
                         ros_gammon_rs485::GammonExchange::Response &res);

    T &getTransmit();

    T const &getTransmit() const;

    void setTransmit(T &new_req);

    R &getReceive();

    R const &getReceive() const;

    void setReceive(T &new_rec);

    size_t getMaxMsgLength();

    size_t getMaxResLength();

    bool sendMsg() {};

    bool recvMsg() {};

protected:
private:
    size_t max_message_length;
    size_t max_response_length;

    boost::shared_array<uint8_t> serialize();

    bool deSerialize(boost::shared_array<uint8_t> res);

    T transmit;

    R receive;

    ros::ServiceServer service;
};

//class std_msgs_Int8_Slave : public RS485Slave<std_msgs::Int8, std_msgs::Int8> {
//public:
//    std_msgs_Int8_Slave(ros::NodeHandle &nh_, uint8_t id_, unsigned long timeOut_, serial::Serial &port_,
//                        SubscribeOptions &sub_opts, AdvertiseOptions &ad_opts);
//
////    ~std_msgs_Int8_Slave() {}
//
////    std_msgs_Int8_Slave() {}
//
//    bool recvMsg();
//
//    bool sendMsg();
//
//    void sub_callback(const std_msgs::Int8ConstPtr &msg_);
//
//protected:
//private:
//
//    boost::shared_array<byte> serialize(std_msgs::Int8);
//
//    std::string sub_topic;
//
//    std::string pub_topic;
//
//    std_msgs::Int8 in_msg;
//
//    std_msgs::Int8 out_msg;
//
//    ros::Subscriber sub;
//
//    ros::Publisher pub;
//};

#endif //ROS_GAMMON_RS485_RS485SLAVE_H
