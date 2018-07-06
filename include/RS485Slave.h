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
    std::vector <RS485Slave> slaves;
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

    boost::shared_array <uint8_t> serialize();

    bool deSerialize(boost::shared_array <uint8_t> res);

    T transmit;

    R receive;

    ros::ServiceServer service;
};

template<typename T, typename R>
class GammonTopicSlave : public RS485Slave {
public:
    GammonTopicSlave(ros::NodeHandle nh_, size_t id_, unsigned long timeout_, serial::Serial &port_,
                     std::string pub_topic = "", std::string sub_topic = "", size_t max_transmit_len_ = 1,
                     size_t max_receive_len_ = 1);

    std::string getSubTopic();

    void setSubTopic(std::string topic_);

    std::string getPubTopic();

    void setPubTopic(std::string topic_);

    void subCallback(const typename T::ConstPtr &msg_);

    bool makeExchange();

    const T &getTransmit() const;

    void setTransmit(T &msg);

    const R &getReceive() const;

    void setReceive(R &msg);

    bool sendMsg(byte *data);

    bool recvMsg();

    byte *getTransmitData();

    void setTransmitData(byte *new_data);

    byte *getReceiveData();

    void setReceiveData(byte *new_data);

    size_t getMaxTransmitLen();

    void setMaxTransmitLen(size_t len);

    size_t getMaxReceiveLen();

    void setMaxReceiveLen(size_t len);

protected:
private:
    std::string pub_topic;
    std::string sub_topic;
    ros::Subscriber sub;
    ros::Publisher pub;

    T transmit;

    R receive;

    byte *transmitData;

    byte *receiveData;

    size_t max_transmit_len;

    size_t max_receive_len;

    boost::shared_array <byte> serialize();

    bool deSerialize(boost::shared_array <byte> res);

};

#endif //ROS_GAMMON_RS485_RS485SLAVE_H
