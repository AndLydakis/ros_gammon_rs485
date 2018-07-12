//
// Created by Andreas Lydakis on 04/07/18.
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
    virtual ~RS485Slave() {};

    RS485Slave(ros::NodeHandle &nh_, uint8_t id_, unsigned long timeOut_, serial::Serial &port_);

    virtual bool recvMsg();

    virtual bool sendMsg();

    virtual bool makeExchange();

    uint8_t getID();

    void setID(uint8_t id);

    unsigned long getTimeOut();

    void setTimeout(unsigned long timeout);

    virtual void init_sub();

protected:
    ros::NodeHandle nh;

    uint8_t ID;

    unsigned long timeOut;

    serial::Serial &port;
private:

};

template<typename T, typename R>
class GammonTopicSlave : public RS485Slave {
public:
    GammonTopicSlave(ros::NodeHandle nh_, size_t id_, unsigned long timeout_, serial::Serial &port_,
                     std::string pub_topic = "", std::string sub_topic = "", size_t max_transmit_len_ = 1,
                     size_t max_receive_len_ = 1);

    ~GammonTopicSlave();

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

    bool sendMsg();

    bool recvMsg();

    byte *getTransmitData();

    void setTransmitData(byte *new_data);

    byte *getReceiveData();

    void setReceiveData(byte *new_data);

    size_t getMaxTransmitLen();

    void setMaxTransmitLen(size_t len);

    size_t getMaxReceiveLen();

    void setMaxReceiveLen(size_t len);

    void init_sub();
//    void operator()(const typename T::ConstPtr &msg_);
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

    boost::shared_array<byte> serialize();

    bool deSerialize();

};

#endif //ROS_GAMMON_RS485_RS485SLAVE_H
