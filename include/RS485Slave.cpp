//
// Created by Andreas Lydakis on 04/07/18.
//
#include <csignal>
#include "RS485Slave.h"
#include "std_msgs/Int64.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/Int8.h"

constexpr size_t MAX_RESPONSE_LENGTH_LIMIT = 255;
constexpr size_t MAX_MESSAGE_LENGTH_LIMIT = 255;

constexpr char INT_8[] = "std_msgs/Int8";
constexpr char INT_16[] = "std_msgs/Int16";
constexpr char INT_32[] = "std_msgs/Int32";
constexpr char INT_64[] = "std_msgs/Int64";
constexpr char FLOAT_32[] = "std_msgs/Float32";
constexpr char FLOAT_64[] = "std_msgs/Float64";

/**
 * Create a GammonTopicSlave according to the given parameters
 * @param type1: the type of message the slave subscribes to
 * @param type2: the type of message the slave publishes
 * @param nh_: a ros::Node handle for your ROS-related needs
 * @param id_: the ID of the slave
 * @param timeout_: the timeout for the send/receive routines in milliseconds
 * @param port_: a serial::Serial that is connected to the rest of the devices
 * @param pub_topic: the topic the slave publishes its messages to
 * @param sub_topic: the topic the slave subscribes to for additional information
 * @param max_transmit_len_: maximum length of the transmitted message (must be between 1-255)
 * @param max_receive_len_: maximum length of the received message (must be between 1-255)
 * @return
 */
std::unique_ptr<RS485Slave>
createGammonTopicSlave(std::string type1, std::string type2, ros::NodeHandle nh_, size_t id_, unsigned long timeout_,
                       serial::Serial &port_,
                       std::string pub_topic = "", std::string sub_topic = "", size_t max_transmit_len_ = 1,
                       size_t max_receive_len_ = 1) {
    ROS_INFO("New Slave: ");
    ROS_INFO("Listening for %s on %s", type2.c_str(), sub_topic.c_str());
    ROS_INFO("Publishing %s on %s", type1.c_str(), pub_topic.c_str());
    if ((type1 == INT_8) && (type2 == INT_8)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int8, std_msgs::Int8>>{
                new GammonTopicSlave<std_msgs::Int8, std_msgs::Int8>(nh_, id_, timeout_, port_, pub_topic,
                                                                     sub_topic,
                                                                     max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_8) && (type2 == INT_16)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int8, std_msgs::Int16>>{
                new GammonTopicSlave<std_msgs::Int8, std_msgs::Int16>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_8) && (type2 == INT_32)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int8, std_msgs::Int32>>{
                new GammonTopicSlave<std_msgs::Int8, std_msgs::Int32>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_8) && (type2 == INT_64)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int8, std_msgs::Int64>>{
                new GammonTopicSlave<std_msgs::Int8, std_msgs::Int64>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_16) && (type2 == INT_8)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int16, std_msgs::Int8>>{
                new GammonTopicSlave<std_msgs::Int16, std_msgs::Int8>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_16) && (type2 == INT_16)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int16, std_msgs::Int16>>{
                new GammonTopicSlave<std_msgs::Int16, std_msgs::Int16>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_16) && (type2 == INT_32)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int16, std_msgs::Int32>>{
                new GammonTopicSlave<std_msgs::Int16, std_msgs::Int32>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_16) && (type2 == INT_64)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int16, std_msgs::Int64>>{
                new GammonTopicSlave<std_msgs::Int16, std_msgs::Int64>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_32) && (type2 == INT_8)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int32, std_msgs::Int8>>{
                new GammonTopicSlave<std_msgs::Int32, std_msgs::Int8>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_32) && (type2 == INT_16)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int32, std_msgs::Int16>>{
                new GammonTopicSlave<std_msgs::Int32, std_msgs::Int16>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_32) && (type2 == INT_32)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int32, std_msgs::Int32>>{
                new GammonTopicSlave<std_msgs::Int32, std_msgs::Int32>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_32) && (type2 == INT_64)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int32, std_msgs::Int64>>{
                new GammonTopicSlave<std_msgs::Int32, std_msgs::Int64>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_64) && (type2 == INT_8)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int64, std_msgs::Int8>>{
                new GammonTopicSlave<std_msgs::Int64, std_msgs::Int8>(nh_, id_, timeout_, port_, pub_topic,
                                                                      sub_topic,
                                                                      max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_64) && (type2 == INT_16)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int64, std_msgs::Int16>>{
                new GammonTopicSlave<std_msgs::Int64, std_msgs::Int16>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_64) && (type2 == INT_32)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int64, std_msgs::Int32>>{
                new GammonTopicSlave<std_msgs::Int64, std_msgs::Int32>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    } else if ((type1 == INT_64) && (type2 == INT_64)) {
        return std::unique_ptr<GammonTopicSlave<std_msgs::Int64, std_msgs::Int64>>{
                new GammonTopicSlave<std_msgs::Int64, std_msgs::Int64>(nh_, id_, timeout_, port_, pub_topic,
                                                                       sub_topic,
                                                                       max_transmit_len_, max_receive_len_)};
    }
}

/**
 * Virtual function the derived classes should implement
 */
void RS485Slave::init_sub() {}

/**
 * Virtual function the derived classes should implement
 * @return
 */
bool RS485Slave::recvMsg() {}

/**
 * Virtual function the derived classes should implement
 * @return
 */
bool RS485Slave::sendMsg() {}

/**
 * Return the ID of the slave
 * @return int, the ID of the slave
 */
uint8_t RS485Slave::getID() { return ID; }

/**
 * Set the ID of the slave
 * @param id, the number the ID should be set to
 */
void RS485Slave::setID(uint8_t id) { ID = id; }

/**
 * Return the timeout of the slave
 * @return the timeout of the slave
 */
unsigned long RS485Slave::getTimeOut() { return timeOut; }

/**
 * Set the timeout of the slave
 * @param timeout, the value that will be the timeout of the slave
 */
void RS485Slave::setTimeout(unsigned long timeout) { timeOut = timeout; }

/**
 * constructor
 * @param nh_ : a ros NodeHandle
 * @param id_ : the positive id of the slave
 * @param timeOut_: the timeout in milliseconds for the slave
 * @param port_: the port the devices are connected to
 */
RS485Slave::RS485Slave(ros::NodeHandle &nh_, uint8_t id_, unsigned long timeOut_, serial::Serial &port_) : nh(
        nh_), ID(id_), timeOut(timeOut_), port(port_) {

    if (ID <= 0)throw std::invalid_argument("Non-positive id, exiting");
    if (timeOut <= 0)throw std::invalid_argument("Non-positive timeout, exiting");
}

bool RS485Slave::makeExchange() {}

template<typename T, typename R>
GammonTopicSlave::~GammonTopicSlave() {}

/**
 * Constructor
 * @tparam T , type of ROS message transmitted by the slave
 * @tparam R , type of ROS message received and republished by the slave
 * @param nh_ , ros::NodeHandle that handles ROS related affairs
 * @param id_ , the positive id of the slave
 * @param timeout_ , the positive timeout (in milliseconds) for send and receive functions
 * @param port_ , a serial::Serial port that performs the exchanges
 * @param pub_topic_ , a std::string representing the topic the slave republishes incoming data to
 * @param sub_topic_ , a std::string representing the topic the slaves listens to for changes to its data
 * @param max_transmit_len_ , maximum length in bytes of the message to be sent
 * @param max_receive_len_ , maximum length in bytes of the message to be received
 */
template<typename T, typename R>
GammonTopicSlave<T, R>::GammonTopicSlave(ros::NodeHandle nh_, size_t id_, unsigned long timeout_, serial::Serial &port_,
                                         std::string pub_topic_, std::string sub_topic_, size_t max_transmit_len_,
                                         size_t max_receive_len_) : RS485Slave(nh_, id_,
                                                                               timeout_,
                                                                               port_),
                                                                    sub_topic(sub_topic_),
                                                                    pub_topic(pub_topic_),
                                                                    max_transmit_len(max_transmit_len_),
                                                                    max_receive_len(max_receive_len_) {
    if (!pub_topic.empty()) {
        pub = nh.advertise<R>(pub_topic, 10, false);
    }
    transmitData = new byte[max_transmit_len]{0};
    transmit.data = 10;
    receiveData = new byte[max_receive_len]{0};
}

/**
 * Callback to listen to a specific topic and change the message sent down the line
 * (or do some other function you can specify)
 * @param msg_ a ros message of type T, the same as the one being transmitted down the line by the slave
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::subCallback(const typename T::ConstPtr &msg_) {
    transmit = *msg_;
}

/**
 * Initialize the slave's subscriber to the subCallback function
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::init_sub() {
    sub = nh.subscribe<T>(sub_topic, 10, boost::bind(&GammonTopicSlave<T, R>::subCallback, this, _1), false);
};

/**
 * Return the topic the slave publishes to
 * @return the name of the topic
 */
template<typename T, typename R>
std::string GammonTopicSlave<T, R>::getPubTopic() { return pub_topic; }

/**
 * set the topic the slave publishes to
 * @param topic_, the new topic
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setPubTopic(std::string topic_) {
    pub_topic = topic_;
    pub = nh.advertise<R>(pub_topic, 10, false);
}

/**
 * Return the topic the topic the slaves subscribes to
 * @return the name of the topic
 */
template<typename T, typename R>
std::string GammonTopicSlave<T, R>::getSubTopic() { return sub_topic; }


template<typename T, typename R>
void GammonTopicSlave<T, R>::setSubTopic(std::string topic_) {
    sub_topic = topic_;
    sub = nh.advertise<R>(sub_topic, 10, false);
    sub = nh.subscribe(sub_topic, 10, &GammonTopicSlave<T, R>::subCallback);
}

/**
 * Get the ROS message the slave is about to transmit
 * @return a ROS message of type T
 */
template<typename T, typename R>
const T &GammonTopicSlave<T, R>::getTransmit() const { return transmit; }

/**
 * Set the ROS message to be serialized and transmitted
 * @param msg, a ROS message of type T
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setTransmit(T &msg) { transmit = msg; }

/**
 * Get the ROS message that the slave received
 * @return a ROS message of type R
 */
template<typename T, typename R>
const R &GammonTopicSlave<T, R>::getReceive() const { return receive; }

/**
 * Set the ROS message that was received and deserialized
 * @param msg, a ROS message of type R
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setReceive(R &msg) { receive = msg; }

/**
 * Get the data that is about to be transmitted as a byte array
 * @return a byte array containing the serialized data to be transmitted
 */
template<typename T, typename R>
byte *GammonTopicSlave<T, R>::getTransmitData() { return transmitData; }

/**
 * Set the serialized message to be transmitted directly
 * @param new_data a new byte array that will replace the message of the slave
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setTransmitData(uint8_t *new_data) { transmitData = new_data; }

/**
 * Get the data that was received before it was deserialized
 * @return, a byte array containing the last message that was received
 */
template<typename T, typename R>
byte *GammonTopicSlave<T, R>::getReceiveData() { return receiveData; }

/**
 * Set the received data
 * @param new_data a new byte array that will replace the message of the slave
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setReceiveData(uint8_t *new_data) { receiveData = new_data; }

/**
 * Return the maximum length of received data in bytes
 * @return the maximum length of received data
 */
template<typename T, typename R>
size_t GammonTopicSlave<T, R>::getMaxReceiveLen() { return max_receive_len; }

/**
 * Set the maximum length of received data in bytes
 * @param len, the new maximum length
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setMaxReceiveLen(size_t len) { max_receive_len = len; }

/**
 * Return the maximum length of transmitted data in bytes
 * @return the maximum length of transmitted data
 */
template<typename T, typename R>
size_t GammonTopicSlave<T, R>::getMaxTransmitLen() { return max_transmit_len; }

/**
 * Set the maximum length of transmitted data in bytes
 * @param len, the new maximum length
 */
template<typename T, typename R>
void GammonTopicSlave<T, R>::setMaxTransmitLen(size_t len) { max_transmit_len = len; }

/**
 * Method that performs the exchange between the slave instance and the
 * corresponding device
 * @return true if the exchange was successful, false othewrise
 */
template<typename T, typename R>
bool GammonTopicSlave<T, R>::makeExchange() {
    try {
//        ROS_INFO("Slave %d making the exchange", ID);
        serialize();
        sendMsg();
        recvMsg();
        port.flush();
        deSerialize();
    } catch (ros::Exception re) {
        port.flush();
        ROS_ERROR("Exception during the exchange: %s", re.what());
    } catch (...) {
        ROS_ERROR("Exception during the exchange");
        port.flush();
        return false;
    }
    return true;
}

/**
 * Serializes the ROS message to be transmitted to a byte array
 * @return the buffer containing the serialized message
 */
template<typename T, typename R>
boost::shared_array<byte> GammonTopicSlave<T, R>::serialize() {
    try {
        uint32_t serialized_length = ros::serialization::serializationLength(transmit);
//        ROS_INFO("Serialization length: %d", serialized_length);
        //Create empty buffer
        boost::shared_array<uint8_t> buffer(new byte[serialized_length]{0});
        ros::serialization::OStream stream(buffer.get(), serialized_length);
        //Fill buffer
        ros::serialization::Serializer<T>::read(stream, transmit);
        //Return buffer
        transmitData = new byte[serialized_length]{0};
        transmitData[0] = ((byte) ((ID)) << 4);
        for (size_t i = 0; i < serialized_length; ++i) {
            transmitData[i + 1] = buffer[i];
//            ROS_ERROR("Adding to message: %d", transmitData[i]);
        }
        return buffer;
    } catch (ros::serialization::StreamOverrunException) {
        ROS_ERROR("Exception when serializing message: Buffer Overflow");
        return nullptr;
    }

}

/**
 * Send the message to the slave
 * @return true if the send was successful, false otherwise
 */
template<typename T, typename R>
bool GammonTopicSlave<T, R>::sendMsg() {
    try {
//        ROS_INFO("Slave %d Sending Message of Length: %d", ID, max_transmit_len);
        ROS_GAMMON_RS485_RS485_PORT_H::sendMsg(port, transmitData, sizeof(*transmitData));
    } catch (serial::SerialException &se) {
        ROS_ERROR("Serial Exception: %s", se.what());
        port.flush();
        return false;
    } catch (...) {
        ROS_ERROR("Unknown exception while transmitting data");
        port.flush();
        return false;
    }
    return true;
}

/**
 * Receive a response from the slave
 * @return true if the receive was successful, false otherwise
 */
template<typename T, typename R>
bool GammonTopicSlave<T, R>::recvMsg() {
    try {
//        ROS_INFO("Slave %d Receiving Message: ", ID);
        int rcv = ROS_GAMMON_RS485_RS485_PORT_H::recvMsg(port, receiveData, max_receive_len, timeOut);
        port.flush();
//        for (size_t i = 0; i < max_receive_len; ++i) {
//            if (receiveData[i])ROS_ERROR("%d", receiveData[i]);
//        }
    } catch (serial::SerialException &se) {
        ROS_ERROR("Serial Exception: %s", se.what());
        port.flush();
        return false;
    } catch (...) {
        ROS_ERROR("Unknown exception while receiving response from device");
        port.flush();
        return false;
    }
    return true;

}

/**
 * Deserialize the received message to a ROS msg and publish it if possible
 * @return true if the deserialization was successful, false otherwise
 */
template<typename T, typename R>
bool GammonTopicSlave<T, R>::deSerialize() {
//    ROS_ERROR("Slave %d Deserializing response", ID);
    try {
        uint32_t deserialization_length = ros::serialization::serializationLength(receive);
        boost::shared_array<uint8_t> buffer(new byte[deserialization_length]);
        ros::serialization::IStream stream(receiveData, deserialization_length);
        ros::serialization::deserialize(stream, receive);
//        ROS_INFO("Deserialized Message: ");
//        ROS_INFO_STREAM(res);
//        for (size_t i = 0; i < deserialization_length; ++i) {
//            ROS_ERROR("Read message: %d", receiveData[i]);
//        }
        pub.publish(receive);
    } catch (ros::serialization::StreamOverrunException soe) {
        ROS_ERROR("Deserialization Exception: %s", soe.what());
        return false;
    } catch (...) {
        ROS_ERROR("Unknown Deserialization Exception");
        return false;
    }
//    ROS_ERROR("Successfully deserialized message");
    return true;
}



