//
// Created by lydakis on 04/07/18.
//
#include "RS485Slave.h"

constexpr size_t MAX_RESPONSE_LENGTH_LIMIT = 255;
constexpr size_t MAX_MESSAGE_LENGTH_LIMIT = 255;

//template<typename T, typename R>
bool RS485Slave::recvMsg() {}

//template<typename T, typename R>
bool RS485Slave::sendMsg() {}

//template<typename T, typename R>
uint8_t RS485Slave::getID() { return ID; }

//template<typename T, typename R>
void RS485Slave::setID(uint8_t id) { ID = id; }

//template<typename T, typename R>
unsigned long RS485Slave::getTimeOut() { return timeOut; }

//template<typename T, typename R>
void RS485Slave::setTimeout(unsigned long timeout) { timeOut = timeout; }

//template<typename T, typename R>
RS485Slave::RS485Slave(ros::NodeHandle &nh_, uint8_t id_, unsigned long timeOut_, serial::Serial &port_) : nh(
        nh_), ID(id_), timeOut(timeOut_), port(port_) {}

//GammonBroker::GammonBroker(ros::NodeHandle &nh_, size_t id_) : nh(nh_), broker_id(id_) {
//    service = nh.advertiseService("gammon_exchange", exchangeService);
//}

template<typename T, typename R>
GammonSlave<T, R>::GammonSlave(ros::NodeHandle &nh_, size_t id_, unsigned long timeOut_, serial::Serial &port_,
                               size_t mx_msg_ln, size_t mx_res_len):
        RS485Slave(nh_, id_, timeOut_, port_), max_message_length(mx_msg_ln), max_response_length(mx_msg_ln) {
    max_message_length = std::min(max_message_length, MAX_MESSAGE_LENGTH_LIMIT);
    max_response_length = std::min(max_response_length, MAX_RESPONSE_LENGTH_LIMIT);
//    service = nh.advertiseService("gammon_exchange_" + ID, std::bind(&exchangeService, this));
}

template<typename T, typename R>
size_t GammonSlave<T, R>::getMaxMsgLength() { return max_message_length; }

template<typename T, typename R>
size_t GammonSlave<T, R>::getMaxResLength() { return max_response_length; }

template<typename T, typename R>
void GammonSlave<T, R>::setMaxMsgLength(size_t len) { max_message_length = len; }

template<typename T, typename R>
void GammonSlave<T, R>::setMaxResLength(size_t len) { max_response_length = len; }

template<typename T, typename R>
T &GammonSlave<T, R>::getTransmit() { return transmit; }

template<typename T, typename R>
T const &GammonSlave<T, R>::getTransmit() const { return transmit; }

template<typename T, typename R>
void GammonSlave<T, R>::setTransmit(T &new_req) { transmit = new_req; }

template<typename T, typename R>
R &GammonSlave<T, R>::getReceive() { return receive; }

template<typename T, typename R>
R const &GammonSlave<T, R>::getReceive() const { return receive; }

template<typename T, typename R>
void GammonSlave<T, R>::setReceive(T &new_rec) { receive = new_rec; }

template<typename T, typename R>
bool GammonSlave<T, R>::exchange() {
    try {
        ros_gammon_rs485::GammonExchange srv;
        srv.request.message = serialize();
        exchangeService(srv.request, srv.response);
        deSerialize(srv.response);

    } catch (ros::Exception re) {
        ROS_ERROR("Exception during the exchange: %s", re.what());
    } catch (...) {
        ROS_ERROR("Exception during the exchange");
        return false;
    }
    return true;
}

template<typename T, typename R>
bool GammonSlave<T, R>::exchangeService(ros_gammon_rs485::GammonExchange::Request &req,
                                        ros_gammon_rs485::GammonExchange::Response &res) {
    try {
        //Send Message
        byte payload[max_message_length + 1];
        payload[0] = ((byte) ((this->ID)) << 4);
        for (size_t i = 0; i < max_message_length; ++i) {
            ROS_ERROR("Adding to message: %d", req.message[i]);
            payload[i + 1] = req.message[i];
        }
        ROS_GAMMON_RS485_RS485_PORT_H::sendMsg(this->port, payload, sizeof(*payload));

        //Receive response
        int rcv = ROS_GAMMON_RS485_RS485_PORT_H::recvMsg(this->port, res.response, 255, this->timeOut);
        if (!rcv) {
            ROS_ERROR("Failed to receive msg");
            return false;
        }

        //Clean up stream
        this->port.flush();

        //Print received message
        for (size_t i = 0; i < 255; ++i) {
            if (res.response[i])ROS_ERROR("%d", res.response[i]);
        }
    } catch (...) {
        ROS_ERROR("Error at the exchange, flushing stream");
        this->port.flush();
        return false;
    }
    return true;
}

template<typename T, typename R>
boost::shared_array <byte> GammonSlave<T, R>::serialize() {
    try {
        uint32_t serialized_length = ros::serialization::serializationLength(transmit);
        //Create empty buffer
        boost::shared_array <uint8_t> buffer(new byte[serialized_length]);
        ros::serialization::OStream stream(buffer.get(), serialized_length);
        //Fill buffer
        ros::serialization::Serializer<std_msgs::Int8>::read(stream, transmit);
        //Return buffer
        return buffer;
    } catch (ros::serialization::StreamOverrunException) {
        ROS_ERROR("Exception when serializing message: Buffer Overflow");
        return nullptr;
    }

}

template<typename T, typename R>
bool GammonSlave<T, R>::deSerialize(boost::shared_array <byte> res) {
    ROS_ERROR("Deserializing response");
    try {
        uint32_t deserialization_length = ros::serialization::serializationLength(receive);
        ros::serialization::IStream stream(res.get(), deserialization_length);
        ros::serialization::deserialize(stream, receive);
        ROS_INFO("Deserialized Message: ");
//        ROS_INFO_STREAM(res);
    } catch (ros::serialization::StreamOverrunException soe) {
        ROS_ERROR("Deserialization Exception: %s", soe.what());
        return false;
    } catch (...) {
        ROS_ERROR("Unknown Deserialization Exception");
        return false;
    }
    ROS_ERROR("Successfully deserialized message");
    return true;
}

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
    if (!sub_topic.empty()) {
        sub = nh.subscribe(sub_topic, 10, &GammonTopicSlave<T, R>::subCallback, this);
    }
    if (!pub_topic.empty()) {
        pub = nh.advertise<R>(pub_topic, 10, false);
    }
}

template<typename T, typename R>
void GammonTopicSlave<T, R>::subCallback(const typename T::ConstPtr &msg_) {
    transmit = msg_;
}

template<typename T, typename R>
std::string GammonTopicSlave<T, R>::getPubTopic() { return pub_topic; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setPubTopic(std::string topic_) {
    pub_topic = topic_;
    pub = nh.advertise<R>(pub_topic, 10, false);
}

template<typename T, typename R>
std::string GammonTopicSlave<T, R>::getSubTopic() { return sub_topic; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setSubTopic(std::string topic_) {
    sub_topic = topic_;
    sub = nh.advertise<R>(sub_topic, 10, false);
    sub = nh.subscribe(sub_topic, 10, &GammonTopicSlave<T, R>::subCallback, this);
}

template<typename T, typename R>
const T &GammonTopicSlave<T, R>::getTransmit() const { return transmit; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setTransmit(T &msg) { transmit = msg; }

template<typename T, typename R>
const R &GammonTopicSlave<T, R>::getReceive() const { return receive; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setReceive(R &msg) { receive = msg; }

template<typename T, typename R>
byte *GammonTopicSlave<T, R>::getTransmitData() { return transmitData; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setTransmitData(uint8_t *new_data) { transmitData = new_data; }

template<typename T, typename R>
byte *GammonTopicSlave<T, R>::getReceiveData() { return receiveData; }

template<typename T, typename R>
void GammonTopicSlave<T, R>::setReceiveData(uint8_t *new_data) { receiveData = new_data; }

template<typename T, typename R>
size_t GammonTopicSlave<T, R>::getMaxReceiveLen(){return max_receive_len;}

template<typename T, typename R>
void GammonTopicSlave<T, R>::setMaxReceiveLen(size_t len){max_receive_len = len;}

template<typename T, typename R>
size_t GammonTopicSlave<T, R>::getMaxTransmitLen(){return max_transmit_len;}

template<typename T, typename R>
void GammonTopicSlave<T, R>::setMaxTransmitLen(size_t len){max_transmit_len = len;}

template<typename T, typename R>
boost::shared_array <byte> GammonTopicSlave<T, R>::serialize() {
    try {
        uint32_t serialized_length = ros::serialization::serializationLength(transmit);
        //Create empty buffer
        boost::shared_array <uint8_t> buffer(new byte[serialized_length]);
        ros::serialization::OStream stream(buffer.get(), serialized_length);
        //Fill buffer
        ros::serialization::Serializer<T>::read(stream, transmit);
        //Return buffer
        return buffer;
    } catch (ros::serialization::StreamOverrunException) {
        ROS_ERROR("Exception when serializing message: Buffer Overflow");
        return nullptr;
    }

}

template<typename T, typename R>
bool GammonTopicSlave<T, R>::deSerialize(boost::shared_array <byte> res) {
    ROS_ERROR("Deserializing response");
    try {
        uint32_t deserialization_length = ros::serialization::serializationLength(receive);
        ros::serialization::IStream stream(res.get(), deserialization_length);
        ros::serialization::deserialize(stream, receive);
        ROS_INFO("Deserialized Message: ");
//        ROS_INFO_STREAM(res);
    } catch (ros::serialization::StreamOverrunException soe) {
        ROS_ERROR("Deserialization Exception: %s", soe.what());
        return false;
    } catch (...) {
        ROS_ERROR("Unknown Deserialization Exception");
        return false;
    }
    ROS_ERROR("Successfully deserialized message");
    return true;
}

template<typename T, typename R>
bool GammonTopicSlave<T, R>::makeExchange() {
    try {
        auto payload = serialize();
        sendMsg(payload);
//        deSerialize(srv.response);

    } catch (ros::Exception re) {
        ROS_ERROR("Exception during the exchange: %s", re.what());
    } catch (...) {
        ROS_ERROR("Exception during the exchange");
        return false;
    }
    return true;
}

template<typename T, typename R>
bool GammonTopicSlave<T, R>::sendMsg(byte *data) {

}
