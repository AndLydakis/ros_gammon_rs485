//
// Created by lydakis on 14/06/18.
//

#ifndef ROS_GAMMON_RS485_RS485_PORT_H
#define ROS_GAMMON_RS485_RS485_PORT_H

#include <serial/serial.h>
#include <cstddef>
#include <cstdint>

typedef uint8_t byte;

void sendMsg(serial::Serial &port, const byte *data, byte length);

byte recvMsg(serial::Serial &port, byte *data, const byte length, unsigned long timeout = 500);


#endif //ROS_GAMMON_RS485_RS485_PORT_H
