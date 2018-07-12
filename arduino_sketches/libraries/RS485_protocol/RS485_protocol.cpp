/*
 RS485 protocol library.

 Devised and written by Nick Gammon.
 Date: 14 November 2011
 Version: 1.1

 Version 1.1 reset the timeout period after getting STX.

 Can send from 1 to 255 bytes from one node to another with:

 * Packet start indicator (STX)
 * Each data byte is doubled and inverted to check validity
 * Packet end indicator (ETX)
 * Packet CRC (checksum)


 To allow flexibility with hardware (eg. Serial, SoftwareSerial, I2C)
 you provide three "callback" functions which send or receive data. Examples are:

 void fWrite (const byte what)
 {
 Serial.write (what);
 }

 int fAvailable ()
 {
 return Serial.available ();
 }

 int fRead ()
 {
 return Serial.read ();
 }


PERMISSION TO DISTRIBUTE

 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the "Software"), to deal in the Software without restriction,
 including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so,
 subject to the following conditions:

 The above copyright notice and this permission notice shall be included in
 all copies or substantial portions of the Software.


 LIMITATION OF LIABILITY

 The software is provided "as is", without warranty of any kind, express or implied,
 including but not limited to the warranties of merchantability, fitness for a particular
 purpose and noninfringement. In no event shall the authors or copyright holders be liable
 for any claim, damages or other liability, whether in an action of contract,
 tort or otherwise, arising from, out of or in connection with the software
 or the use or other dealings in the software.

 */

#include <HardwareSerial.h>
#include <RS485_protocol.h>

typedef union {
    float number;
    uint8_t bytes[8];
} ROS_FLOAT32;

typedef union {
    double number;
    uint8_t bytes[16];
} ROS_FLOAT64;

typedef union {
    cnumber;
    int8_t byte;
} ROS_INT8;

typedef union {
    uint8_t number;
    uint8_t byte;
} ROS_UINT8;

typedef union {
    int16_t number;
    uint8_t bytes[2];
} ROS_INT16;

typedef union {
    uint16_t number;
    uint8_t bytes[2];
} ROS_UINT16;

typedef union {
    int32_t number;
    uint8_t bytes[4];
} ROS_INT32;

typedef union {
    uint32_t number;
    uint8_t bytes[4];
} ROS_UINT32;

typedef union {
    int64_t number;
    uint8_t bytes[8];
} ROS_INT64;

typedef union {
    uint64_t number;
    uint8_t bytes[8];
} ROS_UINT64;


const byte STX = '\2';
const byte ETX = '\3';

/**
 * Calculate 8bit CRC
 * @param addr pointer to the address of the message
 * @param len length of the message
 * @return the crc that was calculated
 */
static byte crc8(const byte *addr, byte len) {
    byte crc = 0;
    while (len--) {
        byte inbyte = *addr++;
        for (byte i = 8; i; i--) {
            byte mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix)
                crc ^= 0x8C;
            inbyte >>= 1;
        }  // end of for
    }  // end of while
    return crc;
}  // end of crc8

/**
 * send a byte complemented, repeated
 * only values sent would be (in hex):
 * 0F, 1E, 2D, 3C, 4B, 5A, 69, 78, 87, 96, A5, B4, C3, D2, E1, F0
 * @param fSend a function to process the send
 * @param what the byte to be sent
 */
void sendComplemented(WriteCallback fSend, const byte what) {
    byte c;

    // first nibble
    c = what >> 4;
    fSend((c << 4) | (c ^ 0x0F));

    // second nibble
    c = what & 0x0F;
    fSend((c << 4) | (c ^ 0x0F));

}  // end of sendComplemented

//
//
/**
 * send a message of "length" bytes (max 255) to other end
 * put STX at start, ETX at end, and add CRC
 * @param fSend a function to process the send
 * @param data a byte array containing the message
 * @param length the length of the message
 */
void sendMsg(WriteCallback fSend, const byte *data, const byte length) {
//    Serial.println("Sending Message");
    fSend(STX);  // STX
    for (byte i = 0; i < length; i++)
        sendComplemented(fSend, data[i]);
    fSend(ETX);  // ETX
    sendComplemented(fSend, crc8(data, length));
//    Serial.println("Sent Message!");
}  // end of sendMsg


/**
 * receive a message, maximum "length" bytes, timeout after "timeout" milliseconds
 * if nothing received, or an error (eg. bad CRC, bad data) return 0
 * otherwise, returns length of received data
 * @param fAvailable a function returning an int that shows if there is readable data
 * @param fRead a function to read the buffer
 * @param data the message to be received
 * @param length the length of the message
 * @param timeout in millis
 * @return
 */
byte recvMsg(AvailableCallback fAvailable,   // return available count
             ReadCallback fRead,             // read one byte
             byte *data,                    // buffer to receive into
             const byte length,              // maximum buffer size
             unsigned long timeout)          // milliseconds before timing out
{
    unsigned long start_time = millis();
    Serial.println("Receiving");
    bool have_stx = false;

    // variables below are set when we get an STX
    bool have_etx;
    byte input_pos;
    bool first_nibble;
    byte current_byte;

    while (millis() - start_time < timeout) {
        if (fAvailable() > 0) {
            byte inByte = fRead();
//            Serial.print("Read byte: ");
            Serial.println(int(inByte));
            switch (inByte) {
                case STX:   // start of text
//                    Serial.println("Found Start of Packet");
                    have_stx = true;
                    have_etx = false;
                    input_pos = 0;
                    first_nibble = true;
                    start_time = millis();  // reset timeout period
                    break;

                case ETX:   // end of text
//                    Serial.println("Found End of Packet");
                    have_etx = true;
                    break;

                default:
                    // wait until packet officially starts
                    if (!have_stx)
                        break;

                    // check byte is in valid form (4 bits followed by 4 bits complemented)
                    if ((inByte >> 4) != ((inByte & 0x0F) ^ 0x0F)) {
//                        Serial.println("Invalid Character, Returning");
                        return 0;  // bad character
                    }

                    // convert back
                    inByte >>= 4;

                    // high-order nibble?
                    if (first_nibble) {
//                        Serial.println("High Order Nibble");
                        current_byte = inByte;
                        first_nibble = false;
                        break;
                    }  // end of first nibble

                    // low-order nibble
//                    Serial.println("Low Order Nibble");
                    current_byte <<= 4;
                    current_byte |= inByte;
                    first_nibble = true;

                    // if we have the ETX this must be the CRC
                    if (have_etx) {
                        if (crc8(data, input_pos) != current_byte) {
                            Serial.println("Bad CRC");
                            return 0;  // bad crc
                        }
                        return input_pos;  // return received length
                    }  // end if have ETX already

                    // keep adding if not full
                    if (input_pos < length) {
//                        Serial.println("Keep adding");
                        data[input_pos++] = current_byte;
                    } else {
//                        Serial.println("Overflow");
                        return 0;  // overflow
                    }
                    break;

            }  // end of switch
        }  // end of incoming data
    } // end of while not timed out
    Serial.println("timeout");
    return 0;  // timeout
} // end of recvMsg

