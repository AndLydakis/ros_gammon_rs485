//
// Created by lydakis on 14/06/18.
//

#include "RS485_port.h"
#include <chrono>

typedef uint8_t byte;

const byte STX = '\2';
const byte ETX = '\3';

/**
 * Return the current time in millis as the Arduino
 * function does
 * @return current time in millis
 */
long millis() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
}

/**
 * Calculate a checksum for a packet
 * @param addr the message as a byte array
 * @param len the length of the message
 * @return the checksum
 */
static byte crc8(const byte *addr, byte len) {
    byte crc = 0;
    while (len--) {
        byte inbyte = *addr++;
        for (byte i = 8; i; i--) {
            byte mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8c;
            inbyte >>= 1;
        }
    }
    return crc;
}

/**
 * Complement a byte and send it down the line
 * @param port a serial::Serial port used for communication with the arduin
 * @param what a byte to send
 */
void sendComplemented(serial::Serial &port,
                      const byte what) {
    byte c;
    // first nibble
    c = what >> 4;
    byte s = (c << 4) | (c ^ 0x0F);
    port.write(&s, sizeof(s));
    // std::cout << "Sending First Nibble: " << s << std::endl;
    // second nibble
    c = what & 0x0F;
    s = (c << 4) | (c ^ 0x0F);
    // std::cout << "Sending Second Nibble: " << s << std::endl;
    port.write(&s, sizeof(s));

}// end of sendComplemented

/**
 * Send a message down the line
 * @param port the serial::Serial port used to send the message
 * @param data the message to be sent as a byte array
 * @param length the length of the message
 */
void sendMsg(serial::Serial &port,
             const byte *data,
             byte length) {
    port.write(&STX, sizeof(STX));  // STX

    for (byte i = 0; i < length; ++i)
        sendComplemented(port, data[i]);
    port.write(&ETX, sizeof(ETX));  // STX
    sendComplemented(port, crc8(data, length));
}

/**
 * Receive a message from some device
 * @param port the serial::Serial port listening for the message
 * @param data the buffer to write incoming data to
 * @param length the length of the buffer
 * @param timeout milliseconds before timing out
 * @return
 */
byte recvMsg(serial::Serial &port,             // read one byte
             byte *data,                    // buffer to receive into
             const byte length,              // maximum buffer size
             unsigned long timeout)          // milliseconds before timing out
{
    unsigned long start_time = millis();

    bool have_stx = false;

    // variables below are set when we get an STX
    bool have_etx;
    byte input_pos;
    bool first_nibble;
    byte current_byte;


    //std::cout << "Starting to receive message at " << start_time << std::endl;
    //While not timed out
    while (millis() - start_time < timeout) {
        //poll the port
        if (port.available()) {
//            std::cout << "Data available at port" << std::endl;
//            std::cout << port.read(5) << std::endl;
            byte inbyte;
            port.read(&inbyte, 1);
            switch (inbyte) {
                case STX:   // start of text
//                    std::cout << "Found Start of Packet" << std::endl;
                    have_stx = true;
                    have_etx = false;
                    input_pos = 0;
                    first_nibble = true;
                    start_time = millis();  // reset timeout period
                    break;
                case ETX:   // end of text
//                    std::cout << "Found End of Packet\n";
                    have_etx = true;
                    break;

                default:
//                    std::cout << "Reading middle bytes\n";
                    // wait until packet officially starts
                    if (!have_stx)
                        break;

                    // check byte is in valid form (4 bits followed by 4 bits complemented)
                    if ((inbyte >> 4) != ((inbyte & 0x0F) ^ 0x0F)) {
                        std::cout << "Invalid Character, Returning\n";
                        return 0;  // bad character
                    }

                    // convert back
                    inbyte >>= 4;

                    // high-order nibble?
                    if (first_nibble) {
//                        std::cout << "Receiving High Order Nibble\n";
                        current_byte = inbyte;
                        first_nibble = false;
                        break;
                    }  // end of first nibble

                    // low-order nibble
//                    std::cout << "Receiving Low Order Nibble\n";
                    current_byte <<= 4;
                    current_byte |= inbyte;
                    first_nibble = true;

                    // if we have the ETX this must be the CRC
                    if (have_etx) {
                        if (crc8(data, input_pos) != current_byte) {
                            std::cerr << "Bad CRC\n";
                            return 0;  // bad crc
                        }
                        return input_pos;  // return received length
                    }  // end if have ETX already

                    // keep adding if not full
                    if (input_pos < length)
                        data[input_pos++] = current_byte;
                    else
                        return 0;  // overflow
                    break;

            }  // end of switch
        }  // end of incoming data
    } // end of while not timed out
//    std::cerr << "Receive timeout\n";
    return 0;  // timeout
} // end of recvMsg
