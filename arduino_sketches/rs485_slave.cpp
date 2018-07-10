//
// Created by Andreas Lydakis on 10/07/18.
//
/**
 * Import some needed libraries
 * we need to import everything we use here
 * to develop in an IDE other than the Arduino IDE
 */
#include <HardwareSerial.h>
#include <RS485_protocol.h>

//Use these pins for Serial1 interface
#define Serial1RX 19 //Serial1 Receive pin
#define Serial1TX 18 //Serial1 Transmit pin
#define HSerialTxControl 3   //RS485 Direction control

#define RS485Transmit HIGH
#define RS485Receive LOW
#define Pin13LED 13

#define MASTER_ID 0
#define MY_ID 1

const byte ENABLE_PIN = 3;

/*
*   TX_MSG: Outgoing packet
*   ------------------------
*   3 bytes long:
*   N1: Recipient ID (high nibble)
*   N2: Sender ID (low nibble)
*   B2-B4: Int16 (low to high byte)
*
*   +------+------+------+------+
*   |  N1  |  N2  |  B2  |  B3  |
*   +------+------+------+------+
*   |      B1     |
*   +-------------+
*
*
*   RX_MSG: Incoming packet
*   ------------------------
*   5 bytes long:
*   N1: Recipient ID (high nibble)
*   N2: Sender ID (low nibble)
*   B3-B6: float payload (low to high byte)
*
*   +------+------+------+------+------+------+
*   |  N1  |  N2  |  B2  |  B3  |  B4  |  B5  |
*   +------+------+------+------+------+------+
*   |      B1     |
*   +-------------+
*
*/

/*-----( Declare Variables )-----*/
int byteReceived;
int byteSend;

/**
 * Write function that is used in RS485.
 * In this case it is the write function of
 * the Serial interface used
 * @param what a byte to write to the output interface
 */
void fWrite(const byte what) {
    Serial1.write(what);
}

/**
 * Function that checks if there is available data to read in the buffer
 * In this case it is the available() function provided with the
 * Arduino Serial Interfaces
 * @return the number of bytes available to read
 */
int fAvailable() {
    return Serial1.available();
}

/**
 * Function that reads data from the buffer
 * In this case our Serial interface's read function
 * @return the byte read from the buffer
 */
int fRead() {
    return Serial1.read();
}

/**
 * All the communication turned into a function
 * so it can be used in interrupts, events etc.
 */
static void commFunction() {

    byte buf[4];
    //Try to receive message
    byte received = recvMsg(fAvailable, fRead, buf, sizeof(buf), 2000);
    if (received) {
        Serial.println("Sucessfully Received something");
        //Check if high nibble corresponds to MY_ID and low nibble to MASTER_ID
        if ((0x0f & (buf[0] >> 4)) != MY_ID ){
            Serial.println("Not my ID");
            return;
        }
        if((0x0f & buf[0]) != MASTER_ID) {
            Serial.println("Not my master");
            return;
        }
        //Compose a response
        byte header = 0x0f & MY_ID;
        byte msg[] = {
                header,
                buf[1], buf[2], // Return int16 payload to confirm errorless transmission
                128, 128        // Placeholder payload
        };
        Serial.println("Sending back");
        delayMicroseconds(100);
        digitalWrite(ENABLE_PIN, HIGH);  // enable sending
        sendMsg(fWrite, msg, sizeof msg); // send message down the line

        Serial1.flush();
        digitalWrite(ENABLE_PIN, LOW);  // disable sending
    }
}//--(end commFunction)--


/**
 * Arduino setup function, happens once before anything
 */
void setup() {
    // Start the built-in serial port, probably to Serial Monitor
    Serial.begin(115200);
    Serial1.begin(115200);

    pinMode(Pin13LED, OUTPUT);
//    pinMode(SSerialTxControl, OUTPUT);
    pinMode(HSerialTxControl, OUTPUT);

    digitalWrite(HSerialTxControl, RS485Receive);  // Init Transceiver
}//--(end setup)--

/**
 * Arduino's loop function
 */
void loop() {
    commFunction();

}//--(end loop)--

