#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <vector>
#include <serial/serial.h>
#include <RS485_port.cpp>
#include <cmath>
#include <algorithm>

#include "cxxopts.hpp"

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


/*
 * Example CLI: ./ros_gammon_rs485 -p /dev/ttyUSB0 -r 5 -i 10000 -b 115200 -t 6 -s 2
 */

#define MASTER_ID 0
#define NO_SLAVES 2
#define TIME_OUT 6
#define NO_ITERS 10000
#define NO_TRIALS 5
#define BAUD 115200

using namespace std;
using namespace serial;
typedef uint8_t byte;

int main(int argc, char *argv[]) {
    cxxopts::Options options(argv[0], " -example command line options");
    options.add_options()
            ("p,port", "Communications Port", cxxopts::value<string>()->default_value("/dev/ttyUSB0"), "")
            ("r,runs", "Number of Runs", cxxopts::value<string>()->default_value("5"), "")
            ("i,iter", "Number of Messages per run", cxxopts::value<string>()->default_value("10000"), "")
            ("b,baud", "BaudRate of port", cxxopts::value<string>()->default_value("115200"), "")
            ("t,timeout", "RecvMsg timeout", cxxopts::value<string>()->default_value("6"), "")
            ("s,slaves", "Number of slave devices", cxxopts::value<string>()->default_value("6"), "");
    string decor = "#-----------------------------#\n";
    string port;
    int runs_, iter_, timeout_, slaves_;
    uint32_t baud_;
    try {
        auto result = options.parse(argc, argv);
        port = result["port"].as<string>();
        runs_ = stoi(result["runs"].as<string>());
        (runs_ > 0) ? runs_ : runs_ = NO_TRIALS;
        iter_ = stoi(result["iter"].as<string>());
        (iter_ > 0) ? iter_ : iter_ = NO_ITERS;
        baud_ = static_cast<uint32_t>(stoi(result["baud"].as<string>()));
        (baud_ > 0) ? baud_ : baud_ = BAUD;
        timeout_ = stoi(result["timeout"].as<string>());
        (timeout_ > 0) ? timeout_ : timeout_ = TIME_OUT;
        slaves_ = stoi(result["slaves"].as<string>());
        (slaves_ > 0) ? slaves_ : slaves_ = NO_SLAVES;
    } catch (const cxxopts::OptionException &oe) {
        cout << oe.what() << endl << endl << " *** usage *** " << endl;
        cout << options.help() << endl;
        return -1;
    }

    cout << decor;
    cout << "Starting RS485 Tests\n";
    cout << "Port: " << port << endl;
    cout << "Baudrate: " << baud_ << endl;
    cout << "Runs: " << runs_ << endl;
    cout << "Messages/Run: " << iter_ << endl;
    cout << decor;

    // Open, setup and test a communications port
    Serial *arduino_port;
    try {
        arduino_port = new serial::Serial(port, baud_, serial::Timeout::simpleTimeout(1000));
    } catch (serial::SerialException &se) {
        cout << "Serial Exception (Is the adapter plugged in ?)\n";
        cout << se.what() << endl;
        return -1;
    } catch (serial::IOException &ioe) {
        cerr << "IO Exception (Is the adapter plugged in ?)\n";
        cerr << ioe.what() << endl;
        return -1;
    }
    
    double overall_error_rate = 0.0;
    double overall_round_trip = 0.0;
    
    vector<double> round_trips (iter_);
    vector<double> means(runs_);
    vector<double> sdevs(runs_);
    
    for (uint16_t t = 0; t < runs_; ++t) {
        round_trips.clear();
        arduino_port->flush();
        uint16_t errors = 0;
	    uint16_t timeouts = 0;
        unsigned long start_time = 0;
        unsigned long total_time = 0;
        for (uint16_t i = 0; i < iter_; ++i) {
            byte low = (byte) (i & 0xff);
            byte high = (byte) ((i >> 8) & 0xff);
            start_time = millis();

            // Header high nibble is the SLAVE_ID and the low nibble is the MASTER_ID (0)
            byte tx_header = ((byte) ((i % slaves_) + 1) << 4);

            // Prepare and send payload
            const byte tx_msg[3] = {
                    tx_header,
                    low,
                    high
            };

            sendMsg(*arduino_port, tx_msg, sizeof(tx_msg));

            // Wait for response
            byte rx_msg[7];
            byte response = recvMsg(*arduino_port, rx_msg, sizeof(rx_msg), timeout_);

            // Confirm that there are no transmission errors
            if (response) {
                if (((rx_msg[0] & 0xf0) >> 4)!= MASTER_ID) { // High nibble must be the MASTER_ID
                    cerr << "Wrong MASTER_ID\n";
                    errors += 1;
                } else if ((rx_msg[0] & 0x0f) != (byte) (i % slaves_ + 1)) { // Low nibble must be the current SLAVE_ID
                    cerr << "Wrong SLAVE_ID: expected " << i % slaves_ + 1 << ", got " << rx_msg[1] << endl;
                    errors += 1;
//                    return -1;
                } else if (rx_msg[1] != low || rx_msg[2] != high) { // Confirm that payload was uncorrupted by noise
                    cerr << "Wrong COMP BYTES\n";
                    errors += 1;
                }
            } else {
                cerr << "No Response Received\n";
                timeouts += 1;
            }
            unsigned long cur_time = millis() - start_time;
            total_time += cur_time;
            round_trips.emplace_back(cur_time);
            arduino_port->flush();
        }

        // Statistics
        double standardDeviation = 0;
        double mean = (float) total_time / (float) iter_;
        double error_rate = (float) (errors + timeouts) / (float) (iter_) * 100.0;
        for (int i = 0; i < iter_; ++i) {
            standardDeviation += pow(round_trips[i] - mean, 2);
        }
        standardDeviation = sqrt(standardDeviation / (float) iter_);
        cout << "-------------- TRIAL # " << t + 1 << "-----------------\n";
        cout << "Number of messages: " << iter_ << endl;
        cout << "Number of I/O errors: " << errors << endl;
        cout << "Number of timeouts: " << timeouts << endl;
        cout << "Total time: " << total_time << "ms\n";
        cout << "Average roundtrip: " << mean << "ms\n";
        cout << "Max roundtrip: " << *max_element(round_trips.begin(), round_trips.end()) << "ms\n";
        cout << "Min roundtrip: " << *min_element(round_trips.begin(), round_trips.end()) << "ms\n";
        cout << "Roundtrip Standard Deviation: " << standardDeviation << "ms\n";
        cout << "Error rate: " << error_rate << "%\n";

        means.emplace_back(mean);
        sdevs.emplace_back(standardDeviation);
        overall_error_rate += error_rate;
        overall_round_trip += mean;
    }
    
    float mean = 0;
    float standardDeviation = 0;
    float avgStandardDeviation = 0;
    float error_rate = (float) overall_error_rate / (float) (runs_);
    for (int i = 0; i < runs_; ++i) mean += means[i] / (float) runs_;

    for (int i=0; i < runs_; ++i) avgStandardDeviation += sdevs[i]/ (float) runs_;

    for (int i = 0; i < runs_; ++i) standardDeviation += pow(means[i] - mean, 2);
    standardDeviation = sqrt(standardDeviation / (float) runs_);

    cout << "\nAfter " << runs_ << " trials:\n";
    cout << "Average roundtrip time: " << mean << "ms\n";
    cout << "Average Standard Deviation: " << avgStandardDeviation << "ms\n";
    cout << "Roundtrip Standard Deviation: " << standardDeviation << "ms\n";
    cout << "Error rate: " << error_rate << "%\n";
    return EXIT_SUCCESS;
}
