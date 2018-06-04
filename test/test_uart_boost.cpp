/*
 * Copyright (c) 2016 Nordic Semiconductor ASA
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 *   2. Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 *   3. Neither the name of Nordic Semiconductor ASA nor the names of other
 *   contributors to this software may be used to endorse or promote products
 *   derived from this software without specific prior written permission.
 *
 *   4. This software must only be used in or with a processor manufactured by Nordic
 *   Semiconductor ASA, or in or with a processor manufactured by a third party that
 *   is used in combination with a processor manufactured by Nordic Semiconductor.
 *
 *   5. Any software provided in binary or object form under this license must not be
 *   reverse engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#define BOOST_TEST_MODULE uart_boost
#include <boost/test/unit_test.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "uart_boost.h"

#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>
#include <thread>
#include <random>

#if defined(_MSC_VER)
// Disable warning "This function or variable may be unsafe. Consider using _dupenv_s instead."
#pragma warning(disable: 4996)
#endif

using namespace boost::unit_test;
using namespace std::chrono_literals;
using std::chrono::system_clock;

struct PortStats {
    uint32_t pktCount;
    uint32_t pktMaxSize;
    uint32_t pktMinSize;

    PortStats() : pktCount(0), pktMinSize(std::numeric_limits<uint32_t>::max()), pktMaxSize(0) {}
};

std::ostream& operator<<(std::ostream &out, PortStats stats) {
    out << "pkt_count: " << stats.pktCount
        << " pkt_max_size: " << stats.pktMaxSize
        << " pkt_min_size: ";

    if (stats.pktMinSize == std::numeric_limits<uint32_t>::max()) {
        out << "-";
    }
    else {
        out << stats.pktMinSize;
    }

    return out;
};

BOOST_AUTO_TEST_CASE(open_close)
{
    auto envPortA = std::getenv("BLE_DRIVER_TEST_SERIAL_PORT_A");
    BOOST_TEST_REQUIRE(envPortA != nullptr, "Environment variable BLE_DRIVER_TEST_SERIAL_PORT_A not set, terminating test.");
    std::string portA = std::getenv("BLE_DRIVER_TEST_SERIAL_PORT_A");

    auto envPortB = std::getenv("BLE_DRIVER_TEST_SERIAL_PORT_B");
    BOOST_TEST_REQUIRE(envPortB != nullptr, "Environment variable BLE_DRIVER_TEST_SERIAL_PORT_B not set, terminating test.");
    std::string portB = std::getenv("BLE_DRIVER_TEST_SERIAL_PORT_B");

    auto ap = UartCommunicationParameters();
    ap.baudRate = 1000000;
    ap.dataBits = UartDataBitsEight;
    ap.flowControl = UartFlowControlNone;
    ap.parity = UartParityNone;
    ap.portName = portA.c_str();
    ap.stopBits = UartStopBitsOne;
    auto a = new UartBoost(ap);

    auto bp = UartCommunicationParameters();
    bp.baudRate = 1000000;
    bp.dataBits = UartDataBitsEight;
    bp.flowControl = UartFlowControlNone;
    bp.parity = UartParityNone;
    bp.portName = portB.c_str();
    bp.stopBits = UartStopBitsOne;
    auto b = new UartBoost(bp);

    std::vector<uint8_t> receivedOnA;
    std::vector<uint8_t> receivedOnB;

    // Generate random values to send
    std::vector<uint8_t> sendOnB(10000);
    std::vector<uint8_t> sendOnA(10000);

    std::generate(sendOnB.begin(), sendOnB.end(), std::rand);
    std::generate(sendOnA.begin(), sendOnA.end(), std::rand);

    auto status_callback = [](sd_rpc_app_status_t code, const char *message) -> void {
        BOOST_TEST_MESSAGE("code: " << code << " message: " << message);
        BOOST_TEST_REQUIRE(code == NRF_SUCCESS, message);
    };

    auto log_callback = [&](sd_rpc_log_severity_t severity, std::string &message) -> void {
        BOOST_TEST_MESSAGE("severity: " << severity << " message: " << message);

        if (severity == 1) {
            char port[100];
            auto ret = std::sscanf(message.c_str(), "UART read operation on port %s aborted.", port);
            BOOST_TEST_REQUIRE(ret == 1);

            std::string port_(port);
            BOOST_TEST_REQUIRE((port_ == portA || port_ == portB));
        }
    };

    PortStats portAStats;
    PortStats portBStats;

    b->open(
        status_callback,
        [&](uint8_t *data, size_t length) -> void {
            receivedOnB.insert(receivedOnB.end(), data, data + length);

            if (portBStats.pktMaxSize < length) {
                portBStats.pktMaxSize = length;
            }

            if (portBStats.pktMinSize > length) {
                portBStats.pktMinSize = length;
            }

            portBStats.pktCount++;
        },
        log_callback
        );

    a->open(
        status_callback,
        [&](uint8_t *data, size_t length) -> void {
            receivedOnA.insert(receivedOnA.end(), data, data + length);

            if (portAStats.pktMaxSize < length) {
                portAStats.pktMaxSize = length;
            }

            if (portAStats.pktMinSize > length) {
                portAStats.pktMinSize = length;
            }

            portAStats.pktCount++;
        },
        log_callback
    );

    b->send(sendOnB);
    a->send(sendOnA);

    // Let the data be sent between the serial ports before closing
    std::this_thread::sleep_until(system_clock::now() + 1s);

    b->close();
    a->close();

    delete b;
    delete a;

    BOOST_TEST_REQUIRE(sendOnA == receivedOnB);
    BOOST_TEST_REQUIRE(sendOnB == receivedOnA);

    BOOST_TEST_MESSAGE("stats port A " << portAStats);
    BOOST_TEST_MESSAGE("stats port B " << portBStats);
}
