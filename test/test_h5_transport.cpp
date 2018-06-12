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

#define BOOST_TEST_MODULE h5_transport
#include <boost/test/unit_test.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "transport.h"
#include "h5_transport.h"
#include "h5.h"
#include "nrf_error.h"

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

std::string convertToString(const std::vector<uint8_t> data) {
    std::stringstream ss;
    ss << std::hex << std::setfill('0');

    for (uint8_t const& value : data) {
        ss << std::setw(2) << std::hex << static_cast<int>(value) << ' ';
    }

    return ss.str();
}


class VirtualTransportSendSync : public Transport
{
public:
    explicit VirtualTransportSendSync(): Transport(), pushData(false) {}

    uint32_t open(status_cb_t status_callback, data_cb_t data_callback, log_cb_t log_callback) 
    {
        Transport::open(status_callback, data_callback, log_callback);
        pushData = true;

        auto inboundDataThread = [=]() -> void {
            std::vector<uint8_t> syncPacket { 0xc0, 0x00, 0x2f, 0x00, 0xd1, 0x01, 0x7e, 0xc0 };

            while (pushData) {
                std::this_thread::sleep_for(250ms);
                Transport::dataCallback(syncPacket.data(), syncPacket.size());
            }
        };

        dataPusher = std::thread(inboundDataThread);

        return NRF_SUCCESS;
    }

    uint32_t close()
    {
        pushData = false;
        dataPusher.join();

        Transport::close();
        return NRF_SUCCESS;
    }

    uint32_t send(std::vector<uint8_t> &data)
    {
        BOOST_TEST_MESSAGE("-> " << convertToString(data) << " length: " << data.size());
        return NRF_SUCCESS;
    }

private:
    std::thread dataPusher;
    bool pushData;
};

BOOST_AUTO_TEST_CASE(fail_open_invalid_inbound)
{
    auto status_callback = [](sd_rpc_app_status_t code, const char *message) -> void {
        BOOST_TEST_MESSAGE("[status] code: " << code << " message: " << message);
    };

    auto log_callback = [&](sd_rpc_log_severity_t severity, std::string &message) -> void {
        BOOST_TEST_MESSAGE("[log] severity: " << severity << " message: " << message);
    };

    auto lowerTransport = new VirtualTransportSendSync();
    auto h5Transport = new H5Transport(lowerTransport, 250);

    auto result = h5Transport->open(
        status_callback,
        [&](uint8_t *data, size_t length) -> void {
            std::vector<uint8_t> inbound;
            inbound.assign(data, data + length);
            BOOST_TEST_MESSAGE("<- " << convertToString(inbound) << " length: " << length);
        },
        log_callback
        );

    BOOST_TEST_REQUIRE(result == NRF_ERROR_TIMEOUT);

    h5Transport->close();
    BOOST_TEST_MESSAGE("H5Transport closed");

    delete h5Transport;
}
