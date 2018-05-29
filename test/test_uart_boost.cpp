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

using namespace boost::unit_test;

BOOST_AUTO_TEST_CASE(open_close)
{
	auto ap = UartCommunicationParameters();
	ap.baudRate = 1000000;
	ap.dataBits = UartDataBitsEight;
	ap.flowControl = UartFlowControlNone;
	ap.parity = UartParityNone;
	ap.portName = "COM4";
	ap.stopBits = UartStopBitsOne;
	auto a = new UartBoost(ap);

	auto bp = UartCommunicationParameters();
	bp.baudRate = 1000000;
	bp.dataBits = UartDataBitsEight;
	bp.flowControl = UartFlowControlNone;
	bp.parity = UartParityNone;
	bp.portName = "COM3";
	bp.stopBits = UartStopBitsOne;
	auto b = new UartBoost(bp);

	b->open(
		[](sd_rpc_app_status_t code, const char *message) -> void {
			std::cout << "code: " << code << "message: " << message << std::endl;
		},
		[](uint8_t *data, size_t length) -> void {
			std::cout << "received data, length:" << length << std::endl;
		},
		[](sd_rpc_log_severity_t severity, std::string message) -> void {
			std::cout << "severity:" << severity << "message:" << message << std::endl;
		}
	);

	a->open(
		[](sd_rpc_app_status_t code, const char *message) -> void {
			std::cout << "code: " << code << "message: " << message << std::endl;
		},
		[](uint8_t *data, size_t length) -> void {
			std::cout << "received data, length:" << length << std::endl;
		},
		[](sd_rpc_log_severity_t severity, std::string message) -> void {
			std::cout << "severity:" << severity << "message:" << message << std::endl;
		}
	);

	b->send(std::vector<unsigned char> { 1, 2, 3 });
	a->send(std::vector<unsigned char> { 11, 12, 13 });

	b->close();
	a->close();

	delete b;
	delete a;

	BOOST_TEST(false);
}
