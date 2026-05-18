/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/Serial.hpp>

#include <inttypes.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using device::Serial;

static volatile sig_atomic_t g_should_exit = 0;

static void signal_handler(int)
{
	g_should_exit = 1;
}

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		"Receive RK3588 messages from a UART and print them as both text and hex.\n"
		"This is intended for quick end-to-end validation of RK3588 -> flight controller\n"
		"serial communication. By default it listens on ttyS2."
	);

	PRINT_MODULE_USAGE_NAME_SIMPLE("rk3588_uart_test", "command");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS2", "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('i', "Enable RX/TX inversion", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('x', "Print hex only", true);
	PRINT_MODULE_USAGE_ARG("[device]", "Optional serial device path", true);
	PRINT_MODULE_USAGE_ARG("[baudrate]", "Optional baudrate", true);
}

static void print_hex_bytes(const uint8_t *buffer, size_t length)
{
	printf("HEX:");

	for (size_t i = 0; i < length; ++i) {
		printf(" %02x", buffer[i]);
	}

	printf("\n");
}

static void print_text_bytes(const uint8_t *buffer, size_t length)
{
	printf("TXT:");

	for (size_t i = 0; i < length; ++i) {
		const char c = (buffer[i] >= 32 && buffer[i] <= 126) ? static_cast<char>(buffer[i]) : '.';
		putchar(c);
	}

	printf("\n");
}

extern "C" __EXPORT int rk3588_uart_test_main(int argc, char *argv[]);

int rk3588_uart_test_main(int argc, char *argv[])
{
	g_should_exit = 0;

	const char *device = "/dev/ttyS2";
	int baudrate = 115200;
	bool invert = false;
	bool hex_only = false;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	int ch;

	while ((ch = px4_getopt(argc, argv, "d:b:ixh", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device = myoptarg;
			break;

		case 'b':
			baudrate = atoi(myoptarg);
			break;

		case 'i':
			invert = true;
			break;

		case 'x':
			hex_only = true;
			break;

		case 'h':
		default:
			print_usage();
			return 1;
		}
	}

	if (myoptind < argc) {
		device = argv[myoptind++];
	}

	if (myoptind < argc) {
		baudrate = atoi(argv[myoptind++]);
	}

	if (myoptind < argc || baudrate <= 0) {
		print_usage();
		return 1;
	}

	Serial serial(device, static_cast<uint32_t>(baudrate));

	if (invert) {
		serial.setInvertedMode(true);
	}

	if (!serial.open()) {
		PX4_ERR("failed to open %s at %d", device, baudrate);
		return 1;
	}

	struct sigaction sa {};
	sa.sa_handler = signal_handler;
	sigaction(SIGINT, &sa, nullptr);

	PX4_INFO("listening on %s @ %d, Ctrl-C to stop", device, baudrate);

	uint8_t read_buffer[128] {};
	uint32_t packet_count = 0;
	uint32_t total_bytes = 0;

	while (!g_should_exit) {
		const ssize_t bytes_read = serial.readAtLeast(read_buffer, sizeof(read_buffer), 1, 200000);

		if (bytes_read < 0) {
			PX4_ERR("read failed");
			serial.close();
			return 1;
		}

		if (bytes_read == 0) {
			continue;
		}

		total_bytes += static_cast<uint32_t>(bytes_read);
		++packet_count;

		printf("RX[%" PRIu32 "] %zd bytes (total=%" PRIu32 ")\n", packet_count, bytes_read, total_bytes);
		print_hex_bytes(read_buffer, static_cast<size_t>(bytes_read));

		if (!hex_only) {
			print_text_bytes(read_buffer, static_cast<size_t>(bytes_read));
		}

		fflush(stdout);
	}

	serial.close();
	putchar('\n');
	return 0;
}
