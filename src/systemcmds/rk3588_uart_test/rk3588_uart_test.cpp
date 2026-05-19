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
#include <px4_platform_common/tasks.h>

#include <errno.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

using device::Serial;

namespace rk3588_uart_test
{

struct Options {
	char device[32]{"/dev/ttyS2"};
	int baudrate{115200};
	int idle_timeout_ms{3000};
	bool invert{false};
	bool listen_forever{false};
	bool hex_only{false};
};

static volatile bool g_thread_should_exit = false;
static bool g_thread_running = false;
static int g_daemon_task = -1;
static Options g_options{};

static void print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		"Receive RK3588 messages from a UART and print them as both text and hex.\n"
		"This is intended for quick end-to-end validation of RK3588 -> flight controller\n"
		"serial communication. By default it listens on ttyS2 and exits after a short idle timeout."
	);

	PRINT_MODULE_USAGE_NAME_SIMPLE("rk3588_uart_test", "command");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS2", "<file:dev>", "Serial device", true);
	PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
	PRINT_MODULE_USAGE_PARAM_INT('t', 3000, 0, 60000, "Exit after this many ms without RX data (0 disables timeout)", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('i', "Enable RX/TX inversion", true);
	PRINT_MODULE_USAGE_PARAM_FLAG('f', "Listen forever. In direct mode this starts a background worker; stop it with 'rk3588_uart_test stop'", true);
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

static bool parse_options(int argc, char *argv[], const Options &defaults, Options &options)
{
	options = defaults;

	int myoptind = 1;
	const char *myoptarg = nullptr;
	int ch;

	while ((ch = px4_getopt(argc, argv, "d:b:t:ifxh", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			strncpy(options.device, myoptarg, sizeof(options.device) - 1);
			options.device[sizeof(options.device) - 1] = '\0';
			break;

		case 'b':
			options.baudrate = atoi(myoptarg);
			break;

		case 't':
			options.idle_timeout_ms = atoi(myoptarg);
			options.listen_forever = (options.idle_timeout_ms == 0);
			break;

		case 'i':
			options.invert = true;
			break;

		case 'f':
			options.listen_forever = true;
			options.idle_timeout_ms = 0;
			break;

		case 'x':
			options.hex_only = true;
			break;

		case 'h':
		default:
			return false;
		}
	}

	if (myoptind < argc) {
		strncpy(options.device, argv[myoptind++], sizeof(options.device) - 1);
		options.device[sizeof(options.device) - 1] = '\0';
	}

	if (myoptind < argc) {
		options.baudrate = atoi(argv[myoptind++]);
	}

	return myoptind >= argc && options.baudrate > 0 && options.idle_timeout_ms >= 0;
}

static int run_uart_test(const Options &options)
{
	Serial serial(options.device, static_cast<uint32_t>(options.baudrate));

	if (options.invert) {
		serial.setInvertedMode(true);
	}

	if (!serial.open()) {
		PX4_ERR("failed to open %s at %d", options.device, options.baudrate);
		return 1;
	}

	const uint32_t read_timeout_ms = 200;

	if (options.listen_forever || options.idle_timeout_ms == 0) {
		PX4_INFO("listening on %s @ %d until stopped", options.device, options.baudrate);

	} else {
		PX4_INFO("listening on %s @ %d, idle timeout %d ms", options.device, options.baudrate, options.idle_timeout_ms);
	}

	uint8_t read_buffer[128] {};
	uint32_t packet_count = 0;
	uint32_t total_bytes = 0;
	uint32_t idle_elapsed_ms = 0;

	while (!g_thread_should_exit) {
		const ssize_t bytes_read = serial.readAtLeast(read_buffer, sizeof(read_buffer), 1, read_timeout_ms);

		if (bytes_read < 0) {
			PX4_ERR("read failed");
			serial.close();
			return 1;
		}

		if (bytes_read == 0) {
			if (!options.listen_forever && options.idle_timeout_ms > 0) {
				idle_elapsed_ms += read_timeout_ms;

				if (idle_elapsed_ms >= (uint32_t)options.idle_timeout_ms) {
					PX4_INFO("no RX data for %d ms, exiting", options.idle_timeout_ms);
					break;
				}
			}

			continue;
		}

		idle_elapsed_ms = 0;
		total_bytes += static_cast<uint32_t>(bytes_read);
		++packet_count;

		printf("RX[%" PRIu32 "] %zd bytes (total=%" PRIu32 ")\n", packet_count, bytes_read, total_bytes);
		print_hex_bytes(read_buffer, static_cast<size_t>(bytes_read));

		if (!options.hex_only) {
			print_text_bytes(read_buffer, static_cast<size_t>(bytes_read));
		}

		fflush(stdout);
	}

	serial.close();
	putchar('\n');
	return 0;
}

int rk3588_uart_test_thread_main(int argc, char *argv[])
{
	(void)argc;
	(void)argv;

	g_thread_running = true;
	const int ret = run_uart_test(g_options);
	g_thread_running = false;
	g_daemon_task = -1;
	return ret;
}

} // namespace rk3588_uart_test

extern "C" __EXPORT int rk3588_uart_test_main(int argc, char *argv[]);

int rk3588_uart_test_main(int argc, char *argv[])
{
	using namespace rk3588_uart_test;

	if (argc >= 2 && !strcmp(argv[1], "stop")) {
		g_thread_should_exit = true;

		if (!g_thread_running) {
			PX4_WARN("not running");
			return 1;
		}

		PX4_INFO("stop requested");
		return 0;
	}

	if (argc >= 2 && !strcmp(argv[1], "status")) {
		if (g_thread_running) {
			PX4_INFO("running on %s @ %d%s", g_options.device, g_options.baudrate, g_options.listen_forever ? " forever" : "");

		} else {
			PX4_INFO("not running");
		}

		return 0;
	}

	if (argc >= 2 && !strcmp(argv[1], "start")) {
		if (g_thread_running) {
			PX4_INFO("already running");
			return 0;
		}

		Options start_defaults{};
		start_defaults.listen_forever = true;
		start_defaults.idle_timeout_ms = 0;

		if (!parse_options(argc - 1, argv + 1, start_defaults, g_options)) {
			print_usage();
			return 1;
		}

		g_thread_should_exit = false;
		g_daemon_task = px4_task_spawn_cmd("rk3588_uart_test", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 4096,
						   rk3588_uart_test_thread_main, nullptr);

		if (g_daemon_task < 0) {
			PX4_ERR("task start failed (%d)", errno);
			g_daemon_task = -1;
			return 1;
		}

		PX4_INFO("started in background, use 'rk3588_uart_test stop' to stop");
		return 0;
	}

	Options direct_defaults{};
	Options direct_options{};

	if (!parse_options(argc, argv, direct_defaults, direct_options)) {
		print_usage();
		return 1;
	}

	if (direct_options.listen_forever) {
		if (g_thread_running) {
			PX4_INFO("already running");
			return 0;
		}

		g_options = direct_options;
		g_thread_should_exit = false;
		g_daemon_task = px4_task_spawn_cmd("rk3588_uart_test", SCHED_DEFAULT, SCHED_PRIORITY_DEFAULT, 4096,
						   rk3588_uart_test_thread_main, nullptr);

		if (g_daemon_task < 0) {
			PX4_ERR("task start failed (%d)", errno);
			g_daemon_task = -1;
			return 1;
		}

		PX4_INFO("started in background, use 'rk3588_uart_test stop' to stop");
		return 0;
	}

	g_thread_should_exit = false;
	return run_uart_test(direct_options);
}
