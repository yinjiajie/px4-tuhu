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

#include <drivers/drv_hrt.h>
#include <inttypes.h>
#include <mavlink.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/Serial.hpp>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parachute_status.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;
using device::Serial;

extern "C" __EXPORT int parachute_rs485_main(int argc, char *argv[]);

class ParachuteRS485 : public ModuleBase<ParachuteRS485>, public px4::ScheduledWorkItem
{
public:
	ParachuteRS485(const char *port, uint32_t baudrate, bool invert) :
		ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
		_serial(port, baudrate),
		_baudrate(baudrate),
		_invert(invert)
	{
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	}

	~ParachuteRS485() override
	{
		_serial.close();
	}

	static int task_spawn(int argc, char *argv[])
	{
		const char *device_path = "/dev/ttyS6";
		int baudrate = 115200;
		bool invert = false;
		int ch = '\0';
		int myoptind = 1;
		const char *myoptarg = nullptr;

		while ((ch = px4_getopt(argc, argv, "d:b:ih", &myoptind, &myoptarg)) != EOF) {
			switch (ch) {
			case 'd':
				device_path = myoptarg;
				break;

			case 'b':
				baudrate = atoi(myoptarg);
				break;

			case 'i':
				invert = true;
				break;

			case 'h':
			default:
				print_usage();
				return PX4_ERROR;
			}
		}

		if (myoptind < argc) {
			device_path = argv[myoptind++];
		}

		if (myoptind < argc) {
			baudrate = atoi(argv[myoptind++]);
		}

		if (myoptind < argc || baudrate <= 0 || !Serial::validatePort(device_path)) {
			print_usage();
			return PX4_ERROR;
		}

		ParachuteRS485 *instance = new ParachuteRS485(device_path, static_cast<uint32_t>(baudrate), invert);

		if (instance == nullptr) {
			PX4_ERR("allocation failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

		delete instance;
		_object.store(nullptr);
		_task_id = -1;
		return PX4_ERROR;
	}

	static int custom_command(int argc, char *argv[])
	{
		return print_usage("unknown command");
	}

	static int print_usage(const char *reason = nullptr)
	{
		if (reason != nullptr) {
			PX4_WARN("%s", reason);
		}

		PRINT_MODULE_DESCRIPTION(
			R"DESCR_STR(
### Description
RS485 parachute interface.

This module sends the latest `VEHICLE_CMD_DO_PARACHUTE` content to an external
parachute module at 10 Hz and decodes the 32-bit status reply into
`parachute_status`.

Transmit data uses the MAVLink `COMMAND_LONG` frame format with command
`MAV_CMD_DO_PARACHUTE`.

Receive data also uses MAVLink framing. The packed 32-bit parachute status
word is expected in `COMMAND_ACK.result_param2` for `MAV_CMD_DO_PARACHUTE`.
 )DESCR_STR");

		PRINT_MODULE_USAGE_NAME("parachute_rs485", "module");
		PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the RS485 parachute interface");
		PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS6", "<file:dev>", "Serial device", true);
		PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
		PRINT_MODULE_USAGE_PARAM_FLAG('i', "Enable RX/TX inversion", true);
		PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the RS485 parachute interface");
		PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print module status");
		return PX4_OK;
	}

	bool init()
	{
		if (_invert) {
			_serial.setInvertedMode(true);
		}

		if (!_serial.open()) {
			PX4_ERR("failed to open %s at %" PRIu32, _port, _baudrate);
			return false;
		}

		ScheduleOnInterval(kUpdateInterval);
		return true;
	}

	int print_status() override
	{
		PX4_INFO("device: %s @ %" PRIu32, _port, _baudrate);
		PX4_INFO("command_valid: %s", _command_valid ? "true" : "false");
		PX4_INFO("connected: %s", _connected ? "true" : "false");
		PX4_INFO("height: %.1f m", (double)_last_status.height_above_takeoff_m);
		PX4_INFO("voltage: %.1f V", (double)_last_status.voltage_v);
		PX4_INFO("release_source: %u flags: 0x%x state: 0x%x raw: 0x%08" PRIx32,
			 static_cast<unsigned>(_last_status.release_source),
			 static_cast<unsigned>(_last_status.flags),
			 static_cast<unsigned>(_last_status.state),
			 _last_status.raw);
		return PX4_OK;
	}

private:
	static constexpr uint32_t kUpdateInterval{100_ms};
	static constexpr uint32_t kReplyTimeoutUs{20000};
	static constexpr hrt_abstime kConnectionTimeout{500_ms};
	static constexpr size_t kTxPacketMaxSize{MAVLINK_MAX_PACKET_LEN};
	static constexpr size_t kRxBufferSize{MAVLINK_MAX_PACKET_LEN};

	void Run() override
	{
		if (should_exit()) {
			ScheduleClear();
			_serial.close();
			exit_and_cleanup();
			return;
		}

		update_command();

		if (!_command_valid) {
			publish_disconnected_if_needed(hrt_absolute_time());
			return;
		}

		const hrt_abstime now = hrt_absolute_time();
		uint8_t tx[kTxPacketMaxSize] {};
		const size_t tx_size = build_tx_mavlink_packet(_last_command, tx);

		if (_serial.write(tx, tx_size) != static_cast<ssize_t>(tx_size)) {
			PX4_WARN("write failed");
			publish_disconnected_if_needed(now);
			return;
		}

		_serial.flush();

		uint8_t rx[kRxBufferSize] {};
		const ssize_t bytes_read = _serial.readAtLeast(rx, sizeof(rx), 1, kReplyTimeoutUs);

		if (bytes_read > 0) {
			for (ssize_t i = 0; i < bytes_read; ++i) {
				mavlink_message_t message {};
				mavlink_status_t parse_status {};

				if (mavlink_frame_char_buffer(&_mavlink_rx_buffer, &_mavlink_rx_status, rx[i], &message, &parse_status)) {
					handle_mavlink_message(now, message);
				}
			}

		} else if (bytes_read < 0) {
			PX4_WARN("read failed");
			publish_disconnected_if_needed(now);

		} else {
			publish_disconnected_if_needed(now);
		}
	}

	void update_command()
	{
		vehicle_command_s command {};

		while (_vehicle_command_sub.update(&command)) {
			if (command.command == vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE) {
				_last_command = command;
				_command_valid = true;
			}
		}
	}

	static size_t build_tx_mavlink_packet(const vehicle_command_s &command, uint8_t (&packet)[kTxPacketMaxSize])
	{
		mavlink_message_t msg {};
		mavlink_msg_command_long_pack(
			command.source_system,
			static_cast<uint8_t>(command.source_component),
			&msg,
			command.target_system,
			command.target_component,
			static_cast<uint16_t>(command.command),
			command.confirmation,
			command.param1,
			command.param2,
			command.param3,
			command.param4,
			static_cast<float>(command.param5),
			static_cast<float>(command.param6),
			command.param7);

		return mavlink_msg_to_send_buffer(packet, &msg);
	}

	void handle_mavlink_message(hrt_abstime now, const mavlink_message_t &message)
	{
		switch (message.msgid) {
		case MAVLINK_MSG_ID_COMMAND_ACK: {
				mavlink_command_ack_t ack {};
				mavlink_msg_command_ack_decode(&message, &ack);

				if (ack.command == MAV_CMD_DO_PARACHUTE) {
					mark_connected(now);
					decode_and_publish(now, static_cast<uint32_t>(ack.result_param2));
				}

				break;
			}

		default:
			break;
		}
	}

	void decode_and_publish(hrt_abstime now, uint32_t raw)
	{
		parachute_status_s status {};
		status.timestamp = now;
		status.connected = true;
		status.height_above_takeoff_m = static_cast<float>(raw & 0xFFFFu) * 0.1f;
		status.voltage_v = static_cast<float>((raw >> 16) & 0x3Fu) * 0.1f;
		status.release_source = static_cast<uint8_t>((raw >> 22) & 0x03u);
		status.flags = static_cast<uint8_t>((raw >> 24) & 0x0Fu);
		status.state = static_cast<uint8_t>((raw >> 28) & 0x0Fu);
		status.raw = raw;
		_last_status = status;
		_last_reply_timestamp = now;
		_connected = true;
		_parachute_status_pub.publish(status);
	}

	void mark_connected(hrt_abstime now)
	{
		_last_reply_timestamp = now;
		_connected = true;
	}

	void publish_disconnected_if_needed(hrt_abstime now)
	{
		if ((now <= _last_reply_timestamp + kConnectionTimeout) || !_last_status.connected) {
			return;
		}

		_connected = false;
		parachute_status_s status = _last_status;
		status.timestamp = now;
		status.connected = false;
		_last_status = status;
		_parachute_status_pub.publish(status);
	}

	Serial _serial {};
	char _port[32] {};
	const uint32_t _baudrate;
	const bool _invert;

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Publication<parachute_status_s> _parachute_status_pub{ORB_ID(parachute_status)};

	vehicle_command_s _last_command {};
	parachute_status_s _last_status {};
	mavlink_message_t _mavlink_rx_buffer {};
	mavlink_status_t _mavlink_rx_status {};
	hrt_abstime _last_reply_timestamp{0};
	bool _command_valid{false};
	bool _connected{false};
};

int parachute_rs485_main(int argc, char *argv[])
{
	return ParachuteRS485::main(argc, argv);
}
