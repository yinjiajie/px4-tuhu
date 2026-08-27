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
#include <errno.h>
#include <inttypes.h>
#include <mavlink.h>
#include <math.h>
#include <stdio.h>
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
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/debug_array.h>
#include <uORB/topics/parachute_status.h>
#include <uORB/topics/vehicle_command.h>

using namespace time_literals;
using device::Serial;

extern "C" __EXPORT int parachute_rs485_main(int argc, char *argv[]);

namespace
{
static constexpr px4::wq_config_t parachute_rs485_wq{"wq:parachute_rs485", 6000, -18};
static constexpr uint16_t kParachuteDebugArrayId{0x5053};
static constexpr size_t kParachuteDebugArrayUsedFields{7};
static constexpr float kParachuteVoltageReadyMinV{4.0f};
static constexpr uint8_t kParachuteStateTriggered{5};
static constexpr uint8_t kParachuteStateReleased{6};
}

class ParachuteRS485 : public ModuleBase<ParachuteRS485>, public px4::ScheduledWorkItem
{
public:
	ParachuteRS485(const char *port, uint32_t baudrate, bool invert) :
		ScheduledWorkItem(MODULE_NAME, parachute_rs485_wq),
		_baudrate(baudrate),
		_invert(invert)
	{
		strncpy(_port, port, sizeof(_port) - 1);
		_port[sizeof(_port) - 1] = '\0';
	}

	~ParachuteRS485() override
	{
		close_serial();
	}

	static int task_spawn(int argc, char *argv[])
	{
		const char *device_path = "/dev/ttyS1";
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
		if (!is_running()) {
			print_usage("not running");
			return PX4_ERROR;
		}

		if (argc >= 1 && !strcmp(argv[0], "test")) {
			return get_instance()->test(argc, argv);
		}

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
		PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<file:dev>", "Serial device", true);
		PRINT_MODULE_USAGE_PARAM_INT('b', 115200, 0, 3000000, "Baudrate", true);
		PRINT_MODULE_USAGE_PARAM_FLAG('i', "Enable RX/TX inversion", true);
		PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the RS485 parachute interface");
		PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print module status");
		PRINT_MODULE_USAGE_COMMAND("test");
		PRINT_MODULE_USAGE_ARG("disable|enable|release", "Publish a test parachute command", false);
		return PX4_OK;
	}

	bool init()
	{
		initialize_default_command();
		publish_debug_status(hrt_absolute_time());
		ScheduleOnInterval(kUpdateInterval);
		return true;
	}

	int print_status() override
	{
		const uint32_t last_msgid = _last_msgid_valid ? _last_msgid : 0;
		const uint32_t last_ack_command = _last_ack_valid ? static_cast<uint32_t>(_last_ack_command) : 0;
		const uint32_t last_ack_result = _last_ack_valid ? static_cast<uint32_t>(_last_ack_result) : 0;
		const uint32_t last_rx_ms_ago = _last_rx_timestamp != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_last_rx_timestamp) / 1000) : 0;
		const uint32_t last_msg_ms_ago = _last_message_timestamp != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_last_message_timestamp) / 1000) : 0;
		const uint32_t last_ack_ms_ago = _last_ack_timestamp != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_last_ack_timestamp) / 1000) : 0;
		const uint32_t stage_ms_ago = _stage_timestamp != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_stage_timestamp) / 1000) : 0;
		const uint32_t last_run_start_ms_ago = _last_run_start != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_last_run_start) / 1000) : 0;
		const uint32_t last_run_complete_ms_ago = _last_run_complete != 0 ? static_cast<uint32_t>(hrt_elapsed_time(&_last_run_complete) / 1000) : 0;

		PX4_INFO("device: %s @ %" PRIu32, _port, _baudrate);
		PX4_INFO("inverted: %s", _invert ? "true" : "false");
		PX4_INFO("command_valid: %s", _command_valid ? "true" : "false");
		PX4_INFO("connected: %s", _connected ? "true" : "false");
		PX4_INFO("run_stage: %s stage_ms_ago: %s%" PRIu32 " iter: %" PRIu32,
			 stage_name(_run_stage),
			 _stage_timestamp != 0 ? "" : "n/a ",
			 stage_ms_ago,
			 _run_iteration);
		PX4_INFO("last_run_start_ms_ago: %s%" PRIu32 " last_run_complete_ms_ago: %s%" PRIu32,
			 _last_run_start != 0 ? "" : "n/a ",
			 last_run_start_ms_ago,
			 _last_run_complete != 0 ? "" : "n/a ",
			 last_run_complete_ms_ago);
		PX4_INFO("tx_count: %" PRIu32 " last_action: %.0f", _tx_count, (double)_last_command.param1);
		PX4_INFO("rx_reads: %" PRIu32 " timeouts: %" PRIu32 " read_errors: %" PRIu32 " last_read: %" PRId32,
			 _rx_read_count, _rx_timeout_count, _read_error_count, _last_read_result);
		PX4_INFO("rx_bytes: %" PRIu32 " rx_msgs: %" PRIu32 " ack_count: %" PRIu32 " parachute_ack: %" PRIu32,
			 _rx_byte_count, _rx_message_count, _ack_count, _parachute_ack_count);
		PX4_INFO("parse_errors: %" PRIu32 " buffer_overruns: %" PRIu32 " packet_drops: %" PRIu32,
			 _parse_error_count, _buffer_overrun_count, _packet_drop_count);
		PX4_INFO("last_msgid: %s%" PRIu32 " last_ack_cmd: %s%" PRIu32 " last_ack_result: %s%" PRIu32,
			 _last_msgid_valid ? "" : "n/a ",
			 last_msgid,
			 _last_ack_valid ? "" : "n/a ",
			 last_ack_command,
			 _last_ack_valid ? "" : "n/a ",
			 last_ack_result);
		PX4_INFO("last_rx_ms_ago: %s%" PRIu32 " last_msg_ms_ago: %s%" PRIu32 " last_ack_ms_ago: %s%" PRIu32,
			 _last_rx_timestamp != 0 ? "" : "n/a ",
			 last_rx_ms_ago,
			 _last_message_timestamp != 0 ? "" : "n/a ",
			 last_msg_ms_ago,
			 _last_ack_timestamp != 0 ? "" : "n/a ",
			 last_ack_ms_ago);
		PX4_INFO("height: %.1f m", (double)_last_status.height_above_takeoff_m);
		PX4_INFO("voltage: %.1f V", (double)_last_status.voltage_v);
		PX4_INFO("release_source: %u flags: 0x%x state: 0x%x raw: 0x%08" PRIx32,
			 static_cast<unsigned>(_last_status.release_source),
			 static_cast<unsigned>(_last_status.flags),
			 static_cast<unsigned>(_last_status.state),
			 _last_status.raw);
		PX4_INFO("last_tx_size: %zu bytes", _last_tx_size);
		print_packet_bytes("last_tx_packet", _tx_packet, _last_tx_size);
		return PX4_OK;
	}

private:
	static constexpr uint32_t kUpdateInterval{100_ms};
	// The Serial API names this timeout argument in microseconds, but PX4 NuttX
	// UART callers pass millisecond-style values here and the implementation
	// converts them through poll() accordingly.
	static constexpr uint32_t kReplyTimeoutMs{20};
	static constexpr hrt_abstime kReplyTimeoutUs{static_cast<hrt_abstime>(kReplyTimeoutMs) * 1000ULL};
	static constexpr hrt_abstime kConnectionTimeout{500_ms};
	static constexpr size_t kTxPacketMaxSize{MAVLINK_MAX_PACKET_LEN};
	static constexpr size_t kRxBufferSize{MAVLINK_MAX_PACKET_LEN};

	enum class RunStage : uint8_t {
		Idle = 0,
		OpenSerial,
		UpdateCommand,
		BuildPacket,
		WritePacket,
		WaitTx,
		ReadReply,
		ParseReply
	};

	static const char *stage_name(RunStage stage)
	{
		switch (stage) {
		case RunStage::Idle: return "idle";
		case RunStage::OpenSerial: return "open";
		case RunStage::UpdateCommand: return "update_cmd";
		case RunStage::BuildPacket: return "build";
		case RunStage::WritePacket: return "write";
		case RunStage::WaitTx: return "wait_tx";
		case RunStage::ReadReply: return "read";
		case RunStage::ParseReply: return "parse";
		}

		return "unknown";
	}

	enum class DebugStatusCode : uint8_t {
		NotInstalled = 0,
		Ready = 1,
		Triggered = 2,
		Error = 3
	};

	void set_stage(RunStage stage)
	{
		_run_stage = stage;
		_stage_timestamp = hrt_absolute_time();
	}

	bool open_serial()
	{
		if (_serial == nullptr) {
			_serial = new Serial(_port, _baudrate);

			if (_serial == nullptr) {
				PX4_ERR("serial alloc failed");
				return false;
			}
		}

		if (_serial->isOpen()) {
			return true;
		}

		if (!_serial->setBaudrate(_baudrate)) {
			PX4_ERR("failed to set baudrate %" PRIu32, _baudrate);
			return false;
		}

		if (_invert) {
			_serial->setInvertedMode(true);
		}

		if (!_serial->open()) {
			PX4_ERR("failed to open %s at %" PRIu32, _port, _baudrate);
			return false;
		}

		return true;
	}

	void initialize_default_command()
	{
		_last_command.timestamp = hrt_absolute_time();
		_last_command.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
		_last_command.param1 = static_cast<float>(vehicle_command_s::PARACHUTE_ACTION_DISABLE);
		_last_command.source_system = 1;
		_last_command.target_system = 1;
		_last_command.source_component = 1;
		_last_command.target_component = MAV_COMP_ID_PARACHUTE;
		_command_valid = true;
	}

	ssize_t read_reply_with_timeout(uint8_t *buffer, size_t buffer_size, hrt_abstime timeout_us)
	{
		const hrt_abstime deadline = hrt_absolute_time() + timeout_us;
		size_t total_bytes_read = 0;

		while (hrt_absolute_time() < deadline && total_bytes_read < buffer_size) {
			const ssize_t ret = _serial->read(&buffer[total_bytes_read], buffer_size - total_bytes_read);

			if (ret > 0) {
				total_bytes_read += static_cast<size_t>(ret);
				continue;
			}

			if ((ret < 0) && (errno != EAGAIN) && (errno != EWOULDBLOCK) && (errno != EINTR)) {
				return -1;
			}

			if (total_bytes_read > 0) {
				break;
			}

			px4_usleep(1_ms);
		}

		return static_cast<ssize_t>(total_bytes_read);
	}

	void close_serial()
	{
		if (_serial != nullptr) {
			_serial->close();
			delete _serial;
			_serial = nullptr;
		}
	}

	int test(int argc, char *argv[])
	{
		if (argc < 2) {
			return print_usage("missing test action");
		}

		uint8_t parachute_action{};

		if (!strcmp(argv[1], "disable")) {
			parachute_action = vehicle_command_s::PARACHUTE_ACTION_DISABLE;

		} else if (!strcmp(argv[1], "enable")) {
			parachute_action = vehicle_command_s::PARACHUTE_ACTION_ENABLE;

		} else if (!strcmp(argv[1], "release")) {
			parachute_action = vehicle_command_s::PARACHUTE_ACTION_RELEASE;

		} else {
			return print_usage("unknown test action");
		}

		vehicle_command_s vcmd{};
		vcmd.timestamp = hrt_absolute_time();
		vcmd.command = vehicle_command_s::VEHICLE_CMD_DO_PARACHUTE;
		vcmd.param1 = static_cast<float>(parachute_action);
		vcmd.source_system = 1;
		vcmd.target_system = 1;
		vcmd.source_component = 1;
		vcmd.target_component = MAV_COMP_ID_PARACHUTE;

		uORB::Publication<vehicle_command_s> vcmd_pub{ORB_ID(vehicle_command)};
		vcmd_pub.publish(vcmd);

		ScheduleNow();
		PX4_INFO("published test command: %s", argv[1]);
		return PX4_OK;
	}

	DebugStatusCode get_debug_status_code(const parachute_status_s &status) const
	{
		if (!status.connected) {
			return DebugStatusCode::NotInstalled;
		}

		if ((status.state == kParachuteStateTriggered) || (status.state == kParachuteStateReleased)) {
			return DebugStatusCode::Triggered;
		}

		const bool voltage_valid = PX4_ISFINITE(status.voltage_v) && (status.voltage_v >= kParachuteVoltageReadyMinV);
		return voltage_valid ? DebugStatusCode::Ready : DebugStatusCode::Error;
	}

	void publish_debug_status(hrt_abstime now)
	{
		debug_array_s debug{};
		debug.timestamp = now;
		debug.id = kParachuteDebugArrayId;
		strncpy(debug.name, "para_stat", sizeof(debug.name) - 1);

		const bool connected = _last_status.connected;
		debug.data[0] = static_cast<float>(static_cast<uint8_t>(get_debug_status_code(_last_status)));
		debug.data[1] = connected ? 1.f : 0.f;
		debug.data[2] = connected ? _last_status.voltage_v : NAN;
		debug.data[3] = connected ? _last_status.height_above_takeoff_m : NAN;
		debug.data[4] = connected ? static_cast<float>(_last_status.release_source) : NAN;
		debug.data[5] = connected ? static_cast<float>(_last_status.flags) : NAN;
		debug.data[6] = connected ? static_cast<float>(_last_status.state) : NAN;

		for (size_t i = kParachuteDebugArrayUsedFields; i < debug_array_s::ARRAY_SIZE; ++i) {
			debug.data[i] = NAN;
		}

		_debug_array_pub.publish(debug);
	}

	void Run() override
	{
		_last_run_start = hrt_absolute_time();
		++_run_iteration;

		if (should_exit()) {
			ScheduleClear();
			close_serial();
			exit_and_cleanup();
			return;
		}

		set_stage(RunStage::OpenSerial);
		if (!open_serial()) {
			const hrt_abstime now = hrt_absolute_time();
			publish_disconnected_if_needed(now);
			publish_debug_status(now);
			return;
		}

		set_stage(RunStage::UpdateCommand);
		update_command();

		if (!_command_valid) {
			const hrt_abstime now = hrt_absolute_time();
			publish_disconnected_if_needed(now);
			set_stage(RunStage::Idle);
			_last_run_complete = hrt_absolute_time();
			publish_debug_status(now);
			return;
		}

		const hrt_abstime now = hrt_absolute_time();
		set_stage(RunStage::BuildPacket);
		const size_t tx_size = build_tx_mavlink_packet(_last_command, _tx_packet);
		_last_tx_size = tx_size;

		set_stage(RunStage::WritePacket);
		if (_serial->write(_tx_packet, tx_size) != static_cast<ssize_t>(tx_size)) {
			PX4_WARN("write failed");
			close_serial();
			publish_disconnected_if_needed(now);
			publish_debug_status(now);
			return;
		}

		++_tx_count;
		// Serial::flush() maps to tcflush(TCIOFLUSH) on NuttX and drops queued TX/RX data.
		// That breaks half-duplex request/reply exchanges on UART-backed RS485 adapters.
		const uint32_t tx_wire_time_us = 500 + static_cast<uint32_t>((tx_size * 1000000ULL * 10) / _baudrate);
		set_stage(RunStage::WaitTx);
		px4_usleep(tx_wire_time_us);

		set_stage(RunStage::ReadReply);
		const ssize_t bytes_read = read_reply_with_timeout(_rx_buffer, sizeof(_rx_buffer), kReplyTimeoutUs);
		_last_read_result = static_cast<int32_t>(bytes_read);

		if (bytes_read > 0) {
			++_rx_read_count;
			_rx_byte_count += static_cast<uint32_t>(bytes_read);
			_last_rx_timestamp = now;

			const uint8_t parse_error_before = _mavlink_parse_status.parse_error;
			const uint8_t buffer_overrun_before = _mavlink_parse_status.buffer_overrun;
			const uint16_t packet_drop_before = _mavlink_parse_status.packet_rx_drop_count;

			set_stage(RunStage::ParseReply);
			for (ssize_t i = 0; i < bytes_read; ++i) {
				if (mavlink_frame_char_buffer(&_mavlink_rx_buffer, &_mavlink_rx_status, _rx_buffer[i], &_mavlink_message,
								     &_mavlink_parse_status)) {
					++_rx_message_count;
					_last_msgid = _mavlink_message.msgid;
					_last_msgid_valid = true;
					_last_message_timestamp = now;
					handle_mavlink_message(now, _mavlink_message);
				}
			}

			_parse_error_count += counter_delta(_mavlink_parse_status.parse_error, parse_error_before);
			_buffer_overrun_count += counter_delta(_mavlink_parse_status.buffer_overrun, buffer_overrun_before);
			_packet_drop_count += counter_delta(_mavlink_parse_status.packet_rx_drop_count, packet_drop_before);

		} else if (bytes_read < 0) {
			++_read_error_count;
			PX4_WARN("read failed");
			close_serial();
			publish_disconnected_if_needed(now);

		} else {
			++_rx_timeout_count;
			publish_disconnected_if_needed(now);
		}

		set_stage(RunStage::Idle);
		_last_run_complete = hrt_absolute_time();
		publish_debug_status(hrt_absolute_time());
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

		// Serialize a fixed-width MAVLink 2 COMMAND_LONG frame so the trailing zero confirmation byte
		// is kept on the wire for parachute modules that expect a constant 45-byte packet.
		const uint8_t payload_length = MAVLINK_MSG_ID_COMMAND_LONG_LEN;
		packet[0] = MAVLINK_STX;
		packet[1] = payload_length;
		packet[2] = 0; // incompat_flags
		packet[3] = 0; // compat_flags
		packet[4] = msg.seq;
		packet[5] = msg.sysid;
		packet[6] = msg.compid;
		packet[7] = msg.msgid & 0xFF;
		packet[8] = (msg.msgid >> 8) & 0xFF;
		packet[9] = (msg.msgid >> 16) & 0xFF;
		memcpy(&packet[10], _MAV_PAYLOAD(&msg), payload_length);

		uint16_t checksum = crc_calculate(&packet[1], MAVLINK_CORE_HEADER_LEN);
		crc_accumulate_buffer(&checksum, reinterpret_cast<const char *>(&packet[10]), payload_length);
		crc_accumulate(MAVLINK_MSG_ID_COMMAND_LONG_CRC, &checksum);
		packet[10 + payload_length] = static_cast<uint8_t>(checksum & 0xFF);
		packet[11 + payload_length] = static_cast<uint8_t>(checksum >> 8);

		return 10 + payload_length + 2;
	}

	template<typename T>
	static uint32_t counter_delta(T after, T before)
	{
		return static_cast<T>(after - before);
	}

	static void print_packet_bytes(const char *label, const uint8_t *packet, size_t packet_size)
	{
		if (packet_size == 0) {
			PX4_INFO("%s: n/a", label);
			return;
		}

		char line[3 * 16 + 1] {};

		for (size_t offset = 0; offset < packet_size; offset += 16) {
			const size_t chunk_size = math::min<size_t>(16, packet_size - offset);
			size_t cursor = 0;

			for (size_t i = 0; i < chunk_size; ++i) {
				cursor += snprintf(&line[cursor], sizeof(line) - cursor, "%02x%s",
						  packet[offset + i], (i + 1 < chunk_size) ? " " : "");
			}

			PX4_INFO("%s[%zu]: %s", label, offset / 16, line);
		}
	}

	void handle_mavlink_message(hrt_abstime now, const mavlink_message_t &message)
	{
		switch (message.msgid) {
		case MAVLINK_MSG_ID_COMMAND_ACK: {
				mavlink_command_ack_t ack {};
				mavlink_msg_command_ack_decode(&message, &ack);
				++_ack_count;
				_last_ack_command = ack.command;
				_last_ack_result = ack.result;
				_last_ack_valid = true;
				_last_ack_timestamp = now;

				if (ack.command == MAV_CMD_DO_PARACHUTE) {
					++_parachute_ack_count;
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

	Serial *_serial{nullptr};
	char _port[32] {};
	const uint32_t _baudrate;
	const bool _invert;

	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::PublicationMulti<debug_array_s> _debug_array_pub{ORB_ID(debug_array)};
	uORB::Publication<parachute_status_s> _parachute_status_pub{ORB_ID(parachute_status)};

	vehicle_command_s _last_command {};
	parachute_status_s _last_status {};
	uint8_t _tx_packet[kTxPacketMaxSize] {};
	uint8_t _rx_buffer[kRxBufferSize] {};
	mavlink_message_t _mavlink_rx_buffer {};
	mavlink_message_t _mavlink_message {};
	mavlink_status_t _mavlink_parse_status {};
	mavlink_status_t _mavlink_rx_status {};
	hrt_abstime _last_reply_timestamp{0};
	hrt_abstime _last_rx_timestamp{0};
	hrt_abstime _last_message_timestamp{0};
	hrt_abstime _last_ack_timestamp{0};
	hrt_abstime _stage_timestamp{0};
	hrt_abstime _last_run_start{0};
	hrt_abstime _last_run_complete{0};
	bool _command_valid{false};
	bool _connected{false};
	RunStage _run_stage{RunStage::Idle};
	uint32_t _tx_count{0};
	uint32_t _run_iteration{0};
	uint32_t _rx_read_count{0};
	uint32_t _rx_timeout_count{0};
	uint32_t _read_error_count{0};
	uint32_t _rx_byte_count{0};
	uint32_t _rx_message_count{0};
	uint32_t _ack_count{0};
	uint32_t _parachute_ack_count{0};
	uint32_t _parse_error_count{0};
	uint32_t _buffer_overrun_count{0};
	uint32_t _packet_drop_count{0};
	uint32_t _last_msgid{0};
	uint16_t _last_ack_command{0};
	uint8_t _last_ack_result{0};
	size_t _last_tx_size{0};
	int32_t _last_read_result{0};
	bool _last_msgid_valid{false};
	bool _last_ack_valid{false};
};

int parachute_rs485_main(int argc, char *argv[])
{
	return ParachuteRS485::main(argc, argv);
}
