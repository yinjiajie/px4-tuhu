/****************************************************************************
 *
 *   Copyright (c) 2020-2023 PX4 Development Team. All rights reserved.
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

#include "HealthAndArmingChecks.hpp"

#include <cstring>

namespace
{

template<typename T>
bool readEventArgument(const uint8_t *arguments, unsigned size, unsigned &offset, T &value)
{
	if (offset + sizeof(T) > size) {
		return false;
	}

	memcpy(&value, arguments + offset, sizeof(T));
	offset += sizeof(T);
	return true;
}

constexpr uint32_t autopilotEventId(const char *name)
{
	return (0xffffff & events::util::hash_32_fnv1a_const(name)) | (1u << 24);
}

const char *parameterHintForEvent(uint32_t event_id)
{
	switch (event_id) {
	case autopilotEventId("check_estimator_hgt_est_err"):
		return "param: COM_ARM_EKF_HGT";

	case autopilotEventId("check_estimator_vel_est_err"):
		return "param: COM_ARM_EKF_VEL";

	case autopilotEventId("check_estimator_pos_est_err"):
		return "param: COM_ARM_EKF_POS";

	case autopilotEventId("check_estimator_yaw_est_err"):
		return "param: COM_ARM_EKF_YAW";

	case autopilotEventId("check_estimator_gps_fix_too_low"):
	case autopilotEventId("check_estimator_gps_num_sats_too_low"):
	case autopilotEventId("check_estimator_gps_pdop_too_high"):
	case autopilotEventId("check_estimator_gps_hor_pos_err_too_high"):
	case autopilotEventId("check_estimator_gps_vert_pos_err_too_high"):
	case autopilotEventId("check_estimator_gps_speed_acc_too_low"):
	case autopilotEventId("check_estimator_gps_hor_pos_drift_too_high"):
	case autopilotEventId("check_estimator_gps_vert_pos_drift_too_high"):
	case autopilotEventId("check_estimator_gps_hor_speed_drift_too_high"):
	case autopilotEventId("check_estimator_gps_vert_speed_drift_too_high"):
	case autopilotEventId("check_estimator_gps_not_fusing"):
	case autopilotEventId("check_estimator_gps_generic"):
		return "param: EKF2_GPS_CHECK";

	case autopilotEventId("check_estimator_mag_interference"):
		return "params: COM_ARM_MAG_STR, EKF2_MAG_CHECK";

	case autopilotEventId("check_mag_consistency"):
		return "param: COM_ARM_MAG_ANG";

	case autopilotEventId("check_imu_accel_inconsistent"):
		return "param: COM_ARM_IMU_ACC";

	case autopilotEventId("check_imu_gyro_inconsistent"):
		return "param: COM_ARM_IMU_GYR";

	case autopilotEventId("check_modes_manual_control"):
		return "param: COM_RC_IN_MODE";

	case autopilotEventId("check_man_control_kill_engaged"):
		return "action: release RC kill switch; params: RC_MAP_KILL_SW, RC_KILLSWITCH_TH";

	case autopilotEventId("check_modes_mission"):
		return "param: COM_ARM_MIS_REQ";

	case autopilotEventId("check_system_no_global_pos"):
	case autopilotEventId("check_system_no_home_pos"):
		return "param: COM_ARM_WO_GPS";

	case autopilotEventId("check_system_usb_connected"):
		return "param: CBRK_USB_CHK";

	case autopilotEventId("check_system_safety_button"):
		return "param: CBRK_IO_SAFETY";

	case autopilotEventId("check_system_flight_term_active"):
		return "action: clear kill/termination state";

	case autopilotEventId("check_system_avoidance_not_ready"):
		return "param: COM_OBS_AVOID";

	case autopilotEventId("check_system_vtol_in_fw_mode"):
		return "param: CBRK_VTOLARMING";

	case autopilotEventId("check_battery_preflight_low"):
		return "param: COM_ARM_BAT_MIN";

	case autopilotEventId("check_estimator_high_accel_bias"):
		return "param: EKF2_ABL_LIM";

	case autopilotEventId("check_estimator_high_gyro_bias"):
		return "param: EKF2_ABL_GYRLIM";

	case autopilotEventId("check_parachute_missing"):
	case autopilotEventId("check_parachute_unhealthy"):
		return "param: COM_PARACHUTE";

	case autopilotEventId("check_missing_fmu_sdcard"):
		return "param: COM_ARM_SDCARD";

	case autopilotEventId("check_hardfault_present"):
		return "param: COM_ARM_HFLT_CHK";

	case autopilotEventId("check_open_drone_id_missing"):
	case autopilotEventId("check_open_drone_id_unhealthy"):
		return "param: COM_ARM_ODID";

	default:
		return nullptr;
	}
}

void printKnownEventDetails(uint32_t event_id, const uint8_t *arguments, unsigned size)
{
	unsigned offset = sizeof(uint32_t) + sizeof(uint8_t);

	if (event_id == autopilotEventId("check_estimator_hgt_est_err")
	    || event_id == autopilotEventId("check_estimator_vel_est_err")
	    || event_id == autopilotEventId("check_estimator_pos_est_err")
	    || event_id == autopilotEventId("check_estimator_yaw_est_err")) {
		float current{};
		float limit{};

		if (readEventArgument(arguments, size, offset, current) && readEventArgument(arguments, size, offset, limit)) {
			PX4_INFO_RAW(" (current=%.3f limit=%.3f)", (double)current, (double)limit);
		}

		return;
	}

	if (event_id == autopilotEventId("check_imu_accel_inconsistent")
	    || event_id == autopilotEventId("check_imu_gyro_inconsistent")) {
		uint8_t instance{};
		float current{};
		float limit{};

		if (readEventArgument(arguments, size, offset, instance)
		    && readEventArgument(arguments, size, offset, current)
		    && readEventArgument(arguments, size, offset, limit)) {
			PX4_INFO_RAW(" (sensor=%u current=%.3f limit=%.3f)", instance, (double)current, (double)limit);
		}

		return;
	}

	if (event_id == autopilotEventId("check_accel_not_calibrated")
	    || event_id == autopilotEventId("check_mag_not_calibrated")
	    || event_id == autopilotEventId("check_mag_fault")) {
		uint8_t instance{};

		if (readEventArgument(arguments, size, offset, instance)) {
			PX4_INFO_RAW(" (sensor=%u)", instance);
		}
	}
}

} // namespace

HealthAndArmingChecks::HealthAndArmingChecks(ModuleParams *parent, vehicle_status_s &status)
	: ModuleParams(parent),
	  _context(status)
{
	// Initialize mode requirements to invalid
	_failsafe_flags.angular_velocity_invalid = true;
	_failsafe_flags.attitude_invalid = true;
	_failsafe_flags.local_altitude_invalid = true;
	_failsafe_flags.local_position_invalid = true;
	_failsafe_flags.local_position_invalid_relaxed = true;
	_failsafe_flags.local_velocity_invalid = true;
	_failsafe_flags.global_position_invalid = true;
	_failsafe_flags.auto_mission_missing = true;
	_failsafe_flags.offboard_control_signal_lost = true;
	_failsafe_flags.home_position_invalid = true;
}

bool HealthAndArmingChecks::update(bool force_reporting)
{
	_reporter.reset();

	_reporter.prepare(_context.status().vehicle_type);

	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->checkAndReport(_context, _reporter);
	}

	const bool results_changed = _reporter.finalize();
	const bool reported = _reporter.report(_context.isArmed(), force_reporting);

	if (reported) {

		// LEGACY start
		// Run the checks again, this time with the mavlink publication set.
		// We don't expect any change, and rate limitation would prevent the events from being reported again,
		// so we only report mavlink_log_*.
		_reporter._mavlink_log_pub = &_mavlink_log_pub;
		_reporter.reset();

		_reporter.prepare(_context.status().vehicle_type);

		for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
			if (!_checks[i]) {
				break;
			}

			_checks[i]->checkAndReport(_context, _reporter);
		}

		_reporter.finalize();
		_reporter.report(_context.isArmed(), false);
		_reporter._mavlink_log_pub = nullptr;
		// LEGACY end

		health_report_s health_report;
		_reporter.getHealthReport(health_report);
		health_report.timestamp = hrt_absolute_time();
		_health_report_pub.publish(health_report);
	}

	// Check if we need to publish the failsafe flags
	const hrt_abstime now = hrt_absolute_time();

	if ((now > _failsafe_flags.timestamp + 500_ms) || results_changed) {
		_failsafe_flags.timestamp = hrt_absolute_time();
		_failsafe_flags_pub.publish(_failsafe_flags);
	}

	return reported;
}

void HealthAndArmingChecks::printArmingBlockersToConsole() const
{
	const Report::Results &current_results = _reporter._results[_reporter._current_result];
	const uint32_t current_mode_group = (uint32_t)_reporter.getModeGroup(_context.status().nav_state);

	if (current_results.num_events == 0) {
		PX4_INFO_RAW("No arming blockers found.\n");
		return;
	}

	int offset = 0;
	int blocker_count = 0;

	for (int event_index = 0;
	     event_index < current_results.num_events && offset < _reporter._next_buffer_idx;
	     ++event_index) {
		const auto *header = reinterpret_cast<const Report::EventBufferHeader *>(_reporter._event_buffer + offset);
		const uint8_t *arguments = _reporter._event_buffer + offset + sizeof(Report::EventBufferHeader);
		uint32_t blocking_modes{};
		const char *message = nullptr;

		memcpy(&blocking_modes, &header->blocking_modes, sizeof(blocking_modes));
		offset += sizeof(Report::EventBufferHeader) + header->size;

		if ((blocking_modes & current_mode_group) == 0) {
			continue;
		}

		memcpy(&message, &header->message, sizeof(message));

		if (blocker_count == 0) {
			PX4_INFO_RAW("Arming blockers:\n");
		}

		PX4_INFO_RAW("  %d. %s", blocker_count + 1, message ? message : "Unknown arming check failure");

		if (const char *hint = parameterHintForEvent(header->id)) {
			PX4_INFO_RAW(" [%s]", hint);
		}

		printKnownEventDetails(header->id, arguments, header->size);
		PX4_INFO_RAW("\n");
		++blocker_count;
	}

	if (blocker_count == 0) {
		PX4_INFO_RAW("No arming blockers found.\n");
	}
}

void HealthAndArmingChecks::updateParams()
{
	for (unsigned i = 0; i < sizeof(_checks) / sizeof(_checks[0]); ++i) {
		if (!_checks[i]) {
			break;
		}

		_checks[i]->updateParams();
	}
}
