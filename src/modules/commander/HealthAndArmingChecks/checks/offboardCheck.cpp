/****************************************************************************
 *
 *   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

#include "offboardCheck.hpp"

using namespace time_literals;

void OffboardChecks::checkAndReport(const Context &context, Report &reporter)
{
	reporter.failsafeFlags().offboard_control_signal_lost = true;

	offboard_control_mode_s offboard_control_mode;

	if (_offboard_control_mode_sub.copy(&offboard_control_mode)) {
		estimator_status_flags_s estimator_status_flags{};
		bool estimator_status_flags_valid = false;

		if (_param_sens_imu_mode.get() == 0) { // multi-ekf
			estimator_selector_status_s estimator_selector_status;

			if (_estimator_selector_status_sub.copy(&estimator_selector_status)) {
				if (_estimator_status_flags_sub.ChangeInstance(estimator_selector_status.primary_instance)) {
					estimator_status_flags_valid = _estimator_status_flags_sub.copy(&estimator_status_flags);
				}
			}

		} else {
			estimator_status_flags_valid = _estimator_status_flags_sub.copy(&estimator_status_flags);
		}

		bool data_is_recent = hrt_absolute_time() < offboard_control_mode.timestamp
				      + static_cast<hrt_abstime>(_param_com_of_loss_t.get() * 1_s);

		bool offboard_available = (offboard_control_mode.position || offboard_control_mode.velocity
					   || offboard_control_mode.acceleration || offboard_control_mode.attitude || offboard_control_mode.body_rate
					   || offboard_control_mode.thrust_and_torque || offboard_control_mode.direct_actuator) && data_is_recent;

		if (offboard_control_mode.position && reporter.failsafeFlags().local_position_invalid) {
			offboard_available = false;

		} else if (offboard_control_mode.velocity && reporter.failsafeFlags().local_velocity_invalid) {
			offboard_available = false;

		} else if (offboard_control_mode.acceleration && reporter.failsafeFlags().local_velocity_invalid) {
			// OFFBOARD acceleration handled by position controller
			offboard_available = false;
		}

		const bool require_gps_fusion_for_offboard_takeoff = !context.isArmed()
				&& offboard_control_mode.position
				&& _param_sys_has_gps.get()
				&& (_param_ekf2_hgt_ref.get() == EKF2_HGT_REF_GNSS);

		if (require_gps_fusion_for_offboard_takeoff) {
			const bool gps_fused = estimator_status_flags_valid && estimator_status_flags.cs_gps;
			const bool gps_hgt_fused = estimator_status_flags_valid && estimator_status_flags.cs_gps_hgt;

			if (!gps_fused || !gps_hgt_fused) {
				offboard_available = false;

				/* EVENT
				 * @description
				 * GNSS horizontal and height fusion must be active before an offboard position takeoff can start.
				 */
				reporter.armingCheckFailure(
					(NavModes)reporter.failsafeFlags().mode_req_offboard_signal,
					health_component_t::local_position_estimate,
					events::ID("check_offboard_gps_fusion_required"),
					events::Log::Error,
					"Offboard takeoff requires GNSS fusion");
			}
		}

		// This is a mode requirement, no need to report
		reporter.failsafeFlags().offboard_control_signal_lost = !offboard_available;
	}
}
