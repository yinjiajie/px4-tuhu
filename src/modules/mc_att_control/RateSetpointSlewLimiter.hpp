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

#pragma once

#include <lib/slew_rate/SlewRate.hpp>
#include <matrix/matrix/math.hpp>

class RateSetpointSlewLimiter
{
public:
	void setSlewRate(const matrix::Vector3f &slew_rate)
	{
		for (int axis = 0; axis < 3; axis++) {
			_slew_rate[axis].setSlewRate(slew_rate(axis));
		}
	}

	void reset(const matrix::Vector3f &rate_setpoint)
	{
		for (int axis = 0; axis < 3; axis++) {
			_slew_rate[axis].setForcedValue(rate_setpoint(axis));
		}
	}

	matrix::Vector3f update(const matrix::Vector3f &rate_setpoint, float dt)
	{
		matrix::Vector3f limited_rate_setpoint;

		for (int axis = 0; axis < 3; axis++) {
			limited_rate_setpoint(axis) = _slew_rate[axis].update(rate_setpoint(axis), dt);
		}

		return limited_rate_setpoint;
	}

private:
	SlewRate<float> _slew_rate[3];
};
