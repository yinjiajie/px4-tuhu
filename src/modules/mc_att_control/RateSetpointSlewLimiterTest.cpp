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

#include "RateSetpointSlewLimiter.hpp"

#include <gtest/gtest.h>
#include <mathlib/math/Functions.hpp>

using matrix::Vector3f;
using math::radians;

TEST(RateSetpointSlewLimiter, LimitsAxesIndependently)
{
	RateSetpointSlewLimiter limiter;
	limiter.setSlewRate(Vector3f(radians(600.f), radians(600.f), radians(400.f)));
	limiter.reset(Vector3f(0.f, 0.f, 0.f));

	const Vector3f limited = limiter.update(Vector3f(2.f, -2.f, 2.f), 0.1f);

	EXPECT_NEAR(limited(0), radians(600.f) * 0.1f, 1e-6f);
	EXPECT_NEAR(limited(1), -radians(600.f) * 0.1f, 1e-6f);
	EXPECT_NEAR(limited(2), radians(400.f) * 0.1f, 1e-6f);
}

TEST(RateSetpointSlewLimiter, ResetSeedsCurrentState)
{
	RateSetpointSlewLimiter limiter;
	limiter.setSlewRate(Vector3f(radians(600.f), radians(600.f), radians(400.f)));
	limiter.reset(Vector3f(0.3f, -0.2f, 0.1f));

	const Vector3f limited = limiter.update(Vector3f(2.f, -2.f, 2.f), 0.1f);

	EXPECT_NEAR(limited(0), 0.3f + radians(600.f) * 0.1f, 1e-6f);
	EXPECT_NEAR(limited(1), -0.2f - radians(600.f) * 0.1f, 1e-6f);
	EXPECT_NEAR(limited(2), 0.1f + radians(400.f) * 0.1f, 1e-6f);
}
