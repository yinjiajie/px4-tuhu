/****************************************************************************
 *
 *   Copyright (c) 2019-2023 PX4 Development Team. All rights reserved.
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

/**
 * Test the fusion start and stop logic
 * @author Kamil Ritz <ka.ritz@hotmail.com>
 */

#include <gtest/gtest.h>
#include "EKF/ekf.h"
#include "sensor_simulator/sensor_simulator.h"
#include "sensor_simulator/ekf_wrapper.h"
#include "test_helper/reset_logging_checker.h"


class EkfFusionLogicTest : public ::testing::Test
{
public:

	EkfFusionLogicTest(): ::testing::Test(),
		_ekf{std::make_shared<Ekf>()},
		_sensor_simulator(_ekf),
		_ekf_wrapper(_ekf) {};

	std::shared_ptr<Ekf> _ekf;
	SensorSimulator _sensor_simulator;
	EkfWrapper _ekf_wrapper;

	// Setup the Ekf with synthetic measurements
	void SetUp() override
	{
		// run briefly to init, then manually set in air and at rest (default for a real vehicle)
		_ekf->init(0);
		_sensor_simulator.runSeconds(0.1);
		_ekf->set_in_air_status(false);
		_ekf->set_vehicle_at_rest(true);

		_sensor_simulator.runSeconds(7);

		// When at rest, the strong zero velocity update keeps the local position
		// valid. Deactivate this to test the change of validity without zvup
		_ekf->set_vehicle_at_rest(false);
	}

	// Use this method to clean up any memory, network etc. after each test
	void TearDown() override
	{
	}
};


TEST_F(EkfFusionLogicTest, doNoFusion)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: having no aiding source
	// THEN: EKF should not have a valid position estimate
	EXPECT_FALSE(_ekf->local_position_is_valid());

	_sensor_simulator.runSeconds(4);

	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doGpsFusion)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: we enable GPS fusion and we send good quality gps data for 11s
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	// THEN: Local and global position should be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());

	// WHEN: GPS data is not send for 11s
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should stop to intend to fuse GPS
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: GPS data is send again for 11s
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should to intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_TRUE(_ekf->global_position_is_valid());

	// WHEN: clients decides to stop GPS fusion
	_ekf_wrapper.disableGpsFusion();

	// THEN: EKF should stop to intend to fuse GPS immediately
	_sensor_simulator.runSeconds(0.01);
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfFusionLogicTest, rejectGpsSignalJump)
{
	// GIVEN: a tilt and heading aligned filter
	// WHEN: we enable GPS fusion and we send good quality gps data for 11s
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(15);

	// THEN: EKF should intend to fuse GPS
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());

	// WHEN: Having a big horizontal position Gps jump coming from the Gps Receiver
	const Vector3f pos_old = _ekf->getPosition();
	const Vector3f vel_old = _ekf->getVelocity();
	const Vector3f accel_bias_old = _ekf->getAccelBias();
	const Vector3f pos_step{20.0f, 0.0f, 0.f};
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f(pos_step));
	_sensor_simulator.runSeconds(2);

	// THEN: The estimate should not change much in the short run
	Vector3f pos_new = _ekf->getPosition();
	Vector3f vel_new = _ekf->getVelocity();
	Vector3f accel_bias_new = _ekf->getAccelBias();
	EXPECT_TRUE(matrix::isEqual(pos_new, pos_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(vel_new, vel_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(accel_bias_new, accel_bias_old, 0.01f));

	// BUT THEN: GPS fusion should reset after a while
	// (it takes some time because vel fusion is still good)
	_sensor_simulator.runSeconds(14);
	pos_new = _ekf->getPosition();
	vel_new = _ekf->getVelocity();
	accel_bias_new = _ekf->getAccelBias();
	EXPECT_TRUE(matrix::isEqual(pos_new, pos_old + pos_step, 0.01f));
	EXPECT_TRUE(matrix::isEqual(vel_new, vel_old, 0.01f));
	EXPECT_TRUE(matrix::isEqual(accel_bias_new, accel_bias_old, 0.01f));
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
}

TEST_F(EkfFusionLogicTest, fallbackFromGpsToFlow)
{
	// GIVEN: GPS and flow setup up and with valid data
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();

	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.enableFlowFusion();

	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(15);

	// THEN: both should be fused
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	// WHEN: GPS data stops
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(10);

	// THEN: GNSS fusion stops after a timing out
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	// BUT WHEN: GPS starts after passing the checks again
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(6);

	// THEN: use it again
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
}

TEST_F(EkfFusionLogicTest, gpsToFlowTransitionKeepsHorizontalStateContinuity)
{
	ResetLoggingChecker reset_logging_checker(_ekf);

	// GIVEN: GPS and flow setup with both aiding sources active
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();

	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.enableFlowFusion();

	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(15);

	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	reset_logging_checker.capturePreResetState();

	// WHEN: GPS stops and optical flow becomes the only horizontal aiding source
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(11);

	// THEN: flow remains active and the local position frame and velocity remain continuous
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isHorizontalVelocityResetCounterIncreasedBy(0));
	EXPECT_TRUE(reset_logging_checker.isHorizontalPositionResetCounterIncreasedBy(0));
}

TEST_F(EkfFusionLogicTest, flowToGpsStartWhenArmedKeepsReferenceAndAvoidsHorizontalPositionReset)
{
	// GIVEN: optical flow aiding while armed in a local frame that already has a global origin
	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.enableFlowFusion();
	_ekf->set_vehicle_armed(true);
	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(5);

	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	const gnssSample gps_data = _sensor_simulator._gps.getDefaultGpsData();
	ASSERT_TRUE(_ekf->setEkfGlobalOrigin(gps_data.lat, gps_data.lon, gps_data.alt));
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f{3.f, 3.f});

	uint64_t origin_time_before = 0;
	double origin_lat_before = 0.0;
	double origin_lon_before = 0.0;
	float origin_alt_before = 0.f;
	ASSERT_TRUE(_ekf->getEkfGlobalOrigin(origin_time_before, origin_lat_before, origin_lon_before, origin_alt_before));

	float delta_xy[2] {};
	uint8_t pos_reset_counter_before = 0;
	_ekf->get_posNE_reset(delta_xy, &pos_reset_counter_before);
	const Vector3f pos_before = _ekf->getPosition();
	const Vector2f pos_before_xy{pos_before(0), pos_before(1)};
	const Vector2f gps_hpos_target{3.f, 3.f};

	_ekf->set_min_required_gps_health_time(200000);
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(1.5f);

	// THEN: GPS starts without changing the reference or hard resetting local position
	uint8_t pos_reset_counter_after = 0;
	_ekf->get_posNE_reset(delta_xy, &pos_reset_counter_after);
	const Vector3f pos_after = _ekf->getPosition();
	const Vector2f pos_after_xy{pos_after(0), pos_after(1)};

	uint64_t origin_time_after = 0;
	double origin_lat_after = 0.0;
	double origin_lon_after = 0.0;
	float origin_alt_after = 0.f;
	ASSERT_TRUE(_ekf->getEkfGlobalOrigin(origin_time_after, origin_lat_after, origin_lon_after, origin_alt_after));

	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_EQ(pos_reset_counter_before, pos_reset_counter_after);
	EXPECT_EQ(origin_time_before, origin_time_after);
	EXPECT_DOUBLE_EQ(origin_lat_before, origin_lat_after);
	EXPECT_DOUBLE_EQ(origin_lon_before, origin_lon_after);
	EXPECT_FLOAT_EQ(origin_alt_before, origin_alt_after);
	EXPECT_GT((pos_after_xy - pos_before_xy).norm(), 0.01f);
	EXPECT_LT((pos_after_xy - gps_hpos_target).norm(), (pos_before_xy - gps_hpos_target).norm());
	EXPECT_GT((pos_after_xy - gps_hpos_target).norm(), 0.1f);
}

TEST_F(EkfFusionLogicTest, flowToGpsStartWhenDisarmedOnGroundAllowsHorizontalPositionReset)
{
	// GIVEN: optical flow aiding on the ground while disarmed in a local frame that already has a global origin
	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator._flow.setData(_sensor_simulator._flow.dataAtRest());
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_ekf_wrapper.enableFlowFusion();
	_ekf->set_vehicle_armed(false);
	_sensor_simulator.runSeconds(5);

	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());

	const gnssSample gps_data = _sensor_simulator._gps.getDefaultGpsData();
	ASSERT_TRUE(_ekf->setEkfGlobalOrigin(gps_data.lat, gps_data.lon, gps_data.alt));
	_sensor_simulator._gps.stepHorizontalPositionByMeters(Vector2f{3.f, 3.f});

	float delta_xy[2] {};
	uint8_t pos_reset_counter_before = 0;
	_ekf->get_posNE_reset(delta_xy, &pos_reset_counter_before);

	_ekf->set_min_required_gps_health_time(200000);
	_ekf_wrapper.enableGpsFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(1.5f);

	// THEN: on the ground, GPS start is allowed to hard reset local position
	uint8_t pos_reset_counter_after = 0;
	_ekf->get_posNE_reset(delta_xy, &pos_reset_counter_after);
	const Vector3f pos_after = _ekf->getPosition();

	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsFusion());
	EXPECT_GT(pos_reset_counter_after, pos_reset_counter_before);
	EXPECT_NEAR(pos_after(0), 3.f, 0.2f);
	EXPECT_NEAR(pos_after(1), 3.f, 0.2f);
}

TEST_F(EkfFusionLogicTest, doFlowFusion)
{
	// GIVEN: a tilt and heading aligned filter
	EXPECT_TRUE(_ekf->attitude_valid());

	// WHEN: sending flow data without having the flow fusion enabled
	//       flow measurement fusion should not be intended.
	const float max_flow_rate = 5.f;
	const float min_ground_distance = 0.f;
	const float max_ground_distance = 50.f;
	_ekf->set_optical_flow_limits(max_flow_rate, min_ground_distance, max_ground_distance);
	_sensor_simulator.startFlow();
	_sensor_simulator.startRangeFinder();
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should not intend to fuse flow measurements
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is not send and we enable flow fusion
	_sensor_simulator.stopFlow();
	_sensor_simulator.runSeconds(1); // empty buffer
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(3);

	// THEN: EKF should not intend to fuse flow
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Flow data is sent and we enable flow fusion
	_sensor_simulator.startFlow();
	_ekf_wrapper.enableFlowFusion();
	_sensor_simulator.runSeconds(10);

	// THEN: EKF should intend to fuse flow
	EXPECT_TRUE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should be valid
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: Stop sending flow data
	_sensor_simulator.stopFlow();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should not intend to fuse flow measurements
	EXPECT_FALSE(_ekf_wrapper.isIntendingFlowFusion());
	// THEN: Local and global position should not be valid
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionPositionFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	_ekf_wrapper.enableExternalVisionPositionFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision position estimate
	//       and we have a valid local position estimate
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionVelocityFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	_ekf_wrapper.enableExternalVisionVelocityFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision position estimate
	//       and we have a valid local position estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_TRUE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
}

TEST_F(EkfFusionLogicTest, doVisionHeadingFusion)
{
	// WHEN: allow vision position to be fused and we send vision data
	const int initial_quat_reset_counter = _ekf_wrapper.getQuaternionResetCounter();
	_ekf_wrapper.enableExternalVisionHeadingFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(4);

	// THEN: EKF should intend to fuse vision heading estimates
	//       and we should not have a valid local position estimate
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
	// THEN: Yaw state should be reset to vision
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 1);

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(7.1);

	// THEN: EKF should stop to intend to fuse vision position estimate
	//       and EKF should not have a valid local position estimate anymore
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionPositionFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionVelocityFusion());
	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeadingFusion());
	EXPECT_FALSE(_ekf->local_position_is_valid());
	EXPECT_FALSE(_ekf->global_position_is_valid());
	// THEN: Yaw state shoud be reset to mag
	EXPECT_TRUE(_ekf_wrapper.isIntendingMagHeadingFusion());
	EXPECT_EQ(_ekf_wrapper.getQuaternionResetCounter(), initial_quat_reset_counter + 2);
}

TEST_F(EkfFusionLogicTest, doBaroHeightFusion)
{
	// GIVEN: EKF that receives baro and GPS data
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);
	_ekf_wrapper.enableGpsHeightFusion();

	// THEN: EKF should intend to fuse baro by default
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: stop sending baro data
	_sensor_simulator.stopBaro();
	_sensor_simulator.runSeconds(6);

	// THEN: EKF should stop to intend to use baro hgt and use GPS as a fallback
	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
}

TEST_F(EkfFusionLogicTest, doBaroHeightFusionTimeout)
{
	// GIVEN: EKF that receives baro data

	// THEN: EKF should intend to fuse baro by default
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());

	// WHEN: the baro data jumps by a lot
	ResetLoggingChecker reset_logging_checker(_ekf);
	reset_logging_checker.capturePreResetState();

	_sensor_simulator._baro.setData(100.f);
	_ekf->set_vehicle_at_rest(false);
	_ekf->set_in_air_status(true);
	_sensor_simulator.runSeconds(6);

	reset_logging_checker.capturePostResetState();

	// THEN: EKF should reset to the measurement
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(1));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));

	// BUT WHEN: GPS height data is also available
	_sensor_simulator.startGps();
	_ekf_wrapper.enableGpsHeightFusion();
	_sensor_simulator.runSeconds(11);
	reset_logging_checker.capturePostResetState();
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(2));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));

	// AND: the baro data jumps by a lot
	_sensor_simulator._baro.setData(800.f);
	_sensor_simulator.runSeconds(20);
	reset_logging_checker.capturePostResetState();

	// THEN: EKF should fallback to GPS height (without reset)
	EXPECT_FALSE(_ekf_wrapper.isIntendingBaroHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(reset_logging_checker.isVerticalVelocityResetCounterIncreasedBy(2));
	EXPECT_TRUE(reset_logging_checker.isVerticalPositionResetCounterIncreasedBy(1));
}

TEST_F(EkfFusionLogicTest, doGpsHeightFusion)
{
	// WHEN: commanding GPS height and sending GPS data
	_ekf_wrapper.enableGpsHeightFusion();
	_sensor_simulator.startGps();
	_sensor_simulator.runSeconds(11);

	// THEN: EKF should intend to fuse gps height
	EXPECT_TRUE(_ekf_wrapper.isIntendingGpsHeightFusion());

	// WHEN: stop sending gps data
	_sensor_simulator.stopGps();
	_sensor_simulator.runSeconds(5);

	// THEN: EKF should stop to intend to use gps height
	EXPECT_FALSE(_ekf_wrapper.isIntendingGpsHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfFusionLogicTest, doRangeHeightFusion)
{
	// WHEN: commanding range height and sending range data
	_ekf_wrapper.enableRangeHeightFusion();
	_sensor_simulator.startRangeFinder();
	_sensor_simulator.runSeconds(2.5f);
	// THEN: EKF should intend to fuse range height
	EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());

	const float dt = 5e-3f;

	for (int i = 0; i < 10; i++) {
		_sensor_simulator.runSeconds(dt);
		// THEN: EKF should intend to fuse range height, even if
		// there is no new data at each EKF iteration (EKF rate > sensor rate)
		EXPECT_TRUE(_ekf_wrapper.isIntendingRangeHeightFusion());
	}

	// WHEN: stop sending range data
	_sensor_simulator.stopRangeFinder();
	_sensor_simulator.runSeconds(5.1);

	// THEN: EKF should stop to intend to use range height
	// and fall back to baro height
	EXPECT_FALSE(_ekf_wrapper.isIntendingRangeHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}

TEST_F(EkfFusionLogicTest, doVisionHeightFusion)
{
	// WHEN: commanding vision height and sending vision data
	_ekf_wrapper.enableExternalVisionHeightFusion();
	_sensor_simulator.startExternalVision();
	_sensor_simulator.runSeconds(2);

	// THEN: EKF should intend to fuse vision height
	EXPECT_TRUE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());

	// WHEN: stop sending vision data
	_sensor_simulator.stopExternalVision();
	_sensor_simulator.runSeconds(12);

	// THEN: EKF should stop to intend to use vision height

	EXPECT_FALSE(_ekf_wrapper.isIntendingExternalVisionHeightFusion());
	EXPECT_TRUE(_ekf_wrapper.isIntendingBaroHeightFusion());
}
