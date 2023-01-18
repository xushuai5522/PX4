/****************************************************************************
 *
 *   Copyright (c) 2015 Estimation and Control Library (ECL). All rights reserved.
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
 * 3. Neither the name ECL nor the names of its contributors may be
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
 * @file ekf.cpp
 * Core functions for ekf attitude and position estimator.
 *
 * @author Roman Bast <bapstroman@gmail.com>
 * @author Paul Riseborough <p_riseborough@live.com.au>
 */

#include "ekf.h"

#include <mathlib/mathlib.h>

bool Ekf::init(uint64_t timestamp)
{
	bool ret = initialise_interface(timestamp);
	reset();
	return ret;
}

void Ekf::reset()
{
	_state.vel.setZero();
	_state.pos.setZero();
	_state.delta_ang_bias.setZero();
	_state.delta_vel_bias.setZero();
	_state.mag_I.setZero();
	_state.mag_B.setZero();
	_state.wind_vel.setZero();
	_state.quat_nominal.setIdentity();

	_range_sensor.setPitchOffset(_params.rng_sens_pitch);
	_range_sensor.setCosMaxTilt(_params.range_cos_max_tilt);
	_range_sensor.setQualityHysteresis(_params.range_valid_quality_s);

	_control_status.value = 0;
	_control_status_prev.value = 0;

	_control_status.flags.in_air = true;
	_control_status_prev.flags.in_air = true;

	_ang_rate_delayed_raw.zero();

	_fault_status.value = 0;
	_innov_check_fail_status.value = 0;

	_prev_dvel_bias_var.zero();

	resetGpsDriftCheckFilters();

	_output_predictor.reset();

	_filter_initialised = false;
}

bool Ekf::update()
{
	bool updated = false;

	if (!_filter_initialised) {
		if (initialiseFilter()) {
			_filter_initialised = true;

			_output_predictor.resetAlignment();
		}
	}

	// Only run the filter if IMU data in the buffer has been updated
	if (_imu_updated) {
		_imu_updated = false;

		// get the oldest IMU data from the buffer
		// TODO: explicitly pop at desired time horizon
		const imuSample imu_sample_delayed = _imu_buffer.get_oldest();

		if (_filter_initialised) {
			// perform state and covariance prediction for the main filter
			predictCovariance(imu_sample_delayed);
			predictState(imu_sample_delayed);

			// control fusion of observation data
			controlFusionModes(imu_sample_delayed);

			// run a separate filter for terrain estimation
			runTerrainEstimator(imu_sample_delayed);

			updated = true;
		}

		_output_predictor.correctOutputStates(imu_sample_delayed.time_us, getGyroBias(), getAccelBias(),
						      _state.quat_nominal, _state.vel, _state.pos);
	}

	return updated;
}

bool Ekf::initialiseFilter()
{
	// Filter accel for tilt initialization
	const imuSample &imu_init = _imu_buffer.get_newest();

	// protect against zero data
	if (imu_init.delta_vel_dt < 1e-4f || imu_init.delta_ang_dt < 1e-4f) {
		return false;
	}

	if (_is_first_imu_sample) {
		_accel_lpf.reset(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.reset(imu_init.delta_ang / imu_init.delta_ang_dt);
		_is_first_imu_sample = false;

	} else {
		_accel_lpf.update(imu_init.delta_vel / imu_init.delta_vel_dt);
		_gyro_lpf.update(imu_init.delta_ang / imu_init.delta_ang_dt);
	}

	// Sum the magnetometer measurements
	if (_mag_buffer) {
		magSample mag_sample;

		if (_mag_buffer->pop_first_older_than(_time_delayed_us, &mag_sample)) {
			if (mag_sample.time_us != 0) {
				if (_mag_counter == 0) {
					_mag_lpf.reset(mag_sample.mag);

				} else {
					_mag_lpf.update(mag_sample.mag);
				}

				_mag_counter++;
			}
		}
	}

	if (!initialiseTilt()) {
		return false;
	}

	// calculate the initial magnetic field and yaw alignment
	// but do not mark the yaw alignement complete as it needs to be
	// reset once the leveling phase is done
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		if (_mag_counter > 1) {
			// rotate the magnetometer measurements into earth frame using a zero yaw angle
			// the angle of the projection onto the horizontal gives the yaw angle
			const Vector3f mag_earth_pred = updateYawInRotMat(0.f, _R_to_earth) * _mag_lpf.getState();
			float yaw_new = -atan2f(mag_earth_pred(1), mag_earth_pred(0)) + getMagDeclination();

			// update the rotation matrix using the new yaw value
			_R_to_earth = updateYawInRotMat(yaw_new, Dcmf(_state.quat_nominal));
			_state.quat_nominal = _R_to_earth;

			// set the earth magnetic field states using the updated rotation
			_state.mag_I = _R_to_earth * _mag_lpf.getState();
			_state.mag_B.zero();

		} else {
			// not enough mag samples accumulated
			return false;
		}
	}

	// initialise the state covariance matrix now we have starting values for all the states
	initialiseCovariance();

	// update the yaw angle variance using the variance of the measurement
	if (_params.mag_fusion_type <= MagFuseType::MAG_3D) {
		// using magnetic heading tuning parameter
		increaseQuatYawErrVariance(sq(fmaxf(_params.mag_heading_noise, 1.0e-2f)));
	}

	// Initialise the terrain estimator
	initHagl();

	// reset the essential fusion timeout counters
	_time_last_hgt_fuse = _time_delayed_us;
	_time_last_hor_pos_fuse = _time_delayed_us;
	_time_last_hor_vel_fuse = _time_delayed_us;

	return true;
}

bool Ekf::initialiseTilt()
{
	const float accel_norm = _accel_lpf.getState().norm();
	const float gyro_norm = _gyro_lpf.getState().norm();

	if (accel_norm < 0.8f * CONSTANTS_ONE_G ||
	    accel_norm > 1.2f * CONSTANTS_ONE_G ||
	    gyro_norm > math::radians(15.0f)) {
		return false;
	}

	// get initial roll and pitch estimate from delta velocity vector, assuming vehicle is static
	const Vector3f gravity_in_body = _accel_lpf.getState().normalized();
	const float pitch = asinf(gravity_in_body(0));
	const float roll = atan2f(-gravity_in_body(1), -gravity_in_body(2));

	_state.quat_nominal = Quatf{Eulerf{roll, pitch, 0.0f}};
	_R_to_earth = Dcmf(_state.quat_nominal);

	return true;
}

void Ekf::predictState(const imuSample &imu_delayed)
{
	// apply imu bias corrections
	const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
	Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;

	// subtract component of angular rate due to earth rotation
	corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

	const Quatf dq(AxisAnglef{corrected_delta_ang});

	// rotate the previous quaternion by the delta quaternion using a quaternion multiplication
	_state.quat_nominal = (_state.quat_nominal * dq).normalized();
	_R_to_earth = Dcmf(_state.quat_nominal);

	// Calculate an earth frame delta velocity
	const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
	const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
	const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;

	// save the previous value of velocity so we can use trapzoidal integration
	const Vector3f vel_last = _state.vel;

	// calculate the increment in velocity using the current orientation
	_state.vel += corrected_delta_vel_ef;

	// compensate for acceleration due to gravity
	_state.vel(2) += CONSTANTS_ONE_G * imu_delayed.delta_vel_dt;

	// predict position states via trapezoidal integration of velocity
	_state.pos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;

	constrainStates();

	// calculate an average filter update time
	float input = 0.5f * (imu_delayed.delta_vel_dt + imu_delayed.delta_ang_dt);

	// filter and limit input between -50% and +100% of nominal value
	const float filter_update_s = 1e-6f * _params.filter_update_interval_us;
	input = math::constrain(input, 0.5f * filter_update_s, 2.f * filter_update_s);
	_dt_ekf_avg = 0.99f * _dt_ekf_avg + 0.01f * input;

	// some calculations elsewhere in code require a raw angular rate vector so calculate here to avoid duplication
	// protect against possible small timesteps resulting from timing slip on previous frame that can drive spikes into the rate
	// due to insufficient averaging
	if (imu_delayed.delta_ang_dt > 0.25f * _dt_ekf_avg) {
		_ang_rate_delayed_raw = imu_delayed.delta_ang / imu_delayed.delta_ang_dt;
	}


	// calculate a filtered horizontal acceleration with a 1 sec time constant
	// this are used for manoeuvre detection elsewhere
	const float alpha = 1.0f - imu_delayed.delta_vel_dt;
	_accel_lpf_NE = _accel_lpf_NE * alpha + corrected_delta_vel_ef.xy();

	// calculate a yaw change about the earth frame vertical
	const float spin_del_ang_D = corrected_delta_ang.dot(Vector3f(_R_to_earth.row(2)));
	_yaw_delta_ef += spin_del_ang_D;

	// Calculate filtered yaw rate to be used by the magnetometer fusion type selection logic
	// Note fixed coefficients are used to save operations. The exact time constant is not important.
	_yaw_rate_lpf_ef = 0.95f * _yaw_rate_lpf_ef + 0.05f * spin_del_ang_D / imu_delayed.delta_ang_dt;
}
