/****************************************************************************
 *
 *   Copyright (c) 2013 Estimation and Control Library (ECL). All rights reserved.
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
 * @file ecl_controller.cpp
 * Definition of base class for other controllers
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 *
 * Acknowledgements:
 *
 *   The control design is based on a design
 *   by Paul Riseborough and Andrew Tridgell, 2013,
 *   which in turn is based on initial work of
 *   Jonathan Challinger, 2012.
 */

#include "ecl_controller.h"

#include <stdio.h>
#include <mathlib/mathlib.h>

ECL_Controller::ECL_Controller() :
	_last_run(0),
	_tc(0.1f),
	_k_p(0.0f),
	_k_i(0.0f),
	_k_ff(0.0f),
	_integrator_max(0.0f),
	_max_rate(0.0f),
	_last_output(0.0f),
	_integrator(0.0f),
	_euler_rate_setpoint(0.0f),
	_body_rate_setpoint(0.0f)
{
}

void ECL_Controller::reset_integrator()
{
	_integrator = 0.0f;
}

void ECL_Controller::set_time_constant(float time_constant)
{
	if (time_constant > 0.1f && time_constant < 3.0f) {
		_tc = time_constant;
	}
}

void ECL_Controller::set_k_p(float k_p)
{
	_k_p = k_p;
}

void ECL_Controller::set_k_i(float k_i)
{
	_k_i = k_i;
}

void ECL_Controller::set_k_ff(float k_ff)
{
	_k_ff = k_ff;
}

void ECL_Controller::set_integrator_max(float max)
{
	_integrator_max = max;
}

void ECL_Controller::set_max_rate(float max_rate)
{
	_max_rate = max_rate;
}

float ECL_Controller::get_euler_rate_setpoint()
{
	return _euler_rate_setpoint;
}

float ECL_Controller::get_body_rate_setpoint()
{
	return _body_rate_setpoint;
}

float ECL_Controller::get_integrator()
{
	return _integrator;
}
