/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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
 * @file rtl.cpp
 *
 * Helper class to access RTL
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Julian Kent <julian@auterion.com>
 */

#include "rtl.h"
#include "navigator.h"

#include <drivers/drv_hrt.h>

using namespace time_literals;

RTL::RTL(Navigator *navigator) :
	NavigatorMode(navigator),
	ModuleParams(navigator),
	_rtl_direct(navigator),
	_rtl_mission(navigator),
	_rtl_mission_reverse(navigator)
{
	initMission();
}

void RTL::on_inactivation()
{
	switch(_rtl_type)
	{
	case RtlType::RTL_MISSION_FAST:
		_rtl_mission.on_inactivation();
		break;
	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_reverse.on_inactivation();
		break;
	case RtlType::RTL_DIRECT:
		_rtl_direct.on_inactivation();
	default:
		break;
	}
}

void RTL::on_inactive()
{
	PlannedMissionInterface::update();

	setRtlType();

	_rtl_mission.on_inactive();
	_rtl_mission_reverse.on_inactive();
	_rtl_direct.on_inactive();

	// Limit inactive calculation to 1Hz
	hrt_abstime now{hrt_absolute_time()};

	if ((now - _destination_check_time) > 1_s) {
		_destination_check_time = now;

		rtl_time_estimate_s estimated_time;
		switch(_rtl_type)
		{
		case RtlType::RTL_DIRECT:
			estimated_time = _rtl_direct.calc_rtl_time_estimate();
			break;
		case RtlType::RTL_MISSION_FAST:
			estimated_time = _rtl_mission.calc_rtl_time_estimate();
			break;
		case RtlType::RTL_MISSION_FAST_REVERSE:
			estimated_time = _rtl_mission_reverse.calc_rtl_time_estimate();
			break;
		default:
			break;
		}
		_rtl_time_estimate_pub.publish(estimated_time);
	}
}

void RTL::on_activation()
{
	switch(_rtl_type)
	{
	case RtlType::RTL_MISSION_FAST:
		_rtl_mission.on_activation();
		break;
	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_reverse.on_activation();
		break;
	case RtlType::RTL_DIRECT:
		_rtl_direct.on_activation(_enforce_rtl_alt);
		break;
	default:
		break;
	}
}

void RTL::on_active()
{
	PlannedMissionInterface::update();

	switch(_rtl_type)
	{
	case RtlType::RTL_MISSION_FAST:
		_rtl_mission.on_active();
		_rtl_mission_reverse.on_inactive();
		_rtl_direct.on_inactive();
		break;
	case RtlType::RTL_MISSION_FAST_REVERSE:
		_rtl_mission_reverse.on_active();
		_rtl_mission.on_inactive();
		_rtl_direct.on_inactive();
		break;
	case RtlType::RTL_DIRECT:
		_rtl_direct.on_active();
		_rtl_mission_reverse.on_inactive();
		_rtl_mission.on_inactive();
		break;
	default:
		break;
	}
}

void RTL::setRtlType()
{

	if (_param_rtl_type.get() == 2) {
		if (hasMissionLandStart()) {
			_rtl_type = RtlType::RTL_MISSION_FAST;

		} else {
			_rtl_type = RtlType::RTL_MISSION_FAST_REVERSE;
		}

	} else {
		_rtl_type = RtlType::RTL_DIRECT;
	}
}

