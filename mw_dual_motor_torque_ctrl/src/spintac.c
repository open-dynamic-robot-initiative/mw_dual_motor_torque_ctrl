/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, LineStream Technologies Incorporated
 * Copyright (c) 2012, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
* *  Neither the names of Texas Instruments Incorporated, LineStream
 *    Technologies Incorporated, nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/

#include "spintac.h"
#include "main_2mtr.h"


//! \brief      Initalizes the SpinTAC (ST) object
ST_Handle ST_init(void *pMemory, const size_t numBytes)
{
	ST_Handle handle;
	ST_Obj *obj;

	handle = (ST_Handle)pMemory;	// assign the handle
	obj = (ST_Obj *)handle;		// assign the object

	// init the ST PosConv object
	obj->posConvHandle = STPOSCONV_init(&obj->vel.conv, sizeof(ST_PosConv_t));
	// get the ST Version object
	obj->versionHandle = ST_initVersion(&obj->version, sizeof(ST_Ver_t));

	return handle;
}

//! \brief      Setups SpinTAC Position Convert
void ST_setupPosConv_mtr1(ST_Handle handle)
{
	// get object from the handle
	ST_Obj *obj = (ST_Obj *)handle;

    // Initalize SpinTAC Position Convert
	STPOSCONV_setSampleTime_sec(obj->posConvHandle,
			_IQ24(ST_POS_CONV_SAMPLE_TIME));
	STPOSCONV_setERevMaximums_erev(obj->posConvHandle, _IQ24(1.0), 0);
	STPOSCONV_setUnitConversion(obj->posConvHandle,
			USER_IQ_FULL_SCALE_FREQ_Hz,
			ST_POS_CONV_SAMPLE_TIME,
			USER_MOTOR_NUM_POLE_PAIRS);
	STPOSCONV_setMRevMaximum_mrev(obj->posConvHandle, ST_MREV_ROLLOVER);
	STPOSCONV_setLowPassFilterTime_tick(obj->posConvHandle, 3);
	if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
		// The Slip Compensator is only needed for ACIM
		STPOSCONV_setupSlipCompensator(obj->posConvHandle,
				ST_POS_CONV_SAMPLE_TIME, USER_IQ_FULL_SCALE_FREQ_Hz,
				USER_MOTOR_Rr, USER_MOTOR_Ls_d);
	}
	STPOSCONV_setEnable(obj->posConvHandle, true);
}

void ST_setupPosConv_mtr2(ST_Handle handle)
{
	// get object from the handle
	ST_Obj *obj = (ST_Obj *)handle;

    // Initalize SpinTAC Position Convert
	STPOSCONV_setSampleTime_sec(obj->posConvHandle,
			_IQ24(ST_POS_CONV_SAMPLE_TIME_2));
	STPOSCONV_setERevMaximums_erev(obj->posConvHandle, _IQ24(1.0), 0);
	STPOSCONV_setUnitConversion(obj->posConvHandle,
			USER_IQ_FULL_SCALE_FREQ_Hz_2,
			ST_POS_CONV_SAMPLE_TIME_2,
			USER_MOTOR_NUM_POLE_PAIRS_2);
	STPOSCONV_setMRevMaximum_mrev(obj->posConvHandle, ST_MREV_ROLLOVER);
	STPOSCONV_setLowPassFilterTime_tick(obj->posConvHandle, 3);
	if(USER_MOTOR_TYPE_2 ==  MOTOR_Type_Induction) {
		// The Slip Compensator is only needed for ACIM
		STPOSCONV_setupSlipCompensator(obj->posConvHandle,
				ST_POS_CONV_SAMPLE_TIME_2, USER_IQ_FULL_SCALE_FREQ_Hz_2,
				USER_MOTOR_Rr_2, USER_MOTOR_Ls_d_2);
	}
	STPOSCONV_setEnable(obj->posConvHandle, true);
}

void ST_runPosConv(
		ST_Handle handle,
		ENC_Handle encHandle,
		SLIP_Handle slipHandle,
		MATH_vec2 *Idq_pu,
		MOTOR_Type_e motorType)
{
	ST_Obj *stObj = (ST_Obj *)handle;

	// get the electrical angle from the ENC module
	STPOSCONV_setElecAngle_erev(
	        stObj->posConvHandle, ENC_getElecAngle(encHandle));

	if(motorType ==  MOTOR_Type_Induction) {
		// The CurrentVector feedback is only needed for ACIM
		// get the vector of the direct/quadrature current input vector values
		// from CTRL
		STPOSCONV_setCurrentVector(stObj->posConvHandle, Idq_pu);
	}

	// run the SpinTAC Position Converter
	STPOSCONV_run(stObj->posConvHandle);

	if(motorType ==  MOTOR_Type_Induction) {
		// The Slip Velocity is only needed for ACIM
		// update the slip velocity in electrical angle per second, Q24
		SLIP_setSlipVelocity(
		        slipHandle, STPOSCONV_getSlipVelocity(stObj->posConvHandle));
	}
}
