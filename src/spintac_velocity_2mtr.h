#ifndef _SPINTAC_VELOCITY_H_
#define _SPINTAC_VELOCITY_H_
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

//! \file   solutions/instaspin_motion/src/spintac_velocity_2mtr.h
//! \brief  Contains public interface to various functions related
//!         to the SpinTAC (ST) object
//!
//! (C) Copyright 2012, LineStream Technologies, Inc.
//! (C) Copyright 2011, Texas Instruments, Inc.


// **************************************************************************
// the includes
#include "main_2mtr.h"

#include "sw/modules/spintac/src/32b/spintac_vel_ctl.h"
#include "sw/modules/spintac/src/32b/spintac_vel_move.h"
#include "sw/modules/spintac/src/32b/spintac_vel_plan.h"
#include "sw/modules/spintac/src/32b/spintac_vel_id.h"
#include "sw/modules/spintac/src/32b/spintac_pos_conv.h"

//!
//!
//! \defgroup ST ST
//!
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

//! \brief ROTATION UNIT MAXIMUMS
// **************************************************************************
//! \brief Defines the maximum and minimum value at which the Mechanical Revolution [MRev] will rollover.
//! \brief The position signal produced by SpinTAC Position Converter is a sawtooth signal.
//! \brief All SpinTAC Components need to be aware of the bounds of this signal.
//! \brief When the position signal reaches the maximum value it will immediatly go to the minimum value.
//! \brief The minimum value is the negative of the maximum value.
#define ST_MREV_ROLLOVER _IQ(100.0)
//#define ST_MREV_ROLLOVER_2 (100)  // use same value for both motors


//! \brief SAMPLE TIME
// **************************************************************************
//! \brief Defines the number of interrupt ticks per SpinTAC tick
//! \brief Should be the same as the InstSpin-FOC speed controller clock tick
#define ISR_TICKS_PER_SPINTAC_TICK (USER_NUM_ISR_TICKS_PER_CTRL_TICK * USER_NUM_CTRL_TICKS_PER_SPEED_TICK)
#define ISR_TICKS_PER_SPINTAC_TICK_2 (USER_NUM_ISR_TICKS_PER_CTRL_TICK_2 * USER_NUM_CTRL_TICKS_PER_SPEED_TICK_2)

//! \brief Defines the SpinTAC execution period, sec
#define ST_SAMPLE_TIME (ISR_TICKS_PER_SPINTAC_TICK / USER_ISR_FREQ_Hz)
#define ST_SAMPLE_TIME_2 (ISR_TICKS_PER_SPINTAC_TICK_2 / USER_ISR_FREQ_Hz_2)

#define ISR_TICKS_PER_POSCONV_TICK (USER_NUM_ISR_TICKS_PER_CTRL_TICK * USER_NUM_CTRL_TICKS_PER_POSCONV_TICK)
#define ISR_TICKS_PER_POSCONV_TICK_2 (USER_NUM_ISR_TICKS_PER_CTRL_TICK_2 * USER_NUM_CTRL_TICKS_PER_POSCONV_TICK_2)
#define ST_POS_CONV_SAMPLE_TIME (ISR_TICKS_PER_POSCONV_TICK / USER_ISR_FREQ_Hz)
#define ST_POS_CONV_SAMPLE_TIME_2 (ISR_TICKS_PER_POSCONV_TICK_2 / USER_ISR_FREQ_Hz_2)

//! \brief UNIT SCALING
// **************************************************************************
//! \brief Defines the speed scale factor for the system
//! \brief Compile time calculation for scale factor (ratio) used throughout the system
#define ST_SPEED_PU_PER_KRPM (USER_MOTOR_NUM_POLE_PAIRS / (0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz))
#define ST_SPEED_PU_PER_KRPM_2 (USER_MOTOR_NUM_POLE_PAIRS_2 / (0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz_2))

//! \brief Defines the speed scale factor for the system
//! \brief Compile time calculation for scale factor (ratio) used throughout the system
#define ST_SPEED_KRPM_PER_PU ((0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz) / USER_MOTOR_NUM_POLE_PAIRS)
#define ST_SPEED_KRPM_PER_PU_2 ((0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz_2) / USER_MOTOR_NUM_POLE_PAIRS_2)

//! \brief Defines the default inertia for the system, PU/(pu/s^2)
//! \brief This value should be calculated from the inertia estimated with SpinTAC Identify
#define ST_SYSTEM_INERTIA_PU (USER_SYSTEM_INERTIA * ST_SPEED_KRPM_PER_PU * (1.0 / USER_IQ_FULL_SCALE_CURRENT_A))
#define ST_SYSTEM_INERTIA_PU_2 (USER_SYSTEM_INERTIA_2 * ST_SPEED_KRPM_PER_PU_2 * (1.0 / USER_IQ_FULL_SCALE_CURRENT_A_2))

//! \brief Defines the default friction for the system, PU/(pu/s^2)
//! \brief This value should be calculated from the friction estimated with SpinTAC Identify
#define ST_SYSTEM_FRICTION_PU (USER_SYSTEM_FRICTION * ST_SPEED_KRPM_PER_PU * (1.0 / USER_IQ_FULL_SCALE_CURRENT_A))
#define ST_SYSTEM_FRICTION_PU_2 (USER_SYSTEM_FRICTION_2 * ST_SPEED_KRPM_PER_PU_2 * (1.0 / USER_IQ_FULL_SCALE_CURRENT_A_2))


//! \brief SPINTAC IDENTIFY SETTINGS
// **************************************************************************
//! \brief Defines the minimum speed from which the SpinTAC Identify will start running, rpm
//! \brief This value needs to be low enough that the motor is effectivly stopped
#define ST_MIN_ID_SPEED_RPM (5)
#define ST_MIN_ID_SPEED_RPM_2 (5)

//! \brief Defines the minimum speed from which the SpinTAC Identify will start running
//! \breif Compile time calculation that converts the minimum identification speed into pu/s
#define ST_MIN_ID_SPEED_PU (ST_MIN_ID_SPEED_RPM * 0.001 * ST_SPEED_PU_PER_KRPM)
#define ST_MIN_ID_SPEED_PU_2 (ST_MIN_ID_SPEED_RPM_2 * 0.001 * ST_SPEED_PU_PER_KRPM_2)

//! \brief Defines the SpinTAC Identify error code that needs to run to completion
//! \breif Identifies the error code that should not halt operation
#define ST_ID_INCOMPLETE_ERROR (2005)


//! \brief GLOBAL VARIABLE INITIALIZATION
// **************************************************************************
//! \brief Initalization values of SpinTAC global variables
#define ST_VARS_DEFAULTS_MTR1 {0}

#define ST_VARS_DEFAULTS_MTR2 {0}
// **************************************************************************
// the typedefs

//! \brief Defines the velocity components of SpinTAC (ST)
//!
//! This structure is quite stupid with only one element. Keep it nonetheless so
//! that it remains compatible with the GUI, which expects the motor position in
//! st_obj.vel.conv.Pos_mrev
typedef struct _VEL_Params_t
{
    ST_PosConv_t conv;    //!< the position converter (ST_PosConv) object
} VEL_Params_t;

//! \brief Defines the SpinTAC (ST) object
//!
typedef struct _ST_Obj
{
	//! Contains the PosConv object. Exists for compatibility with GUI.
	VEL_Params_t	  vel;
	ST_Ver_t          version;     		//!< the version (ST_Ver) object
	ST_POSCONV_Handle posConvHandle;    //!< Handle for Position Converter (ST_PosConv)
	ST_VER_Handle     versionHandle;    //!< Handle for Version (ST_Ver)
} ST_Obj;

//! \brief Handle
//!
typedef struct _ST_Obj_ *ST_Handle; // SpinTAC Velocity Controller Handle

//! \brief Defines the SpinTAC (ST) global variables
//!
typedef struct _ST_Vars_t
{
    uint16_t           PosConvErrorID;           //!< displays the error seen by the Position Converter (ST_PosConv)
} ST_Vars_t;

// **************************************************************************
// the globals


// **************************************************************************
// the functions

//! \brief      Initalizes the SpinTAC (ST) object
inline ST_Handle ST_init(void *pMemory, const size_t numBytes)
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
inline void ST_setupPosConv_mtr1(ST_Handle handle)
{
	// get object from the handle
	ST_Obj *obj = (ST_Obj *)handle;

    // Initalize SpinTAC Position Convert
	STPOSCONV_setSampleTime_sec(obj->posConvHandle, _IQ24(ST_POS_CONV_SAMPLE_TIME));
	STPOSCONV_setERevMaximums_erev(obj->posConvHandle, _IQ24(1.0), 0);
	STPOSCONV_setUnitConversion(obj->posConvHandle, USER_IQ_FULL_SCALE_FREQ_Hz, ST_POS_CONV_SAMPLE_TIME, USER_MOTOR_NUM_POLE_PAIRS);
	STPOSCONV_setMRevMaximum_mrev(obj->posConvHandle, ST_MREV_ROLLOVER);
	STPOSCONV_setLowPassFilterTime_tick(obj->posConvHandle, 3);
	if(USER_MOTOR_TYPE ==  MOTOR_Type_Induction) {
		// The Slip Compensator is only needed for ACIM
		STPOSCONV_setupSlipCompensator(obj->posConvHandle, ST_POS_CONV_SAMPLE_TIME, USER_IQ_FULL_SCALE_FREQ_Hz, USER_MOTOR_Rr, USER_MOTOR_Ls_d);
	}
	STPOSCONV_setEnable(obj->posConvHandle, true);
}

inline void ST_setupPosConv_mtr2(ST_Handle handle)
{
	// get object from the handle
	ST_Obj *obj = (ST_Obj *)handle;

    // Initalize SpinTAC Position Convert
	STPOSCONV_setSampleTime_sec(obj->posConvHandle, _IQ24(ST_POS_CONV_SAMPLE_TIME_2));
	STPOSCONV_setERevMaximums_erev(obj->posConvHandle, _IQ24(1.0), 0);
	STPOSCONV_setUnitConversion(obj->posConvHandle, USER_IQ_FULL_SCALE_FREQ_Hz_2, ST_POS_CONV_SAMPLE_TIME_2, USER_MOTOR_NUM_POLE_PAIRS_2);
	STPOSCONV_setMRevMaximum_mrev(obj->posConvHandle, ST_MREV_ROLLOVER);
	STPOSCONV_setLowPassFilterTime_tick(obj->posConvHandle, 3);
	if(USER_MOTOR_TYPE_2 ==  MOTOR_Type_Induction) {
		// The Slip Compensator is only needed for ACIM
		STPOSCONV_setupSlipCompensator(obj->posConvHandle, ST_POS_CONV_SAMPLE_TIME_2, USER_IQ_FULL_SCALE_FREQ_Hz_2, USER_MOTOR_Rr_2, USER_MOTOR_Ls_d_2);
	}
	STPOSCONV_setEnable(obj->posConvHandle, true);
}


#ifdef QEP
//! \brief      Runs SpinTAC Positon Convert
extern void ST_runPosConv(ST_Handle handle, ENC_Handle encHandle, SLIP_Handle slipHandle, MATH_vec2 *Idq_pu, MOTOR_Type_e motorType);
#endif


// since this function is missing in the spintac library:

//! \brief      Gets the Mechanical Revolution Maximum (cfg.ROMax_mrev) for SpinTAC Position Converter
//! \param[in]  handle    The handle for the SpinTAC Position Converter Object
//! \return     _iq24     Mechanical Revolution Maximum { unit: [MRev] }
static inline _iq24 STPOSCONV_getMRevMaximum_mrev(ST_POSCONV_Handle handle)
{
	ST_PosConv_t *obj = (ST_PosConv_t *)handle;

	return(obj->cfg.ROMax_mrev);
} // end of STPOSCONV_getMRevMaximum_mrev function


#ifdef __cplusplus
}
#endif // extern "C"

//@} // ingroup
#endif // end of _SPINTAC_VELOCITY_H_ definition
