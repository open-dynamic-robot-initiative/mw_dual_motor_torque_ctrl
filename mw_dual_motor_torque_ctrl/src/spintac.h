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

#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/motor/src/32b/motor.h"
#ifdef QEP
#include "sw/modules/enc/src/32b/enc.h"
#include "sw/modules/slip/src/32b/slip.h"
#endif
#include "spintac_types.h"


//!
//! \defgroup ST ST
//!
//@{

#ifdef __cplusplus
extern "C" {
#endif

// **************************************************************************
// the defines

// ROTATION UNIT MAXIMUMS
// **************************************************************************
//! \brief Defines the maximum and minimum value at which the Mechanical
//! 	   Revolution [MRev] will rollover.
//!
//! The position signal produced by SpinTAC Position Converter is a sawtooth
//! signal. All SpinTAC Components need to be aware of the bounds of this
//! signal. When the position signal reaches the maximum value it will
//! immediatly go to the minimum value. The minimum value is the negative of the
//! maximum value.
#define ST_MREV_ROLLOVER _IQ(100.0)
//#define ST_MREV_ROLLOVER_2 (100)  // use same value for both motors


// SAMPLE TIME
// **************************************************************************
//! \brief Defines the number of interrupt ticks per SpinTAC tick
//! \brief Should be the same as the InstSpin-FOC speed controller clock tick
#define ISR_TICKS_PER_SPINTAC_TICK (USER_NUM_ISR_TICKS_PER_CTRL_TICK \
		* USER_NUM_CTRL_TICKS_PER_SPEED_TICK)
#define ISR_TICKS_PER_SPINTAC_TICK_2 (USER_NUM_ISR_TICKS_PER_CTRL_TICK_2 \
		* USER_NUM_CTRL_TICKS_PER_SPEED_TICK_2)

//! \brief Defines the SpinTAC execution period, sec
#define ST_SAMPLE_TIME (ISR_TICKS_PER_SPINTAC_TICK / USER_ISR_FREQ_Hz)
#define ST_SAMPLE_TIME_2 (ISR_TICKS_PER_SPINTAC_TICK_2 / USER_ISR_FREQ_Hz_2)

#define ISR_TICKS_PER_POSCONV_TICK (USER_NUM_ISR_TICKS_PER_CTRL_TICK \
		* USER_NUM_CTRL_TICKS_PER_POSCONV_TICK)
#define ISR_TICKS_PER_POSCONV_TICK_2 (USER_NUM_ISR_TICKS_PER_CTRL_TICK_2 \
		* USER_NUM_CTRL_TICKS_PER_POSCONV_TICK_2)
#define ST_POS_CONV_SAMPLE_TIME (ISR_TICKS_PER_POSCONV_TICK / USER_ISR_FREQ_Hz)
#define ST_POS_CONV_SAMPLE_TIME_2 (ISR_TICKS_PER_POSCONV_TICK_2 \
		/ USER_ISR_FREQ_Hz_2)


// UNIT SCALING
// **************************************************************************
//! \brief Defines the speed scale factor for the system
//! Compile time calculation for scale factor (ratio) used throughout the system
#define ST_SPEED_PU_PER_KRPM (USER_MOTOR_NUM_POLE_PAIRS \
		/ (0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz))
#define ST_SPEED_PU_PER_KRPM_2 (USER_MOTOR_NUM_POLE_PAIRS_2 \
		/ (0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz_2))

//! \brief Defines the speed scale factor for the system
//!
//! Compile time calculation for scale factor (ratio) used throughout the system
#define ST_SPEED_KRPM_PER_PU ((0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz) \
		/ USER_MOTOR_NUM_POLE_PAIRS)
#define ST_SPEED_KRPM_PER_PU_2 ((0.001 * 60.0 * USER_IQ_FULL_SCALE_FREQ_Hz_2) \
		/ USER_MOTOR_NUM_POLE_PAIRS_2)

//! \brief Defines the default inertia for the system, PU/(pu/s^2)
//!
//! This value should be calculated from the inertia estimated with SpinTAC
//! Identify
#define ST_SYSTEM_INERTIA_PU (USER_SYSTEM_INERTIA * ST_SPEED_KRPM_PER_PU * \
		(1.0 / USER_IQ_FULL_SCALE_CURRENT_A))
#define ST_SYSTEM_INERTIA_PU_2 (USER_SYSTEM_INERTIA_2 * ST_SPEED_KRPM_PER_PU_2 \
		* (1.0 / USER_IQ_FULL_SCALE_CURRENT_A_2))

//! \brief Defines the default friction for the system, PU/(pu/s^2)
//!
//! This value should be calculated from the friction estimated with SpinTAC
//! Identify
#define ST_SYSTEM_FRICTION_PU (USER_SYSTEM_FRICTION * ST_SPEED_KRPM_PER_PU * \
		(1.0 / USER_IQ_FULL_SCALE_CURRENT_A))
#define ST_SYSTEM_FRICTION_PU_2 (USER_SYSTEM_FRICTION_2 * \
		ST_SPEED_KRPM_PER_PU_2 * (1.0 / USER_IQ_FULL_SCALE_CURRENT_A_2))


//! \brief GLOBAL VARIABLE INITIALIZATION
// **************************************************************************
//! \brief Initalization values of SpinTAC global variables
#define ST_VARS_DEFAULTS_MTR1 {0}

#define ST_VARS_DEFAULTS_MTR2 {0}


// **************************************************************************
// the globals


// **************************************************************************
// the functions

//! \brief Initalizes the SpinTAC (ST) object
ST_Handle ST_init(void *pMemory, const size_t numBytes);


//! \brief Setups SpinTAC Position Convert for motor 1
void ST_setupPosConv_mtr1(ST_Handle handle);

//! \brief Setups SpinTAC Position Convert for motor 2
void ST_setupPosConv_mtr2(ST_Handle handle);


#ifdef QEP
//! \brief      Runs SpinTAC Positon Convert
extern void ST_runPosConv(ST_Handle handle, ENC_Handle encHandle,
		SLIP_Handle slipHandle, MATH_vec2 *Idq_pu, MOTOR_Type_e motorType);
#endif


// since this function is missing in the spintac library:

//! \brief      Gets the Mechanical Revolution Maximum (cfg.ROMax_mrev) for
//!             SpinTAC Position Converter
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
