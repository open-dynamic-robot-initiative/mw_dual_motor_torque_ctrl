#ifndef _MAIN_H_
#define _MAIN_H_
/* --COPYRIGHT--,BSD
 * Copyright (c) 2012, Texas Instruments Incorporated
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
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
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
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

//! \file
//! \brief Defines the structures, global initialization, and functions used in
//! \brief MAIN
//!
//! (C) Copyright 2011, Texas Instruments, Inc.
//! (C) Copyright 2019, Max Planck Gesellschaft, New York University

// **************************************************************************
// the includes

// modules
#include "sw/modules/math/src/32b/math.h"
#include "sw/modules/memCopy/src/memCopy.h"
#include "sw/modules/est/src/32b/est.h"
#include "sw/modules/svgen/src/32b/svgen_current.h"
#include "sw/modules/fw/src/32b/fw.h"
#include "sw/modules/fem/src/32b/fem.h"
#include "sw/modules/cpu_usage/src/32b/cpu_usage.h"

#include "sw/modules/clarke/src/32b/clarke.h"
#include "sw/modules/park/src/32b/park.h"
#include "sw/modules/ipark/src/32b/ipark.h"
#include "sw/modules/motor/src/32b/motor.h"
#include "sw/modules/offset/src/32b/offset.h"
#include "sw/modules/pid/src/32b/pid.h"
#include "sw/modules/svgen/src/32b/svgen.h"
#include "sw/modules/traj/src/32b/traj.h"
#include "sw/modules/angle_comp/src/32b/angle_comp.h"
#ifdef QEP
#include "sw/modules/enc/src/32b/enc.h"
#include "sw/modules/slip/src/32b/slip.h"
#endif


// drivers


// platforms
#include "sw/modules/ctrl/src/32b/ctrl_eou.h"
#include "hal_2mtr.h"
#include "user1.h"
#include "user2.h"

// local
#include "spintac_types.h"

// **************************************************************************
// the defines


//! \brief Defines the number of main iterations before global variables are
//! \brief updated
//!
#define NUM_MAIN_TICKS_FOR_GLOBAL_VARIABLE_UPDATE  1

//! \brief Defines the speed acceleration scale factor.
//!
#define MAX_ACCEL_KRPMPS_SF _IQ(USER_MOTOR_NUM_POLE_PAIRS * 1000.0 \
		/ USER_TRAJ_FREQ_Hz / USER_IQ_FULL_SCALE_FREQ_Hz / 60.0)

//! \brief Initialization values of global variables
//!
#define MOTOR_Vars_INIT_Mtr1  { \
		false,  /*  Flag_enableSys */ \
		false,  /*  Flag_runIdentify */ \
		false,  /*  Flag_MotorIdentified */ \
		true,   /*  Flag_enableForceAngle */ \
		/* false,  // Flag_enableFieldWeakening */ \
		/* false,  // Flag_enableRsRecalc */ \
		/* true,   // Flag_enableUserParams */ \
		true,   /*  Flag_enableOffsetRecalc */ \
		/* false,  // Flag_enablePowerWrap */ \
		/* false,  // Flag_enableSpeedCtrl */ \
		true,   /*  Flag_enableAlignment */ \
		/* CTRL_State_Idle, // CtrlState; */ \
		/* EST_State_Idle, // EstState; */ \
		/* USER_ErrorCode_NoError, // UserErrorCode; */ \
		/* {0, CTRL_TargetProc_Unknown, 0, 0}, // CtrlVersion */ \
		_IQ(0.0), /*  IdRef_A */ \
		_IQ(0.0), /*  IqRef_A */ \
		/* _IQ(0.0), // SpeedRef_pu */ \
		/* _IQ(0.1), // SpeedRef_krpm */ \
		/* _IQ(0.0), // SpeedTraj_krpm */ \
		/* _IQ(0.2), // MaxAccel_krpmps */ \
		/* _IQ20(500.0), // MaxJrk_krpmps2 */ \
		_IQ(0.0), /*  Speed_krpm */ \
		/* _IQ(USER_MAX_VS_MAG_PU),  // OverModulation */ \
		/* _IQ(0.1 * USER_MOTOR_MAX_CURRENT), // RsOnlineCurrent_A */ \
		/* 400, // SvgenMaxModulation_ticks */ \
		/* _IQ(0.0), // Flux_Wb */ \
		_IQ(0.0), /*  Torque_Nm */ \
		/* 0.0,  // MagnCurr_A */ \
		/* 0.0,  // Rr_Ohm */ \
		/* 0.0,  // Rs_Ohm */ \
		0.0,  /*  RsOnline_Ohm */ \
		/* 0.0,  // Lsd_H */ \
		/* 0.0,  // Lsq_H */ \
		/* 0.0,  // Flux_VpHz */ \
		/* 0.0, */ \
		/* _IQ(0.0), */ \
		/* _IQ(0.0), */ \
		/* _IQ(0.0), */ \
		/* 0.0, */ \
		/* 0.0, */ \
		/* _IQ(0.0),  // Kp_spd */ \
		/* _IQ(0.0),  // Ki_spd */ \
		_IQ(0.0),  /*  Kp_Idq */ \
		_IQ(0.0),  /*  Ki_Idq */ \
		_IQ(0.0),  /*  Vd */ \
		_IQ(0.0),  /*  Vq */ \
		_IQ(0.0),  /*  Vs */ \
		_IQ(0.8 * USER_MAX_VS_MAG_PU),  /*  VsRef */ \
		_IQ(0.0),  /*  VdcBus_kV */ \
		_IQ(0.0),  /*  Id_A */ \
		_IQ(0.0),  /*  Iq_A */ \
		_IQ(0.0),  /*  Is_A */ \
		{0, 0, 0},  /*  I_bias */ \
		{0, 0, 0},  /* V_bias */ \
		ST_VARS_DEFAULTS_MTR1}

#define MOTOR_Vars_INIT_Mtr2  { \
		false,  /*  Flag_enableSys */ \
		false,  /*  Flag_runIdentify */ \
		false,  /*  Flag_MotorIdentified */ \
		true,   /*  Flag_enableForceAngle */ \
		/* false,  // Flag_enableFieldWeakening */ \
		/* false,  // Flag_enableRsRecalc */ \
		/* true,   // Flag_enableUserParams */ \
		true,   /*  Flag_enableOffsetRecalc */ \
		/* false,  // Flag_enablePowerWrap */ \
		/* false,  // Flag_enableSpeedCtrl */ \
		true,   /*  Flag_enableAlignment */ \
		/* CTRL_State_Idle, // CtrlState; */ \
		/* EST_State_Idle, // EstState; */ \
		/* USER_ErrorCode_NoError, // UserErrorCode; */ \
		/* {0, CTRL_TargetProc_Unknown, 0, 0}, // CtrlVersion */ \
		_IQ(0.0), /*  IdRef_A */ \
		_IQ(0.0), /*  IqRef_A */ \
		/* _IQ(0.0), // SpeedRef_pu */ \
		/* _IQ(0.1), // SpeedRef_krpm */ \
		/* _IQ(0.0), // SpeedTraj_krpm */ \
		/* _IQ(0.2), // MaxAccel_krpmps */ \
		/* _IQ20(500.0), // MaxJrk_krpmps2 */ \
		_IQ(0.0), /*  Speed_krpm */ \
		/* _IQ(USER_MAX_VS_MAG_PU_2),  // OverModulation */ \
		/* _IQ(0.1 * USER_MOTOR_MAX_CURRENT_2), // RsOnlineCurrent_A */ \
		/* 400, // SvgenMaxModulation_ticks */ \
		/* _IQ(0.0), // Flux_Wb */ \
		_IQ(0.0), /*  Torque_Nm */ \
		/* 0.0,  // MagnCurr_A */ \
		/* 0.0,  // Rr_Ohm */ \
		/* 0.0,  // Rs_Ohm */ \
		0.0,  /*  RsOnline_Ohm */ \
		/* 0.0,  // Lsd_H */ \
		/* 0.0,  // Lsq_H */ \
		/* 0.0,  // Flux_VpHz */ \
		/* 0.0, */ \
		/* _IQ(0.0), */ \
		/* _IQ(0.0), */ \
		/* _IQ(0.0), */ \
		/* 0.0, */ \
		/* 0.0, */ \
		/* _IQ(0.0),  // Kp_spd */ \
		/* _IQ(0.0),  // Ki_spd */ \
		_IQ(0.0),  /*  Kp_Idq */ \
		_IQ(0.0),  /*  Ki_Idq */ \
		_IQ(0.0),  /*  Vd */ \
		_IQ(0.0),  /*  Vq */ \
		_IQ(0.0),  /*  Vs */ \
		_IQ(0.8 * USER_MAX_VS_MAG_PU_2),  /*  VsRef */ \
		_IQ(0.0),  /*  VdcBus_kV */ \
		_IQ(0.0),  /*  Id_A */ \
		_IQ(0.0),  /*  Iq_A */ \
		_IQ(0.0),  /*  Is_A */ \
		{0, 0, 0},  /*  I_bias */ \
		{0, 0, 0},  /*  V_bias */ \
		ST_VARS_DEFAULTS_MTR2}

// **************************************************************************
// the typedefs

typedef struct _MOTOR_Vars_t_
{
	bool Flag_enableSys;
	bool Flag_Run_Identify;
	bool Flag_MotorIdentified;
	bool Flag_enableForceAngle;
	//bool Flag_enableFieldWeakening;
	//bool Flag_enableRsRecalc;
	//bool Flag_enableUserParams;
	bool Flag_enableOffsetcalc;
	//bool Flag_enablePowerWarp;
	//bool Flag_enableSpeedCtrl;
	bool Flag_enableAlignment;

	//CTRL_State_e CtrlState;
	//EST_State_e EstState;

	//USER_ErrorCode_e UserErrorCode;

	//CTRL_Version CtrlVersion;

	_iq IdRef_A;
	_iq IqRef_A;
	//_iq SpeedRef_pu;
	//_iq SpeedRef_krpm;
	//_iq SpeedTraj_krpm;
	//_iq MaxAccel_krpmps;
	//_iq20 MaxJrk_krpmps2;
	_iq Speed_krpm;
	//_iq OverModulation;
	//_iq RsOnLineCurrent_A;
	//_iq SvgenMaxModulation_ticks;
	//_iq Flux_Wb;
	_iq Torque_Nm; // TODO set this value?

	//float_t MagnCurr_A;
	//float_t Rr_Ohm;
	//float_t Rs_Ohm;
	float_t RsOnLine_Ohm;
	//float_t Lsd_H;
	//float_t Lsq_H;
	//float_t Flux_VpHz;

	//float_t ipd_excFreq_Hz;
	//_iq     ipd_Kspd;
	//_iq     ipd_excMag_coarse_pu;
	//_iq     ipd_excMag_fine_pu;
	//float   ipd_waitTime_coarse_sec;
	//float   ipd_waitTime_fine_sec;

	//_iq Kp_spd;
	//_iq Ki_spd;

	_iq Kp_Idq;
	_iq Ki_Idq;

	_iq Vd;
	_iq Vq;
	_iq Vs;
	_iq VsRef;
	_iq VdcBus_kV;

	_iq Id_A;
	_iq Iq_A;
	_iq Is_A;

	MATH_vec3 I_bias;
	MATH_vec3 V_bias;

	ST_Vars_t SpinTAC;

} MOTOR_Vars_t;


//! \brief Status message bits.
struct ERROR_BITS {         // bits   description
	uint16_t can_error:1;    // 0
	uint16_t qep_error:1;    // 1
	uint16_t can_recv_timeout:1; // 2
	uint16_t posconv_error:1; // 3
	uint16_t pos_rollover:1;  // 4
	uint16_t rsvd:11;        // 5-15  reserved
};

//! \brief Status message that allows integer or bit access.
typedef union _Error_t_ {
	uint16_t           all;
	struct ERROR_BITS  bit;
} Error_t;


//! \brief Encapsulates data of the QEP Index Watchdog
//! Based on the position of the index, errors in the QEP modules (e.g. missed
//! encoder lines) are detected.
typedef struct _QepIndexWatchdog_t_
{
	//! False until indexPosition_counts is set.
	bool isInitialized;
	//! Position of the first detected index.
	int32_t indexPosition_counts;
	//! Error between indexPosition_counts and the last detected index.
	int32_t indexError_counts;
} QepIndexWatchdog_t;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief The main interrupt service (ISR) routine
//!
interrupt void motor1_ISR(void);
interrupt void motor2_ISR(void);

interrupt void can1_ISR();
interrupt void timer0_ISR();

void runOffsetsCalculation(HAL_MtrSelect_e mtrNum);


//! \brief Updates the global variables
//! 
void updateGlobalVariables(const uint_least8_t mtrNum);

CTRL_Handle CTRL_init(void *pMemory, const size_t numBytes);

void EST_getIdq_pu(EST_Handle handle, MATH_vec2 *pIdq_pu);

#if !defined(FAST_ROM_V1p6)
void EST_setEstParams(EST_Handle handle, USER_Params *pUserParams);
void EST_setupEstIdleState(EST_Handle handle);
#endif

EST_Handle EST_init(void *pMemory, const size_t numBytes);


//! \brief Motor ISR that can be used for both motors.
void generic_motor_ISR(const HAL_MtrSelect_e mtrNum);


//! \brief Send the current status of the board via CAN
inline void setCanStatusMsg();

//! \brief Send data (current, position, etc.) of the specified motor via CAN
void setCanMotorData(const HAL_MtrSelect_e mtrNum);

//! \brief Send status message if the last sent message is older than the
//! \brief period specified via CAN_STATUSMSG_TRANS_FREQ_Hz.
void maybeSendCanStatusMsg();


//! \brief ISR triggered by index of QEP1.
interrupt void qep1IndexISR();
//! \brief ISR triggered by index of QEP2.
interrupt void qep2IndexISR();
//! \brief Helper function for the QEP index ISRs (handles interrupt for QEP mtrNum).
inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum);
//! \brief Check if the QEP error is too high.
inline bool checkEncoderError(const QepIndexWatchdog_t);

//! \brief Check if there is any error and set gErrors accordingly.
//!
//! Also turns the red LED on if there is an error.
void checkErrors();

//! \brief Turn LEDs on/off depending on the system state.
void LED_run(HAL_Handle halHandle);


//@} //defgroup
#endif // end of _MAIN_H_ definition
