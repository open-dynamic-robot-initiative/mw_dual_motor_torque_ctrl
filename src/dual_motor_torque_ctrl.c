// --COPYRIGHT--,BSD
// Copyright (c) 2015, Texas Instruments Incorporated
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// *  Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// *  Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//
// *  Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
// THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
// OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
// WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
// EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// --/COPYRIGHT
//! \file   solutions/instaspin_motion/src/proj_lab12c.c
//! \brief Dual sensored speed control using SpinTAC
//!
//! (C) Copyright 2015, Texas Instruments, Inc.

//! \defgroup PROJ_LAB12C PROJ_LAB12C
//@{

//! \defgroup PROJ_LAB12C_OVERVIEW Project Overview
//!
//! Basic implementation of FOC by using the estimator for angle and speed
//! feedback only.  Adds in SpinTAC Velocity Contol and SpinTAC Velocity Move
//!	Dual sensored speed control using SpinTAC
//!

// **************************************************************************
// the includes

// system includes
#include <math.h>
#include "main_2mtr.h"
#include "virtualspring.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR, "ramfuncs");
#pragma CODE_SECTION(motor2_ISR, "ramfuncs");
#endif

// Include header files used in the main function

// **************************************************************************
// the defines

#define LED_BLINK_FREQ_Hz   2
#define LED_ONBOARD_RED  (GPIO_Number_e)HAL_Gpio_LED2
#define LED_ONBOARD_BLUE (GPIO_Number_e)HAL_Gpio_LED3
#define LED_EXTERN_RED GPIO_Number_12
#define LED_EXTERN_YELLOW GPIO_Number_13
#define LED_EXTERN_GREEN GPIO_Number_22

#define CAN_TRANSMISSION_TIMER_FREQ_Hz 1000
#define CAN_STATUSMSG_TRANS_FREQ_Hz 1

#define TIMER0_FREQ_Hz CAN_TRANSMISSION_TIMER_FREQ_Hz

#define QEP_MAX_INDEX_ERROR  (1./360.) * USER_MOTOR_ENCODER_LINES  // 1 degree

#define POTI_RESULT1 ADC_ResultNumber_0
#define POTI_RESULT2 ADC_ResultNumber_8

// **************************************************************************
// the globals

//! Used for various debugging stuff.
uint32_t gFoobar = 0;

CLARKE_Handle   clarkeHandle_I[2];  //!< the handle for the current Clarke
									//!< transform
CLARKE_Obj      clarke_I[2];        //!< the current Clarke transform object

PARK_Handle     parkHandle[2];      //!< the handle for the current Parke
									//!< transform
PARK_Obj        park[2];            //!< the current Parke transform object

CLARKE_Handle   clarkeHandle_V[2];  //!< the handle for the voltage Clarke
									//!< transform
CLARKE_Obj      clarke_V[2];        //!< the voltage Clarke transform object

EST_Handle      estHandle[2];       //!< the handle for the estimator

PID_Obj         pid[2][3];          //!< three objects for PID controllers
									//!< 0 - Speed, 1 - Id, 2 - Iq
PID_Handle      pidHandle[2][3];    //!< three handles for PID controllers
									//!< 0 - Speed, 1 - Id, 2 - Iq
uint16_t        stCntSpeed[2] = {0, 0};      //!< count variable to decimate the execution
									//!< of SpinTAC Velocity Control
uint16_t        stCntPosConv[2] = {0, 0}; //!< count variable to decimate the
                                          //!< execution of SpinTAC Position Converter
//! Store this to array so it can be used in generic_motor_ISR.
const uint16_t  gNumIsrTicksPerPosConvTick[2] = {
        ISR_TICKS_PER_POSCONV_TICK,
        ISR_TICKS_PER_POSCONV_TICK_2
};

IPARK_Handle    iparkHandle[2];     //!< the handle for the inverse Park
									//!< transform
IPARK_Obj       ipark[2];           //!< the inverse Park transform object

SVGEN_Handle    svgenHandle[2];     //!< the handle for the space vector generator
SVGEN_Obj       svgen[2];           //!< the space vector generator object

ENC_Handle      encHandle[2];      //!< the handle for the encoder
ENC_Obj         enc[2];            //!< the encoder object

SLIP_Handle     slipHandle[2];     //!< the handle for the slip compensator
SLIP_Obj        slip[2];           //!< the slip compensator object

HAL_Handle      halHandle;         //!< the handle for the hardware abstraction
								   //!< layer for common CPU setup
HAL_Obj         hal;               //!< the hardware abstraction layer object

ANGLE_COMP_Handle    angleCompHandle[2];  //!< the handle for the angle compensation
ANGLE_COMP_Obj       angleComp[2];        //!< the angle compensation object

HAL_Handle_mtr  halHandleMtr[2]; //!< the handle for the hardware abstraction
								 //!< layer specific to the motor board.
HAL_Obj_mtr     halMtr[2];       //!< the hardware abstraction layer object
                                 //!< specific to the motor board.

HAL_PwmData_t   gPwmData[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},   //!< contains the
		{_IQ(0.0), _IQ(0.0), _IQ(0.0)}};  //!< pwm values for each phase.
										//!< -1.0 is 0%, 1.0 is 100%

HAL_AdcData_t   gAdcData[2];       //!< contains three current values, three
								   //!< voltage values and one DC buss value

MATH_vec3       gOffsets_I_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},  //!< contains
		{_IQ(0.0), _IQ(0.0), _IQ(0.0)}}; //!< the offsets for the current feedback

MATH_vec3       gOffsets_V_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},  //!< contains
		{_IQ(0.0), _IQ(0.0), _IQ(0.0)}}; //!< the offsets for the voltage feedback

MATH_vec2       gIdq_ref_pu[2] = {{_IQ(0.0), _IQ(0.0)},  //!< contains the Id and
		{_IQ(0.0), _IQ(0.0)}}; //!< Iq references


MATH_vec2       gVdq_out_pu[2] = {{_IQ(0.0), _IQ(0.0)},  //!< contains the output
		{_IQ(0.0), _IQ(0.0)}}; //!< Vd and Vq from the current controllers


MATH_vec2       gIdq_pu[2] = {{_IQ(0.0), _IQ(0.0)},   //!< contains the Id and Iq
		{_IQ(0.0), _IQ(0.0)}};  //!< measured values

FILTER_FO_Handle  filterHandle[2][6];            //!< the handles for the 3-current and 3-voltage filters for offset calculation
FILTER_FO_Obj     filter[2][6];                  //!< the 3-current and 3-voltage filters for offset calculation
uint32_t gOffsetCalcCount[2] = {0, 0};

USER_Params     gUserParams[2];

uint32_t gAlignCount[2] = {0, 0};

ST_Obj          st_obj[2];      //!< the SpinTAC objects
ST_Handle       stHandle[2];    //!< the handles for the SpinTAC objects

VIRTUALSPRING_Handle springHandle[2];
VIRTUALSPRING_Obj spring[2];

uint16_t gLEDcnt[2] = {0, 0};

volatile MOTOR_Vars_t gMotorVars[2] = {MOTOR_Vars_INIT_Mtr1, MOTOR_Vars_INIT_Mtr2};   //!< the global motor
//!< variables that are defined in main.h and
//!< used for display in the debugger's watch
//!< window

#ifdef FLASH
// Used for running BackGround in flash, and ISR in RAM
extern uint16_t *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;
#endif

#ifdef DRV8301_SPI
// Watch window interface to the 8301 SPI
DRV_SPI_8301_Vars_t gDrvSpi8301Vars[2];
#endif

#ifdef DRV8305_SPI
// Watch window interface to the 8305 SPI
DRV_SPI_8305_Vars_t gDrvSpi8305Vars[2];
#endif

_iq gFlux_pu_to_Wb_sf[2];

_iq gFlux_pu_to_VpHz_sf[2];

_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];

_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];

_iq gSpeed_krpm_to_pu_sf[2];

_iq gSpeed_pu_to_krpm_sf[2];

_iq gSpeed_hz_to_krpm_sf[2];

_iq gCurrent_A_to_pu_sf[2];

//uint32_t seq_counter = 0;

//! Timestamp based on timer 0 (increased by one at each timer interrupt).
uint32_t gTimer0_stamp = 0;

//! Last time the blinking status LED was toggled (based on gTimer0_stamp).
uint32_t gStatusLedBlinkLastToggleTime = 0;

//! Last time a status message was sent via CAN (based on gTimer0_stamp).
uint32_t gCanLastStatusMsgTime = 0;

uint32_t gEnabledCanMessages = 0;

//! Errors that occured in the system.  gErrors.all == 0 if no errors occured.
Error_t gErrors;

//! QEP index watchdog data for both encoders.
QepIndexWatchdog_t gQepIndexWatchdog[2] = {
		{.isInitialized = false, .indexError_counts = 0},
		{.isInitialized = false, .indexError_counts = 0}};



// **************************************************************************
// the functions

// Little helper function
inline void setCanMboxStatus(const uint32_t mbox, const uint32_t status)
{
	if (status) {
		gEnabledCanMessages |= mbox;
	} else {
		gEnabledCanMessages &= ~mbox;
	}
}

void main(void)
{
	// IMPORTANT NOTE: If you are not familiar with MotorWare coding guidelines
	// please refer to the following document:
	// C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf

	// Only used if running from FLASH
	// Note that the variable FLASH is defined by the project

#ifdef FLASH
	// Copy time critical code and Flash setup code to RAM
	// The RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
	// symbols are created by the linker. Refer to the linker files.
	memCopy((uint16_t *)&RamfuncsLoadStart, (uint16_t *)&RamfuncsLoadEnd,
			(uint16_t *)&RamfuncsRunStart);
#endif

	// At the beginning, there are no errors
	gErrors.all = 0;

	// initialize the Hardware Abstraction Layer  (HAL)
	// halHandle will be used throughout the code to interface with the HAL
	// (set parameters, get and set functions, etc) halHandle is required since
	// this is how all objects are interfaced, and it allows interface with
	// multiple objects by simply passing a different handle. The use of
	// handles is explained in this document:
	// C:/ti/motorware/motorware_1_01_00_1x/docs/motorware_coding_standards.pdf
	halHandle = HAL_init(&hal, sizeof(hal));


	// initialize the user parameters
	// This function initializes all values of structure gUserParams with
	// values defined in user.h. The values in gUserParams will be then used by
	// the hardware abstraction layer (HAL) to configure peripherals such as
	// PWM, ADC, interrupts, etc.
	USER_setParamsMtr1(&gUserParams[HAL_MTR1]);
	USER_setParamsMtr2(&gUserParams[HAL_MTR2]);

	// set the hardware abstraction layer parameters
	// This function initializes all peripherals through a Hardware Abstraction
	// Layer (HAL). It uses all values stored in gUserParams.
	HAL_setParams(halHandle, &gUserParams[HAL_MTR1]);

	// Overwrite GPIO Qualification Settings
	// =====================================
	// To allow fast movement with lots-of-lines-encoders, the sampling period of
	// the GPIO qualification filter has to be reduced (otherwise encoder pulses
	// get rejected as noise).  The following lines overwrite the settings done
	// in HAL_setupGpio() (hal.c).
	// "period = 11" results in actual sampling period 11*2*(1/90MHz) = 0.24us
	// Note: Setting the period is done for blocks of GPIO pins.
	//
	// GPIO 16-23 (covering eQEP1)
	GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_16, 11); //GPIO16-23
	// GPIO 50-55 and 56-58 (covering eQEP2)
	GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_50, 11); //GPIO50-55
	GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_56, 11); //GPIO56-58

	// Overwrite the settings for timer0 (we want it faster)
	overwriteSetupTimer0(halHandle, TIMER0_FREQ_Hz);

	// initialize the estimator
	estHandle[HAL_MTR1] = EST_init((void *)USER_EST_HANDLE_ADDRESS, 0x200);
	estHandle[HAL_MTR2] = EST_init((void *)USER_EST_HANDLE_ADDRESS_1, 0x200);

	{
		uint_least8_t mtrNum;

		for(mtrNum=HAL_MTR1;mtrNum<=HAL_MTR2;mtrNum++)
		{

			// initialize the individual motor hal files
			halHandleMtr[mtrNum] = HAL_init_mtr(
					&halMtr[mtrNum],
					sizeof(halMtr[mtrNum]),
					(HAL_MtrSelect_e)mtrNum);

			// Setup each motor board to its specific setting
			HAL_setParamsMtr(
			        halHandleMtr[mtrNum], halHandle, &gUserParams[mtrNum]);

			{
				// These function calls are used to initialize the estimator
				// with ROM function calls. It needs the specific address where
				// the controller object is declared by the ROM code.
				CTRL_Handle ctrlHandle = CTRL_init(
						(void *)USER_CTRL_HANDLE_ADDRESS, 0x200);
				CTRL_Obj *obj = (CTRL_Obj *)ctrlHandle;

				// this sets the estimator handle (part of the controller
				// object) to the same value initialized above by the
				// EST_init() function call. This is done so the next function
				// implemented in ROM, can successfully initialize the
				// estimator as part of the controller object.
				obj->estHandle = estHandle[mtrNum];

				// initialize the estimator through the controller. These three
				// function calls are needed for the F2806xF/M implementation
				// of InstaSPIN.
				CTRL_setParams(ctrlHandle, &gUserParams[mtrNum]);
				CTRL_setUserMotorParams(ctrlHandle);
				CTRL_setupEstIdleState(ctrlHandle);
			}

			//Compensates for the delay introduced
			//from the time when the system inputs are sampled to when the PWM
			//voltages are applied to the motor windings.
			angleCompHandle[mtrNum] = ANGLE_COMP_init(
					&angleComp[mtrNum],
					sizeof(angleComp[mtrNum]));
			ANGLE_COMP_setParams(angleCompHandle[mtrNum],
					gUserParams[mtrNum].iqFullScaleFreq_Hz,
					gUserParams[mtrNum].pwmPeriod_usec,
					gUserParams[mtrNum].numPwmTicksPerIsrTick);


			// initialize the Clarke modules
			// Clarke handle initialization for current signals
			clarkeHandle_I[mtrNum] = CLARKE_init(
					&clarke_I[mtrNum],
					sizeof(clarke_I[mtrNum]));

			// Clarke handle initialization for voltage signals
			clarkeHandle_V[mtrNum] = CLARKE_init(
					&clarke_V[mtrNum],
					sizeof(clarke_V[mtrNum]));

			// Park handle initialization for current signals
			parkHandle[mtrNum] = PARK_init(
			        &park[mtrNum], sizeof(park[mtrNum]));

			// compute scaling factors for flux and torque calculations
			gFlux_pu_to_Wb_sf[mtrNum] = USER_computeFlux_pu_to_Wb_sf(
					&gUserParams[mtrNum]);

			gFlux_pu_to_VpHz_sf[mtrNum] = USER_computeFlux_pu_to_VpHz_sf(
					&gUserParams[mtrNum]);

			gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum] =
			        USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(
			                &gUserParams[mtrNum]);

			gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum] =
			        USER_computeTorque_Flux_Iq_pu_to_Nm_sf(
			                &gUserParams[mtrNum]);

			gSpeed_krpm_to_pu_sf[mtrNum] = _IQ(
					(float_t)gUserParams[mtrNum].motor_numPolePairs * 1000.0
					/ (gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0));

			gSpeed_pu_to_krpm_sf[mtrNum] = _IQ(
					(gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0)
					/ ((float_t)gUserParams[mtrNum].motor_numPolePairs
					        * 1000.0));

			gSpeed_hz_to_krpm_sf[mtrNum] = _IQ(
			        60.0
			        / (float_t)gUserParams[mtrNum].motor_numPolePairs
					/ 1000.0);

			gCurrent_A_to_pu_sf[mtrNum] = _IQ(
					1.0 / gUserParams[mtrNum].iqFullScaleCurrent_A);

			// initialize the speed reference in kilo RPM where base speed is
			// USER_IQ_FULL_SCALE_FREQ_Hz.
			// Set 10 Hz electrical frequency as initial value, so the kRPM
			// value would be: 10 * 60 / motor pole pairs / 1000.
			gMotorVars[mtrNum].SpeedRef_krpm = _IQmpy(
					_IQ(10.0), gSpeed_hz_to_krpm_sf[mtrNum]);

			// disable Rs recalculation
			EST_setFlag_enableRsRecalc(estHandle[mtrNum], false);

			// set the number of current sensors
			setupClarke_I(clarkeHandle_I[mtrNum],
					gUserParams[mtrNum].numCurrentSensors);

			// set the number of voltage sensors
			setupClarke_V(clarkeHandle_V[mtrNum],
					gUserParams[mtrNum].numVoltageSensors);

			// initialize the PID controllers
			pidSetup((HAL_MtrSelect_e)mtrNum);

			// initialize the inverse Park module
			iparkHandle[mtrNum] = IPARK_init(&ipark[mtrNum],
					sizeof(ipark[mtrNum]));

			// initialize the space vector generator module
			svgenHandle[mtrNum] = SVGEN_init(&svgen[mtrNum],
					sizeof(svgen[mtrNum]));

			// initialize and configure offsets using filters
			{
				uint16_t cnt = 0;
				_iq b0 = _IQ(gUserParams[mtrNum].offsetPole_rps
						/ (float_t)gUserParams[mtrNum].ctrlFreq_Hz);
				_iq a1 = (b0 - _IQ(1.0));
				_iq b1 = _IQ(0.0);

				for(cnt=0;cnt<6;cnt++)
				{
					filterHandle[mtrNum][cnt] = FILTER_FO_init(
							&filter[mtrNum][cnt],
							sizeof(filter[mtrNum][0]));
					FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
					FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
					FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt],
							_IQ(0.0),
							_IQ(0.0));
				}

				gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
			}

			// initialize the encoder module
			encHandle[mtrNum] = ENC_init(&enc[mtrNum], sizeof(enc[mtrNum]));

			// initialize the slip compensation module
			slipHandle[mtrNum] = SLIP_init(
			        &slip[mtrNum], sizeof(slip[mtrNum]));
			// setup the SLIP module
			SLIP_setup(slipHandle[mtrNum],
			        _IQ(gUserParams[mtrNum].ctrlPeriod_sec));

			// setup faults
			HAL_setupFaults(halHandleMtr[mtrNum]);

			// initialize the SpinTAC Components
			stHandle[mtrNum] = ST_init(
			        &st_obj[mtrNum], sizeof(st_obj[mtrNum]));

			// init virtual spring handle
			springHandle[mtrNum] = VIRTUALSPRING_init(
			        &spring[mtrNum], sizeof(spring[mtrNum]));
			VIRTUALSPRING_setup(springHandle[mtrNum],
			        10, _IQ(2.0), st_obj[mtrNum].vel.conv.cfg.ROMax_mrev);

		} // End of for loop
	}

	// setup the encoder module
	ENC_setup(
			encHandle[HAL_MTR1],
			1,
			USER_MOTOR_NUM_POLE_PAIRS,
			USER_MOTOR_ENCODER_LINES,
			0,
			USER_IQ_FULL_SCALE_FREQ_Hz,
			USER_ISR_FREQ_Hz,
			8000.0);
	ENC_setup(
			encHandle[HAL_MTR2],
			1,
			USER_MOTOR_NUM_POLE_PAIRS_2,
			USER_MOTOR_ENCODER_LINES_2,
			0,
			USER_IQ_FULL_SCALE_FREQ_Hz_2,
			USER_ISR_FREQ_Hz_2,
			8000.0);

	// setup encoder index interrupts
	setupQepIndexInterrupt(halHandle, halHandleMtr);

	// setup the SpinTAC Components
	ST_setupPosConv_mtr1(stHandle[HAL_MTR1]);
	ST_setupPosConv_mtr2(stHandle[HAL_MTR2]);

	// set the pre-determined current and voltage feeback offset values
	gOffsets_I_pu[HAL_MTR1].value[0] = _IQ(I_A_offset);
	gOffsets_I_pu[HAL_MTR1].value[1] = _IQ(I_B_offset);
	gOffsets_I_pu[HAL_MTR1].value[2] = _IQ(I_C_offset);
	gOffsets_V_pu[HAL_MTR1].value[0] = _IQ(V_A_offset);
	gOffsets_V_pu[HAL_MTR1].value[1] = _IQ(V_B_offset);
	gOffsets_V_pu[HAL_MTR1].value[2] = _IQ(V_C_offset);

	gOffsets_I_pu[HAL_MTR2].value[0] = _IQ(I_A_offset_2);
	gOffsets_I_pu[HAL_MTR2].value[1] = _IQ(I_B_offset_2);
	gOffsets_I_pu[HAL_MTR2].value[2] = _IQ(I_C_offset_2);
	gOffsets_V_pu[HAL_MTR2].value[0] = _IQ(V_A_offset_2);
	gOffsets_V_pu[HAL_MTR2].value[1] = _IQ(V_B_offset_2);
	gOffsets_V_pu[HAL_MTR2].value[2] = _IQ(V_C_offset_2);

	// initialize the interrupt vector table
	HAL_initIntVectorTable(halHandle);

	// enable the ADC interrupts
	HAL_enableAdcInts(halHandle);

	// enable global interrupts
	HAL_enableGlobalInts(halHandle);

	// enable debug interrupts
	HAL_enableDebugInt(halHandle);

	// enable the Timer 0 interrupts
	HAL_enableTimer0Int(halHandle);
	{   // define ISR function
		// TODO: move this to a function
		PIE_Obj *pie = (PIE_Obj *)hal.pieHandle;
		ENABLE_PROTECTED_REGISTER_WRITE_MODE;
		pie->TINT0 = &timer0_ISR;
		DISABLE_PROTECTED_REGISTER_WRITE_MODE;
	}

	// disable the PWM
	HAL_disablePwm(halHandleMtr[HAL_MTR1]);
	HAL_disablePwm(halHandleMtr[HAL_MTR2]);

	{// config GPIO 13 and 22 as output (for LEDs)
		// TODO: Move this to hal_2mtr.c?  Maybe create an own copy of hal.c with all the modifications?
		HAL_Obj *obj = (HAL_Obj *)halHandle;
		GPIO_setLow(obj->gpioHandle, GPIO_Number_13);
		GPIO_setDirection(obj->gpioHandle,
				GPIO_Number_13,
				GPIO_Direction_Output);

		GPIO_setLow(obj->gpioHandle, GPIO_Number_22);
		GPIO_setDirection(obj->gpioHandle,
				GPIO_Number_22,
				GPIO_Direction_Output);
	}

	// initialize SpinTAC Velocity Control watch window variables
	gMotorVars[HAL_MTR1].SpinTAC.VelCtlBw_radps = STVELCTL_getBandwidth_radps(
			st_obj[HAL_MTR1].velCtlHandle);
	gMotorVars[HAL_MTR2].SpinTAC.VelCtlBw_radps = STVELCTL_getBandwidth_radps(
			st_obj[HAL_MTR2].velCtlHandle);
	// initialize the watch window with maximum and minimum Iq reference
	gMotorVars[HAL_MTR1].SpinTAC.VelCtlOutputMax_A = _IQmpy(
			STVELCTL_getOutputMaximum(st_obj[HAL_MTR1].velCtlHandle),
			_IQ(gUserParams[HAL_MTR1].iqFullScaleCurrent_A));
	gMotorVars[HAL_MTR1].SpinTAC.VelCtlOutputMin_A = _IQmpy(
			STVELCTL_getOutputMinimum(st_obj[HAL_MTR1].velCtlHandle),
			_IQ(gUserParams[HAL_MTR1].iqFullScaleCurrent_A));
	gMotorVars[HAL_MTR2].SpinTAC.VelCtlOutputMax_A = _IQmpy(
			STVELCTL_getOutputMaximum(st_obj[HAL_MTR2].velCtlHandle),
			_IQ(gUserParams[HAL_MTR2].iqFullScaleCurrent_A));
	gMotorVars[HAL_MTR2].SpinTAC.VelCtlOutputMin_A = _IQmpy(
			STVELCTL_getOutputMinimum(st_obj[HAL_MTR2].velCtlHandle),
			_IQ(gUserParams[HAL_MTR2].iqFullScaleCurrent_A));

	// enable the system by default
	gMotorVars[HAL_MTR1].Flag_enableSys = true;

#ifdef DRV8301_SPI
	// turn on the DRV8301 if present
	HAL_enableDrv(halHandleMtr[HAL_MTR1]);
	HAL_enableDrv(halHandleMtr[HAL_MTR2]);
	// initialize the DRV8301 interface
	HAL_setupDrvSpi(halHandleMtr[HAL_MTR1], &gDrvSpi8301Vars[HAL_MTR1]);
	HAL_setupDrvSpi(halHandleMtr[HAL_MTR2], &gDrvSpi8301Vars[HAL_MTR2]);
#endif

#ifdef DRV8305_SPI
	// turn on the DRV8305 if present
	HAL_enableDrv(halHandleMtr[HAL_MTR1]);
	HAL_enableDrv(halHandleMtr[HAL_MTR2]);
	// initialize the DRV8305 interface
	HAL_setupDrvSpi(halHandleMtr[HAL_MTR1], &gDrvSpi8305Vars[HAL_MTR1]);
	HAL_setupDrvSpi(halHandleMtr[HAL_MTR2], &gDrvSpi8305Vars[HAL_MTR2]);
#endif


	// Setup CAN interface
	CAN_initECanaGpio(halHandle);
	CAN_initECana();
	CAN_setupMboxes();

	{ // Set ISR for CAN interrupt
		PIE_Obj *pie = hal.pieHandle;
		ENABLE_PROTECTED_REGISTER_WRITE_MODE;
		pie->ECAN1INT = &can1_ISR;
		DISABLE_PROTECTED_REGISTER_WRITE_MODE;

		PIE_enableInt(hal.pieHandle, PIE_GroupNumber_9, PIE_InterruptSource_ECANA1);
		// enable the cpu interrupt for PIE group 9 interrupts
		CPU_enableInt(hal.cpuHandle, CPU_IntNumber_9);
	}


	// turn red led off (no errors so far)
	HAL_turnLedOff(halHandle, LED_ONBOARD_RED);
	HAL_turnLedOff(halHandle, LED_EXTERN_RED);

	// Begin the background loop
	for(;;)
	{
		// Waiting for enable system flag to be set
		// Motor 1 Flag_enableSys is the master control.
		while(!(gMotorVars[HAL_MTR1].Flag_enableSys))
		{
			HAL_turnLedOff(halHandle, LED_ONBOARD_BLUE);
			HAL_turnLedOff(halHandle, LED_EXTERN_GREEN);

			maybeSendCanStatusMsg();
		}

		// loop while the enable system flag is true
		// Motor 1 Flag_enableSys is the master control.
		while(gMotorVars[HAL_MTR1].Flag_enableSys)
		{
			uint_least8_t mtrNum = HAL_MTR1;


			// Show system and motor status using the blue LED
			if (gMotorVars[0].Flag_Run_Identify || gMotorVars[1].Flag_Run_Identify)
			{
				// toggle status LED
				if(gStatusLedBlinkLastToggleTime
						< (gTimer0_stamp - TIMER0_FREQ_Hz / LED_BLINK_FREQ_Hz))
				{
					HAL_toggleLed(halHandle, LED_ONBOARD_BLUE);
					HAL_toggleLed(halHandle, LED_EXTERN_GREEN);
					gStatusLedBlinkLastToggleTime = gTimer0_stamp;
				}
			}
			else
			{
				HAL_turnLedOn(halHandle, LED_ONBOARD_BLUE);
				HAL_turnLedOn(halHandle, LED_EXTERN_GREEN);
			}


			// Error checks
			gErrors.bit.qep_error = (checkEncoderError(gQepIndexWatchdog[0]) || checkEncoderError(gQepIndexWatchdog[1]));

			if (gErrors.all) {
				// When there is an error, shut down the system to be safe
				gMotorVars[HAL_MTR1].Flag_enableSys = false;
				HAL_turnLedOn(halHandle, LED_ONBOARD_RED);
				HAL_turnLedOn(halHandle, LED_EXTERN_RED);
			} else {
				HAL_turnLedOff(halHandle, LED_ONBOARD_RED);
				HAL_turnLedOff(halHandle, LED_EXTERN_RED);
			}


			// Send status message via CAN
			maybeSendCanStatusMsg();


			for(mtrNum=HAL_MTR1;mtrNum<=HAL_MTR2;mtrNum++)
			{

				// If Flag_enableSys is set AND Flag_Run_Identify is set THEN
				// enable PWMs and set the speed reference
				if(gMotorVars[mtrNum].Flag_Run_Identify)
				{
					// update estimator state
					EST_updateState(estHandle[mtrNum], 0);

#ifdef FAST_ROM_V1p6
					// call this function to fix 1p6. This is only used for
					// F2806xF/M implementation of InstaSPIN (version 1.6 of
					// ROM), since the inductance calculation is not done
					// correctly in ROM, so this function fixes that ROM bug.
					softwareUpdate1p6(estHandle[mtrNum], &gUserParams[mtrNum]);
#endif

					// enable the PWM
					HAL_enablePwm(halHandleMtr[mtrNum]);

					// enable SpinTAC Velocity Control
					STVELCTL_setEnable(st_obj[mtrNum].velCtlHandle, true);
					// provide bandwidth setting to SpinTAC Velocity Control
					STVELCTL_setBandwidth_radps(st_obj[mtrNum].velCtlHandle,
							gMotorVars[mtrNum].SpinTAC.VelCtlBw_radps);
					// provide output maximum and minimum setting to SpinTAC
					// Velocity Control
					STVELCTL_setOutputMaximums(
							st_obj[mtrNum].velCtlHandle,
							_IQmpy(gMotorVars[mtrNum].SpinTAC.VelCtlOutputMax_A,
									gCurrent_A_to_pu_sf[mtrNum]),
							_IQmpy(gMotorVars[mtrNum].SpinTAC.VelCtlOutputMin_A,
									gCurrent_A_to_pu_sf[mtrNum]));
				}
				else  // Flag_enableSys is set AND Flag_Run_Identify is not set
				{
					// set estimator to Idle
					EST_setIdle(estHandle[mtrNum]);

					// disable the PWM
					HAL_disablePwm(halHandleMtr[mtrNum]);

					// clear integrator outputs
					PID_setUi(pidHandle[mtrNum][0], _IQ(0.0));
					PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));
					PID_setUi(pidHandle[mtrNum][2], _IQ(0.0));

					// clear Id and Iq references
					gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
					gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);

					// place SpinTAC Velocity Control into reset
					STVELCTL_setEnable(st_obj[mtrNum].velCtlHandle, false);
					// set SpinTAC Velocity Move start & end velocity to 0
					STVELMOVE_setVelocityEnd(
					        st_obj[mtrNum].velMoveHandle, _IQ(0.0));
					STVELMOVE_setVelocityStart(
					        st_obj[mtrNum].velMoveHandle, _IQ(0.0));
				}

				// update the global variables
				updateGlobalVariables(estHandle[mtrNum], mtrNum);

				// enable/disable the forced angle
				EST_setFlag_enableForceAngle(estHandle[mtrNum],
						gMotorVars[mtrNum].Flag_enableForceAngle);

				// set target speed
				gMotorVars[mtrNum].SpeedRef_pu = _IQmpy(
						gMotorVars[mtrNum].SpeedRef_krpm,
						gSpeed_krpm_to_pu_sf[mtrNum]);

#ifdef DRV8301_SPI
				HAL_writeDrvData(
				        halHandleMtr[mtrNum], &gDrvSpi8301Vars[mtrNum]);
				HAL_readDrvData(
				        halHandleMtr[mtrNum], &gDrvSpi8301Vars[mtrNum]);
#endif
#ifdef DRV8305_SPI
				HAL_writeDrvData(
				        halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
				HAL_readDrvData(
				        halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif

			} // end of for loop
		} // end of while(gFlag_enableSys) loop

		// disable the PWM
		HAL_disablePwm(halHandleMtr[HAL_MTR1]);
		HAL_disablePwm(halHandleMtr[HAL_MTR2]);

		gMotorVars[HAL_MTR1].Flag_Run_Identify = false;
		gMotorVars[HAL_MTR2].Flag_Run_Identify = false;

	} // end of for(;;) loop
} // end of main() function


//! \brief     The main ISR that implements the motor control.
interrupt void motor1_ISR(void)
{
    //HAL_setGpioHigh(halHandle, GPIO_Number_12);

//	// toggle status LED
//	if(gLEDcnt[HAL_MTR1]++
//	        > (uint_least32_t)(USER_ISR_FREQ_Hz / LED_BLINK_FREQ_Hz))
//	{
//	    HAL_toggleLed(halHandle, (GPIO_Number_e)HAL_Gpio_LED2);
//	    gLEDcnt[HAL_MTR1] = 0;
//	}

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

	generic_motor_ISR(HAL_MTR1,
	        _IQ(USER_MOTOR_RES_EST_CURRENT), _IQ(USER_MAX_VS_MAG_PU));

	//HAL_setGpioLow(halHandle, GPIO_Number_12);

	return;
} // end of motor1_ISR() function


interrupt void motor2_ISR(void)
{
	//HAL_setGpioHigh(halHandle, GPIO_Number_13);

//	// toggle status LED
//	if(gLEDcnt[HAL_MTR2]++
//	        > (uint_least32_t)(USER_ISR_FREQ_Hz_2 / LED_BLINK_FREQ_Hz))
//	{
//	    HAL_toggleLed(halHandle, (GPIO_Number_e)HAL_Gpio_LED3);
//	    gLEDcnt[HAL_MTR2] = 0;
//	}

	// acknowledge the ADC interrupt
	HAL_acqAdcInt(halHandle, ADC_IntNumber_2);

	generic_motor_ISR(HAL_MTR2,
	        _IQ(USER_MOTOR_RES_EST_CURRENT_2), _IQ(USER_MAX_VS_MAG_PU_2));


	//HAL_setGpioLow(halHandle, GPIO_Number_13);

	return;
} // end of motor2_ISR() function


void generic_motor_ISR(
        const HAL_MtrSelect_e mtrNum,
        const _iq user_motor_res_est_current,
        const _iq user_max_vs_mag_pu)
{
    // Declaration of local variables
	static _iq angle_pu[2] = {_IQ(0.0), _IQ(0.0)};
	_iq speed_pu = _IQ(0.0);
	_iq oneOverDcBus;
	MATH_vec2 Iab_pu;
	MATH_vec2 Vab_pu;
	MATH_vec2 phasor;


	// convert the ADC data
	HAL_readAdcDataWithOffsets(
	        halHandle, halHandleMtr[mtrNum], &gAdcData[mtrNum]);

	// remove offsets
	gAdcData[mtrNum].I.value[0] =
	        gAdcData[mtrNum].I.value[0] - gOffsets_I_pu[mtrNum].value[0];
	gAdcData[mtrNum].I.value[1] =
	        gAdcData[mtrNum].I.value[1] - gOffsets_I_pu[mtrNum].value[1];
	gAdcData[mtrNum].I.value[2] =
	        gAdcData[mtrNum].I.value[2] - gOffsets_I_pu[mtrNum].value[2];
	gAdcData[mtrNum].V.value[0] =
	        gAdcData[mtrNum].V.value[0] - gOffsets_V_pu[mtrNum].value[0];
	gAdcData[mtrNum].V.value[1] =
	        gAdcData[mtrNum].V.value[1] - gOffsets_V_pu[mtrNum].value[1];
	gAdcData[mtrNum].V.value[2] =
	        gAdcData[mtrNum].V.value[2] - gOffsets_V_pu[mtrNum].value[2];

	// run Clarke transform on current.  Three values are passed, two values
	// are returned.
	CLARKE_run(clarkeHandle_I[mtrNum], &gAdcData[mtrNum].I, &Iab_pu);

	// compute the sine and cosine phasor values which are part of the
	// Park transform calculations. Once these values are computed,
	// they are copied into the PARK module, which then uses them to
	// transform the voltages from Alpha/Beta to DQ reference frames.
	phasor.value[0] = _IQcosPU(angle_pu[mtrNum]);
	phasor.value[1] = _IQsinPU(angle_pu[mtrNum]);

	// set the phasor in the Park transform
	PARK_setPhasor(parkHandle[mtrNum], &phasor);

	// Run the Park module.  This converts the current vector from
	// stationary frame values to synchronous frame values.
	PARK_run(parkHandle[mtrNum], &Iab_pu, &gIdq_pu[mtrNum]);

	// compute the electrical angle
	ENC_calcElecAngle(
	        encHandle[mtrNum], HAL_getQepPosnCounts(halHandleMtr[mtrNum]));

	if(++stCntPosConv[mtrNum] >= gNumIsrTicksPerPosConvTick[mtrNum])
	{
	    stCntPosConv[mtrNum] = 0;
		// Calculate the feedback speed
		ST_runPosConv(
				stHandle[mtrNum],
				encHandle[mtrNum],
				slipHandle[mtrNum],
				&gIdq_pu[mtrNum],
				gUserParams[mtrNum].motor_type);
	}

	// run the appropriate controller
	if(gMotorVars[mtrNum].Flag_Run_Identify)
	{
		// Declaration of local variables.
		_iq refValue;
		_iq fbackValue;
		_iq outMax_pu;

		// check if the motor should be forced into encoder alignment
		if(gMotorVars[mtrNum].Flag_enableAlignment == false)
		{
			// When appropriate, update the current reference.
			// This mechanism provides the decimation for the upper level
			// control loop.
			// FIXME: This is using the wrong decimation
			if(++stCntSpeed[mtrNum]
			              >= gUserParams[mtrNum].numCtrlTicksPerSpeedTick)
			{
				// Reset the Speed execution counter.
				stCntSpeed[mtrNum] = 0;

				// TODO: Is it easily possible to remove (with #ifdef) all virtual spring stuff for Flash build?

				// If virtual spring mode is deactivated, reset IqRef to 0
				if (VIRTUALSPRING_isJustDisabled(springHandle[mtrNum])) {
				    gMotorVars[mtrNum].IqRef_A = 0;
				}
				// If virtual spring mode is activated, reset position offset
				else if (VIRTUALSPRING_isJustEnabled(springHandle[mtrNum])) {
				    VIRTUALSPRING_scheduleResetOffset(springHandle[mtrNum]);
				}

				// If spring is enabled, set IqRef based on it
				if (VIRTUALSPRING_isEnabled(springHandle[mtrNum])) {
					VIRTUALSPRING_run(springHandle[mtrNum],
							st_obj[mtrNum].vel.conv.Pos_mrev);
				    gMotorVars[mtrNum].IqRef_A =
				            VIRTUALSPRING_getIqRef_A(springHandle[mtrNum]);
				}
				else {
					if (mtrNum == HAL_MTR1) {
						gMotorVars[mtrNum].IqRef_A = ECanaMboxes.MBOX1.MDL.all;
					} else {
						gMotorVars[mtrNum].IqRef_A = ECanaMboxes.MBOX1.MDH.all;
					}
				}


				gIdq_ref_pu[mtrNum].value[1] = _IQmpy(
				        gMotorVars[mtrNum].IqRef_A,
				        _IQ(1.0 / USER_IQ_FULL_SCALE_CURRENT_A));
			}

			// generate the motor electrical angle
			if(gUserParams[mtrNum].motor_type == MOTOR_Type_Induction)
			{
				// update the electrical angle for the SLIP module
				SLIP_setElectricalAngle(slipHandle[mtrNum],
						ENC_getElecAngle(encHandle[mtrNum]));
				// compute the amount of slip
				SLIP_run(slipHandle[mtrNum]);
				// set magnetic angle
				angle_pu[mtrNum] = SLIP_getMagneticAngle(slipHandle[mtrNum]);
			}
			else
			{
				angle_pu[mtrNum] = ENC_getElecAngle(encHandle[mtrNum]);
			}

			speed_pu = STPOSCONV_getVelocity(st_obj[mtrNum].posConvHandle);
		}
		else
		{  // the alignment procedure is in effect

			// force motor angle and speed to 0
			angle_pu[mtrNum] = _IQ(0.0);
			speed_pu = _IQ(0.0);

			// set D-axis current to Rs estimation current
			gIdq_ref_pu[mtrNum].value[0] = _IQmpy(user_motor_res_est_current,
			        _IQ(1.0 / USER_IQ_FULL_SCALE_CURRENT_A));
			// set Q-axis current to 0
			gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);

			// save encoder reading when forcing motor into alignment
			if(gUserParams[mtrNum].motor_type == MOTOR_Type_Pm)
			{
				ENC_setZeroOffset(encHandle[mtrNum],
						(uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[mtrNum])
								- HAL_getQepPosnCounts(halHandleMtr[mtrNum])));
			}

			// if alignment counter exceeds threshold, exit alignment
			if(gAlignCount[mtrNum]++
					>= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine])
			{
				gMotorVars[mtrNum].Flag_enableAlignment = false;
				gAlignCount[mtrNum] = 0;
				gIdq_ref_pu[mtrNum].value[0] = _IQ(0.0);
			}
		}

		// Get the reference value for the d-axis current controller.
		refValue = gIdq_ref_pu[mtrNum].value[0];

		// Get the actual value of Id
		fbackValue = gIdq_pu[mtrNum].value[0];

		// The next instruction executes the PI current controller for the
		// d axis and places its output in Vdq_pu.value[0], which is the
		// control voltage along the d-axis (Vd)
		PID_run(pidHandle[mtrNum][1],
				refValue,
				fbackValue,
				&(gVdq_out_pu[mtrNum].value[0]));

		// get the Iq reference value
		refValue = gIdq_ref_pu[mtrNum].value[1];

		// get the actual value of Iq
		fbackValue = gIdq_pu[mtrNum].value[1];

		// The voltage limits on the output of the q-axis current controller
		// are dynamic, and are dependent on the output voltage from the d-axis
		// current controller.  In other words, the d-axis current controller
		// gets first dibs on the available voltage, and the q-axis current
		// controller gets what's left over.  That is why the d-axis current
		// controller executes first. The next instruction calculates the
		// maximum limits for this voltage as:
		// Vq_min_max = +/- sqrt(Vbus^2 - Vd^2)
		outMax_pu = _IQsqrt(_IQmpy(user_max_vs_mag_pu, user_max_vs_mag_pu)
				- _IQmpy(gVdq_out_pu[mtrNum].value[0],
						gVdq_out_pu[mtrNum].value[0]));

		// Set the limits to +/- outMax_pu
		PID_setMinMax(pidHandle[mtrNum][2], -outMax_pu, outMax_pu);

		// The next instruction executes the PI current controller for the
		// q axis and places its output in Vdq_pu.value[1], which is the
		// control voltage vector along the q-axis (Vq)
		PID_run(pidHandle[mtrNum][2],
				refValue,
				fbackValue,
				&(gVdq_out_pu[mtrNum].value[1]));

		// The voltage vector is now calculated and ready to be applied to the
		// motor in the form of three PWM signals.  However, even though the
		// voltages may be supplied to the PWM module now, they won't be
		// applied to the motor until the next PWM cycle. By this point, the
		// motor will have moved away from the angle that the voltage vector
		// was calculated for, by an amount which is proportional to the
		// sampling frequency and the speed of the motor.  For steady-state
		// speeds, we can calculate this angle delay and compensate for it.
		ANGLE_COMP_run(angleCompHandle[mtrNum], speed_pu, angle_pu[mtrNum]);
		angle_pu[mtrNum] = ANGLE_COMP_getAngleComp_pu(angleCompHandle[mtrNum]);

		// compute the sine and cosine phasor values which are part of the
		// inverse Park transform calculations. Once these values are computed,
		// they are copied into the IPARK module, which then uses them to
		// transform the voltages from DQ to Alpha/Beta reference frames.
		phasor.value[0] = _IQcosPU(angle_pu[mtrNum]);
		phasor.value[1] = _IQsinPU(angle_pu[mtrNum]);

		// set the phasor in the inverse Park transform
		IPARK_setPhasor(iparkHandle[mtrNum], &phasor);

		// Run the inverse Park module.  This converts the voltage vector from
		// synchronous frame values to stationary frame values.
		IPARK_run(iparkHandle[mtrNum], &gVdq_out_pu[mtrNum], &Vab_pu);

		// These 3 statements compensate for variations in the DC bus by
		// adjusting the PWM duty cycle. The goal is to achieve the same
		// volt-second product regardless of the DC bus value.  To do this, we
		// must divide the desired voltage values by the DC bus value.  Or...it
		// is easier to multiply by 1/(DC bus value).
		oneOverDcBus = _IQdiv(_IQ(1.0), gAdcData[mtrNum].dcBus);
		Vab_pu.value[0] = _IQmpy(Vab_pu.value[0], oneOverDcBus);
		Vab_pu.value[1] = _IQmpy(Vab_pu.value[1], oneOverDcBus);

		// Now run the space vector generator (SVGEN) module.
		// There is no need to do an inverse CLARKE transform, as this is
		// handled in the SVGEN_run function.
		SVGEN_run(svgenHandle[mtrNum], &Vab_pu, &(gPwmData[mtrNum].Tabc));
	}
	else if(gMotorVars[mtrNum].Flag_enableOffsetcalc == true)
	{
		runOffsetsCalculation(mtrNum);
	}
	else  // gMotorVars.Flag_Run_Identify = 0
	{
		// disable the PWM
		HAL_disablePwm(halHandleMtr[mtrNum]);

		// Set the PWMs to 50% duty cycle
		gPwmData[mtrNum].Tabc.value[0] = _IQ(0.0);
		gPwmData[mtrNum].Tabc.value[1] = _IQ(0.0);
		gPwmData[mtrNum].Tabc.value[2] = _IQ(0.0);
	}

	// write to the PWM compare registers, and then we are done!
	HAL_writePwmData(halHandleMtr[mtrNum], &gPwmData[mtrNum]);
}


interrupt void can1_ISR()
{
	// The same ISR is used by the eCAN module, independent of the source of the
	// interrupt.  This means, we have to distinguish the various cases here,
	// based on the values of certain registers (see SPRUH18f, section 16.13)

	// This ISR is used by GIF1

	// TODO: SPRU074F, sec. 3.4.3.2 describes how to correctly handle all cases
	// (I don't fully understand what is meant by a "half-word read", though).

	// Since this ISR is currently only used for mailbox 0 receives, we only
	// check for this here and simply ignore other cases that call this ISR
	// (there shouldn't be any).
	// Note: ECanaRegs.CANGIF1.bit.MIV1 contains the number of the mailbox that
	// caused this interrupt (this should always be 0 for now).
	if (ECanaRegs.CANRMP.bit.RMP0 == 1)
	{
		uint32_t cmd_id = ECanaMboxes.MBOX0.MDH.all;
		uint32_t cmd_val = ECanaMboxes.MBOX0.MDL.all;

		switch (cmd_id)
		{
		case CAN_CMD_ENABLE_SYS: // enable system
			gMotorVars[HAL_MTR1].Flag_enableSys = cmd_val;
			break;
		case CAN_CMD_ENABLE_MTR1: // run motor 1
			gMotorVars[HAL_MTR1].Flag_Run_Identify = cmd_val;
			break;
		case CAN_CMD_ENABLE_MTR2: // run motor 2
			gMotorVars[HAL_MTR2].Flag_Run_Identify = cmd_val;
			break;
		case CAN_CMD_ENABLE_VSPRING1: // motor 1 enable spring
			spring[HAL_MTR1].enabled = cmd_val;
			break;
		case CAN_CMD_ENABLE_VSPRING2: // motor 2 enable spring
			spring[HAL_MTR2].enabled = cmd_val;
			break;

		case CAN_CMD_SEND_CURRENT:
			setCanMboxStatus(CAN_MBOX_OUT_Iq, cmd_val);
			break;
		case CAN_CMD_SEND_POSITION:
			setCanMboxStatus(CAN_MBOX_OUT_ENC_POS, cmd_val);
			break;
		case CAN_CMD_SEND_VELOCITY:
			setCanMboxStatus(CAN_MBOX_OUT_SPEED, cmd_val);
			break;
		case CAN_CMD_SEND_ADC6:
			setCanMboxStatus(CAN_MBOX_OUT_ADC6, cmd_val);
			break;
		case CAN_CMD_SEND_ALL:
			if (cmd_val) {
				gEnabledCanMessages = (CAN_MBOX_OUT_Iq
						| CAN_MBOX_OUT_ENC_POS
						| CAN_MBOX_OUT_SPEED
						| CAN_MBOX_OUT_ADC6);
			} else {
				gEnabledCanMessages = 0;
			}
			break;
		}

		// Acknowledge interrupt
		ECanaRegs.CANRMP.bit.RMP0 = 1; // clear by writing 1
	}

	// acknowledge interrupt from PIE
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);
}


interrupt void timer0_ISR()
{
	++gTimer0_stamp;

	uint32_t mbox_mask = gEnabledCanMessages;

	// TODO: better abortion handling
	// If there is still an old message waiting for transmission, abort it
	if (ECanaRegs.CANTRS.all & mbox_mask)
	{
		HAL_turnLedOn(halHandle, LED_EXTERN_YELLOW);
		// TODO: notify about the issue
		CAN_abort(mbox_mask);
		// is it okay to block here (or at least wait for a while)?
	}
	else
	{
		HAL_turnLedOff(halHandle, LED_EXTERN_YELLOW);
	}

	setCanMotorData(HAL_MTR1);
	setCanMotorData(HAL_MTR2);

	CAN_setAdcIn6Values(
			HAL_readAdcResult(halHandle, POTI_RESULT1),
			HAL_readAdcResult(halHandle, POTI_RESULT2));

	// send messages via CAN
	CAN_send(mbox_mask);

	// acknowledge interrupt
    HAL_acqTimer0Int(halHandle);
}


void pidSetup(HAL_MtrSelect_e mtrNum)
{
	// This equation uses the scaled maximum voltage vector, which is
	// already in per units, hence there is no need to include the #define
	// for USER_IQ_FULL_SCALE_VOLTAGE_V
	_iq maxVoltage_pu = _IQ(gUserParams[mtrNum].maxVsMag_pu *
			gUserParams[mtrNum].voltage_sf);


	float_t fullScaleCurrent = gUserParams[mtrNum].iqFullScaleCurrent_A;
	float_t fullScaleVoltage = gUserParams[mtrNum].iqFullScaleVoltage_V;
	float_t IsrPeriod_sec = 1.0e-6 * gUserParams[mtrNum].pwmPeriod_usec *
			gUserParams[mtrNum].numPwmTicksPerIsrTick;
	float_t Ls_d = gUserParams[mtrNum].motor_Ls_d;
	float_t Ls_q = gUserParams[mtrNum].motor_Ls_q;
	float_t Rs = gUserParams[mtrNum].motor_Rs;

	// This lab assumes that motor parameters are known, and it does not
	// perform motor ID, so the R/L parameters are known and defined in
	// user.h
	float_t RoverLs_d = Rs / Ls_d;
	float_t RoverLs_q = Rs / Ls_q;

	// For the current controller, Kp = Ls*bandwidth(rad/sec)  But in order
	// to be used, it must be converted to per unit values by multiplying
	// by fullScaleCurrent and then dividing by fullScaleVoltage.  From the
	// statement below, we see that the bandwidth in rad/sec is equal to
	// 0.25/IsrPeriod_sec, which is equal to USER_ISR_FREQ_HZ/4. This means
	// that by setting Kp as described below, the bandwidth in Hz is
	// USER_ISR_FREQ_HZ/(8*pi).
	_iq Kp_Id = _IQ((0.25 * Ls_d * fullScaleCurrent) / (IsrPeriod_sec
			* fullScaleVoltage));

	// In order to achieve pole/zero cancellation (which reduces the
	// closed-loop transfer function from a second-order system to a
	// first-order system), Ki must equal Rs/Ls.  Since the output of the
	// Ki gain stage is integrated by a DIGITAL integrator, the integrator
	// input must be scaled by 1/IsrPeriod_sec.  That's just the way
	// digital integrators work.  But, since IsrPeriod_sec is a constant,
	// we can save an additional multiplication operation by lumping this
	// term with the Ki value.
	_iq Ki_Id = _IQ(RoverLs_d * IsrPeriod_sec);

	// Now do the same thing for Kp for the q-axis current controller.
	// If the motor is not an IPM motor, Ld and Lq are the same, which
	// means that Kp_Iq = Kp_Id
	_iq Kp_Iq = _IQ((0.25 * Ls_q * fullScaleCurrent) / (IsrPeriod_sec
			* fullScaleVoltage));

	// Do the same thing for Ki for the q-axis current controller.  If the
	// motor is not an IPM motor, Ld and Lq are the same, which means that
	// Ki_Iq = Ki_Id.
	_iq Ki_Iq = _IQ(RoverLs_q * IsrPeriod_sec);

	// There are two PI controllers; two current
	// controllers.  Each PI controller has two coefficients; Kp and Ki.
	// So you have a total of four coefficients that must be defined.
	// This is for the Id current controller
	pidHandle[mtrNum][1] = PID_init(&pid[mtrNum][1], sizeof(pid[mtrNum][1]));
	// This is for the Iq current controller
	pidHandle[mtrNum][2] = PID_init(&pid[mtrNum][2], sizeof(pid[mtrNum][2]));

	// The following instructions load the parameters for the d-axis
	// current controller.
	// P term = Kp_Id, I term = Ki_Id, D term = 0
	PID_setGains(pidHandle[mtrNum][1], Kp_Id, Ki_Id, _IQ(0.0));

	// Largest negative voltage = -maxVoltage_pu, largest positive
	// voltage = maxVoltage_pu
	PID_setMinMax(pidHandle[mtrNum][1], -maxVoltage_pu, maxVoltage_pu);

	// Set the initial condition value for the integrator output to 0
	PID_setUi(pidHandle[mtrNum][1], _IQ(0.0));

	// The following instructions load the parameters for the q-axis
	// current controller.
	// P term = Kp_Iq, I term = Ki_Iq, D term = 0
	PID_setGains(pidHandle[mtrNum][2], Kp_Iq, Ki_Iq, _IQ(0.0));

	// The largest negative voltage = 0 and the largest positive
	// voltage = 0.  But these limits are updated every single ISR before
	// actually executing the Iq controller. The limits depend on how much
	// voltage is left over after the Id controller executes. So having an
	// initial value of 0 does not affect Iq current controller execution.
	PID_setMinMax(pidHandle[mtrNum][2], _IQ(0.0), _IQ(0.0));

	// Set the initial condition value for the integrator output to 0
	PID_setUi(pidHandle[mtrNum][2], _IQ(0.0));
}


void runOffsetsCalculation(HAL_MtrSelect_e mtrNum)
{
	uint16_t cnt;

	// enable the PWM
	HAL_enablePwm(halHandleMtr[mtrNum]);

	for(cnt=0;cnt<3;cnt++)
	{
		// Set the PWMs to 50% duty cycle
		gPwmData[mtrNum].Tabc.value[cnt] = _IQ(0.0);

		// reset offsets used
		gOffsets_I_pu[mtrNum].value[cnt] = _IQ(0.0);
		gOffsets_V_pu[mtrNum].value[cnt] = _IQ(0.0);

		// run offset estimation
		FILTER_FO_run(filterHandle[mtrNum][cnt],
				gAdcData[mtrNum].I.value[cnt]);
		FILTER_FO_run(filterHandle[mtrNum][cnt+3],
				gAdcData[mtrNum].V.value[cnt]);
	}

	if(gOffsetCalcCount[mtrNum]++
			>= gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine])
	{
		gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
		gOffsetCalcCount[mtrNum] = 0;

		for(cnt=0;cnt<3;cnt++)
		{
			// get calculated offsets from filter
			gOffsets_I_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(
					filterHandle[mtrNum][cnt]);
			gOffsets_V_pu[mtrNum].value[cnt] = FILTER_FO_get_y1(
					filterHandle[mtrNum][cnt+3]);

			// clear filters
			FILTER_FO_setInitialConditions(
			        filterHandle[mtrNum][cnt], _IQ(0.0), _IQ(0.0));
			FILTER_FO_setInitialConditions(
			        filterHandle[mtrNum][cnt+3], _IQ(0.0), _IQ(0.0));
		}
	}

	return;
} // end of runOffsetsCalculation() function


//! \brief  Call this function to fix 1p6. This is only used for F2806xF/M
//! \brief  implementation of InstaSPIN (version 1.6 of ROM) since the
//! \brief  inductance calculation is not done correctly in ROM, so this
//! \brief  function fixes that ROM bug.
void softwareUpdate1p6(EST_Handle handle, USER_Params *pUserParams)
{
	float_t iqFullScaleVoltage_V = pUserParams->iqFullScaleVoltage_V;
	float_t iqFullScaleCurrent_A = pUserParams->iqFullScaleCurrent_A;
	float_t voltageFilterPole_rps = pUserParams->voltageFilterPole_rps;
	float_t motorLs_d = pUserParams->motor_Ls_d;
	float_t motorLs_q = pUserParams->motor_Ls_q;

	float_t fullScaleInductance = iqFullScaleVoltage_V
			/ (iqFullScaleCurrent_A
					* voltageFilterPole_rps);
	float_t Ls_coarse_max = _IQ30toF(EST_getLs_coarse_max_pu(handle));
	int_least8_t lShift = ceil(log(motorLs_d / (Ls_coarse_max
			* fullScaleInductance)) / log(2.0));
	uint_least8_t Ls_qFmt = 30 - lShift;
	float_t L_max = fullScaleInductance * pow(2.0, lShift);
	_iq Ls_d_pu = _IQ30(motorLs_d / L_max);
	_iq Ls_q_pu = _IQ30(motorLs_q / L_max);

	// store the results
	EST_setLs_d_pu(handle, Ls_d_pu);
	EST_setLs_q_pu(handle, Ls_q_pu);
	EST_setLs_qFmt(handle, Ls_qFmt);

	return;
} // end of softwareUpdate1p6() function


//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle handle, const uint_least8_t numCurrentSensors)
{
	_iq alpha_sf, beta_sf;

	// initialize the Clarke transform module for current
	if(numCurrentSensors == 3)
	{
		alpha_sf = _IQ(MATH_ONE_OVER_THREE);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	}
	else if(numCurrentSensors == 2)
	{
		alpha_sf = _IQ(1.0);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	}
	else
	{
		alpha_sf = _IQ(0.0);
		beta_sf = _IQ(0.0);
	}

	// set the parameters
	CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
	CLARKE_setNumSensors(handle, numCurrentSensors);

	return;
} // end of setupClarke_I() function


//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle handle, const uint_least8_t numVoltageSensors)
{
	_iq alpha_sf, beta_sf;

	// initialize the Clarke transform module for voltage
	if(numVoltageSensors == 3)
	{
		alpha_sf = _IQ(MATH_ONE_OVER_THREE);
		beta_sf = _IQ(MATH_ONE_OVER_SQRT_THREE);
	}
	else
	{
		alpha_sf = _IQ(0.0);
		beta_sf = _IQ(0.0);
	}

	// In other words, the only acceptable number of voltage sensors is three.
	// set the parameters
	CLARKE_setScaleFactors(handle, alpha_sf, beta_sf);
	CLARKE_setNumSensors(handle, numVoltageSensors);

	return;
} // end of setupClarke_V() function


//! \brief     Update the global variables (gMotorVars).
//! \param[in] handle  The estimator (EST) handle
void updateGlobalVariables(EST_Handle handle, const uint_least8_t mtrNum)
{
	// get the speed estimate
	gMotorVars[mtrNum].Speed_krpm = _IQmpy(
			STPOSCONV_getVelocityFiltered(st_obj[mtrNum].posConvHandle),
			gSpeed_pu_to_krpm_sf[mtrNum]);

	// Get the DC buss voltage
	gMotorVars[mtrNum].VdcBus_kV = _IQmpy(gAdcData[mtrNum].dcBus,
			_IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));

	// read Vd and Vq vectors per units
	gMotorVars[mtrNum].Vd = gVdq_out_pu[mtrNum].value[0];
	gMotorVars[mtrNum].Vq = gVdq_out_pu[mtrNum].value[1];

	// calculate vector Vs in per units: (Vs = sqrt(Vd^2 + Vq^2))
	gMotorVars[mtrNum].Vs =
			_IQsqrt(_IQmpy(gMotorVars[mtrNum].Vd, gMotorVars[mtrNum].Vd)
			+ _IQmpy(gMotorVars[mtrNum].Vq, gMotorVars[mtrNum].Vq));

	// read Id and Iq vectors in amps
	gMotorVars[mtrNum].Id_A = _IQmpy(gIdq_pu[mtrNum].value[0],
			_IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
	gMotorVars[mtrNum].Iq_A = _IQmpy(gIdq_pu[mtrNum].value[1],
			_IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));

	// calculate vector Is in amps:  (Is_A = sqrt(Id_A^2 + Iq_A^2))
	gMotorVars[mtrNum].Is_A =
			_IQsqrt(_IQmpy(gMotorVars[mtrNum].Id_A, gMotorVars[mtrNum].Id_A)
			+ _IQmpy(gMotorVars[mtrNum].Iq_A, gMotorVars[mtrNum].Iq_A));

	// gets the Velocity Controller status
	gMotorVars[mtrNum].SpinTAC.VelCtlStatus = STVELCTL_getStatus(
			st_obj[mtrNum].velCtlHandle);

	// get the inertia setting
	gMotorVars[mtrNum].SpinTAC.InertiaEstimate_Aperkrpm = _IQmpy(
			STVELCTL_getInertia(st_obj[mtrNum].velCtlHandle),
			_IQ(((float_t)gUserParams[mtrNum].motor_numPolePairs
					/ (0.001 * 60.0 * gUserParams[mtrNum].iqFullScaleFreq_Hz))
					* gUserParams[mtrNum].iqFullScaleCurrent_A));

	// get the friction setting
	gMotorVars[mtrNum].SpinTAC.FrictionEstimate_Aperkrpm = _IQmpy(
			STVELCTL_getFriction(st_obj[mtrNum].velCtlHandle),
			_IQ(((float_t)gUserParams[mtrNum].motor_numPolePairs
					/ (0.001 * 60.0 * gUserParams[mtrNum].iqFullScaleFreq_Hz))
					* gUserParams[mtrNum].iqFullScaleCurrent_A));

	// get the Velocity Controller error
	gMotorVars[mtrNum].SpinTAC.VelCtlErrorID =
			STVELCTL_getErrorID(st_obj[mtrNum].velCtlHandle);

	// get the Velocity Move status
	gMotorVars[mtrNum].SpinTAC.VelMoveStatus =
			STVELMOVE_getStatus(st_obj[mtrNum].velMoveHandle);

	// get the Velocity Move profile time
	gMotorVars[mtrNum].SpinTAC.VelMoveTime_ticks =
			STVELMOVE_getProfileTime_tick(st_obj[mtrNum].velMoveHandle);

	// get the Velocity Move error
	gMotorVars[mtrNum].SpinTAC.VelMoveErrorID =
			STVELMOVE_getErrorID(st_obj[mtrNum].velMoveHandle);

	// get the Position Converter error
	gMotorVars[mtrNum].SpinTAC.PosConvErrorID =
			STPOSCONV_getErrorID(st_obj[mtrNum].posConvHandle);

	return;
} // end of updateGlobalVariables() function

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


void setCanStatusMsg()
{
	CAN_StatusMsg_t status;

	// Send status message via CAN
	status.all = 0;
	status.bit.enable_system = gMotorVars[HAL_MTR1].Flag_enableSys;
	status.bit.run_motor1 = gMotorVars[HAL_MTR1].Flag_Run_Identify;
	status.bit.ready_motor1 = !gMotorVars[HAL_MTR1].Flag_enableAlignment;
	status.bit.run_motor2 = gMotorVars[HAL_MTR2].Flag_Run_Identify;
	status.bit.ready_motor2 = !gMotorVars[HAL_MTR2].Flag_enableAlignment;
	status.bit.system_error = gErrors.all != 0;

	CAN_setStatusMsg(status);
}


void setCanMotorData(const HAL_MtrSelect_e mtrNum)
{
	_iq current_iq = _IQmpy(gIdq_pu[mtrNum].value[1],
			_IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
	_iq position = st_obj[mtrNum].vel.conv.Pos_mrev;
	_iq speed = _IQmpy(
			STPOSCONV_getVelocityFiltered(st_obj[mtrNum].posConvHandle),
			gSpeed_pu_to_krpm_sf[mtrNum]);

	if (mtrNum == HAL_MTR1)
	{
		// send number sequence for debugging
//		position = seq_counter;
//		speed = seq_counter++;
		CAN_setDataMotor1(current_iq, position, speed);
	}
	else
	{
		CAN_setDataMotor2(current_iq, position, speed);
	}

	return;
}


void maybeSendCanStatusMsg()
{
	if(gCanLastStatusMsgTime
			< (gTimer0_stamp - TIMER0_FREQ_Hz / CAN_STATUSMSG_TRANS_FREQ_Hz))
	{
		// If there is still an old message waiting for transmission, abort it
		if (ECanaRegs.CANTRS.all & CAN_MBOX_OUT_STATUSMSG)
		{
			// TODO: notify about the issue
			CAN_abort(CAN_MBOX_OUT_STATUSMSG);
			// is it okay to block here (or at least wait for a while)?
		}

		setCanStatusMsg();
		CAN_send(CAN_MBOX_OUT_STATUSMSG);

		gCanLastStatusMsgTime = gTimer0_stamp;
	}
}


void overwriteSetupTimer0(HAL_Handle handle, const uint32_t timerFreq_Hz)
{
  HAL_Obj  *obj = (HAL_Obj *)handle;
  uint32_t  timerPeriod_cnts = ((uint32_t)gUserParams[0].systemFreq_MHz * 1e6l) / timerFreq_Hz - 1;

  // use timer 0 for CAN transmissions
  TIMER_setDecimationFactor(obj->timerHandle[0], 0);
  TIMER_setEmulationMode(obj->timerHandle[0], TIMER_EmulationMode_RunFree);
  TIMER_setPeriod(obj->timerHandle[0], timerPeriod_cnts);
  TIMER_setPreScaler(obj->timerHandle[0], 0);

  return;
}  // end of HAL_setupTimers() function


void setupQepIndexInterrupt(HAL_Handle halHandle, HAL_Handle_mtr halHandleMtr[2])
{
	uint_least8_t mtrNum;
	HAL_Obj *halObj = (HAL_Obj *)halHandle;
	HAL_Obj_mtr *halMtrObj;
	PIE_Obj *pie = (PIE_Obj *)halObj->pieHandle;

	// specify ISRs
	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
	pie->EQEP1_INT = &qep1IndexISR;
	pie->EQEP2_INT = &qep2IndexISR;
	//PIE_registerPieIntHandler(obj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP1, &qepIndexISR);
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	for(mtrNum=HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++)
	{
		halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

		// enable QEP interrupt for index
		QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);
		QEP_enable_interrupt(halMtrObj->qepHandle, QEINT_Iel);
	}

	// enable the corresponding interrupts in PIE (group 5)
	PIE_enableInt(halObj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP1);
	PIE_enableInt(halObj->pieHandle, PIE_GroupNumber_5, PIE_InterruptSource_EQEP2);

	// finally enable the CPU interrupt for PIE group 5 interrupts
	CPU_enableInt(halObj->cpuHandle, CPU_IntNumber_5);
}


interrupt void qep1IndexISR()
{
	genericQepIndexISR(HAL_MTR1);
}


interrupt void qep2IndexISR()
{
	genericQepIndexISR(HAL_MTR2);
}


inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;
	HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

	uint32_t index_posn = QEP_read_posn_index_latch(halMtrObj->qepHandle);
	if (gQepIndexWatchdog[mtrNum].isInitialized) {
		gQepIndexWatchdog[mtrNum].indexError_counts = index_posn - gQepIndexWatchdog[mtrNum].indexPosition_counts;
	} else {
		gQepIndexWatchdog[mtrNum].isInitialized = true;
		gQepIndexWatchdog[mtrNum].indexPosition_counts = index_posn;
	}

	// acknowledge QEP interrupt
	QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);  // for some reason I have to clear *all* flags, not only Iel
	// acknowledge interrupt from PIE group 5
	PIE_clearInt(obj->pieHandle, PIE_GroupNumber_5);
}


bool checkEncoderError(const QepIndexWatchdog_t qiwd)
{
	return abs(qiwd.indexError_counts) > QEP_MAX_INDEX_ERROR;
}

//@} //defgroup
// end of file
