// --COPYRIGHT--,BSD
// Copyright (c) 2015, Texas Instruments Incorporated
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
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

//! \file
//! \brief Dual motor torque control via CAN.
//!
//! (C) Copyright 2015, Texas Instruments, Inc.
//! (C) Copyright 2019, Max Planck Gesellschaft, New York University

/**
 * \defgroup DUAL_MOTOR_TORQUE_CTRL Dual Motor Torque Controller
 *
 * \brief Torque controller for two motors using CAN communication.
 *
 * Uses CAN to receive current commands and to send position, velocity and
 * current measurements.
 *
 *
 * ## Disable CAN
 *
 * By default control commands are received via CAN.  To send commands from CCS
 * via the debugger (e.g. using the GUI), set `gFlag_enableCan = false`.
 *
 *
 * ## Virtual Spring Mode
 *
 * \note CAN needs to be disabled for this mode.
 *
 * To enable the "virtual spring" mode of motor `i`, set
 * `gFlag_enableVirtualSpring[i] = true`.  In this mode the torque is set based
 * on the deflection of the motor from its zero position, thus making it behave
 * like a spring.
 *
 * @{
 */


// **************************************************************************
// the includes

// system includes
#include <amd_motorware_ext/button.h>
#include <amd_motorware_ext/utils.h>
#include <math.h>
#include "canapi.h"
#include "main_2mtr.h"
#include "main_helper.h"
#include "spintac.h"
#include "virtualspring.h"

#ifdef FLASH
#pragma CODE_SECTION(motor1_ISR, "ramfuncs");
#pragma CODE_SECTION(motor2_ISR, "ramfuncs");
#endif

// **************************************************************************
// the defines

// **************************************************************************
// the globals

//! Used for various debugging stuff.
int32_t gFoobar = 0;

#pragma DATA_SECTION(ECanaRegs, "ECanaRegsFile");
volatile struct ECAN_REGS ECanaRegs;

#pragma DATA_SECTION(ECanaMboxes, "ECanaMboxesFile");
volatile struct ECAN_MBOXES ECanaMboxes;

//! \name Objects and Handles
//! \brief State objects and handles for the used software modules
//!
//! Typically each module of the MotorWare library has an object type that
//! stores the state of the object as well as a handle type that is simply a
//! pointer to the object.  When calling functions of the module, the handle is
//! passed so that the function can operate on the current state.
//!
//! For most of the modules there are two object instances, one for each motor
//! (the first one in the array refers to HAL_MTR1, the second one to HAL_MTR2).
//!
//! \{

//! \brief The handles for the hardware abstraction layer for common CPU setup
HAL_Handle halHandle;
//! \brief The hardware abstraction layer object
HAL_Obj hal;

//! \brief The handles for the hardware abstraction layer specific to the motor
//!     board.
HAL_Handle_mtr halHandleMtr[2];
//! \brief The hardware abstraction layer object specific to the motor board.
HAL_Obj_mtr halMtr[2];

//! \brief The handles for the current Clarke transform
CLARKE_Handle clarkeHandle_I[2];
//! \brief The current Clarke transform objects
CLARKE_Obj clarke_I[2];

//! \brief The handles for the current Park transfrom
PARK_Handle parkHandle[2];
//! \brief The current Parke transform objects
PARK_Obj park[2];

//! \brief The handles for the voltage Clarke transform
CLARKE_Handle clarkeHandle_V[2];
//! \brief The voltage Clarke transform objects
CLARKE_Obj clarke_V[2];

//! \brief The handles for the inverse Park transform
IPARK_Handle iparkHandle[2];
//! \brief The inverse Park transform object
IPARK_Obj ipark[2];

//! \brief The handles for the estimator
//!
//! There is no EST_Obj because this is stored in the ROM of the MCU.
EST_Handle estHandle[2];

//! \brief The handles for the PID controllers
//!
//! First dimention: 0 = motor 1, 1 = motor 2
//! Second dimention: 0 = Speed, 1 = Id, 2 = Iq
PID_Handle pidHandle[2][3];
//! \brief The objects for the PID controllers
PID_Obj pid[2][3];

//! \brief The handles for the space vector generator
SVGEN_Handle svgenHandle[2];
//! \brief The space vector generator objects
SVGEN_Obj svgen[2];

//! \brief The handles for the encoder
ENC_Handle encHandle[2];
//! \brief The encoder objects
ENC_Obj enc[2];

//! \brief The handles for the slip compensator
SLIP_Handle slipHandle[2];
//! \brief The slip compensator objects
SLIP_Obj slip[2];

//! \brief The handles for the angle compensation
ANGLE_COMP_Handle angleCompHandle[2];
//! \brief The angle compensation objects
ANGLE_COMP_Obj angleComp[2];

//! \brief The handles for the 3 current and 3 voltage filters for offset
//!     calculation.
FILTER_FO_Handle filterHandle[2][6];
//! \brief the 3-current and 3-voltage filters for offset calculation of each
//!     motor.
FILTER_FO_Obj filter[2][6];

//! \brief The handles for the SpinTAC objects
ST_Handle stHandle[2];
//! \brief The SpinTAC objects
ST_Obj st_obj[2];

//! \brief The handles for the virtual spring objects
VIRTUALSPRING_Handle springHandle[2];
//! \brief The virtual spring objects
VIRTUALSPRING_Obj spring[2];

//! \}

//! \name Counters
//! \brief Counter variables used for decimation and timing of processes
//! \{

//! \brief Count variable to decimate the execution of the high-level controller
uint16_t stCntSpeed[2] = {0, 0};

//! \brief Count variable to decimate the execution of SpinTAC Position
//!     Converter
uint16_t stCntPosConv[2] = {0, 0};

//! \brief Count variable to measure duration of the offset calculation
uint32_t gOffsetCalcCount[2] = {0, 0};

//! \brief Count variable to measure duration of the motor alignment
uint32_t gAlignCount[2] = {0, 0};

//! \}

//! \name Enable Flags
//! \brief Flags to enable/disable parts of the program
//! \{

//! \brief Set this to true to enable the virtual spring mode. One flag for each
//!     motor.
bool gFlag_enableVirtualSpring[2] = {false, false};

//! \brief When true, IqRef is set based on received CAN messages.
//!
//! Set this to false when you want to set current references directly to
//! gMotorVars[i].IqRef_A, for example when using the GUI.  Otherwise manually
//! set values will immediately be overwritten by the value in the CAN mailbox.
bool gFlag_enableCan = true;

//! While set, the current position is stored as offset which is removed from
//! the position before sending it via CAN (i.e. the current position becomes
//! zero).
//! \note This feature is currently disabled!
bool gFlag_resetZeroPositionOffset = false;

//! \brief If true, a rollover of the PosConv module will trigger an error.
//!
//! This is nice for position control applications where the motor is not
//! expected to move far enough to ever observe an rollover.  By throwing an
//! error in the case it happens nonetheless, it is ensured that the system will
//! not explode.
//!
//! \note For applications where rollovers are expected (i.e. motor spinning
//! freely), this should be disabled!
bool gFlag_enablePosRolloverError = false;

//! \}

//! \name Data Variables
//! \brief These variables are used to store data values like measured current,
//!     voltage, reference values in pu, scale factors, etc.
//! \{

//! \brief Contains the pwm values for each phase.
//!
//! -1.0 is 0%, 1.0 is 100%
HAL_PwmData_t gPwmData[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                             {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};

//! \brief Contains three current values, three voltage values and one DC bus
//!     value.
HAL_AdcData_t gAdcData[2];

//! \brief Contains the offsets for the current feedback
MATH_vec3 gOffsets_I_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                              {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};

//! \brief Contains the offsets for the voltage feedback
MATH_vec3 gOffsets_V_pu[2] = {{_IQ(0.0), _IQ(0.0), _IQ(0.0)},
                              {_IQ(0.0), _IQ(0.0), _IQ(0.0)}};

//! \brief Contains the Id and Iq references
MATH_vec2 gIdq_ref_pu[2] = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};

//! \brief Contains the output Vd and Vq from the current controllers
MATH_vec2 gVdq_out_pu[2] = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};

//! \brief Contains the Id and Iq measured values
MATH_vec2 gIdq_pu[2] = {{_IQ(0.0), _IQ(0.0)}, {_IQ(0.0), _IQ(0.0)}};

//! \brief Some scale factor used for torque computation
_iq gTorque_Ls_Id_Iq_pu_to_Nm_sf[2];

//! \brief Some scale factor used for torque computation
_iq gTorque_Flux_Iq_pu_to_Nm_sf[2];

//! \brief Scale factor to convert speed from pu to krpm
_iq gSpeed_pu_to_krpm_sf[2];

//! \brief Scale factor to convert current from A to pu
_iq gCurrent_A_to_pu_sf[2];

//! \brief Offset that is removed from the position before sending it via CAN.
_iq gZeroPositionOffset[2] = {0, 0};

//! \}

//! \brief Decimation factor for the SpinTAC Position Converter
//!
//! Store this to array so it can be used in generic_motor_ISR.
const uint16_t gNumIsrTicksPerPosConvTick[2] = {ISR_TICKS_PER_POSCONV_TICK,
                                                ISR_TICKS_PER_POSCONV_TICK_2};

//! \brief User Parameters
//!
//! Contains parameters from the user*.h config files.
USER_Params gUserParams[2];

//! \brief Global motor variables
//!
//! Several status information about the motors is stored here so they can be
//! accessed from the debugger or the GUI.
volatile MOTOR_Vars_t gMotorVars[2] = {MOTOR_Vars_INIT_Mtr1,
                                       MOTOR_Vars_INIT_Mtr2};

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

//! Timestamp based on timer 0 (increased by one at each timer interrupt).
uint32_t gTimer0_stamp = 0;

//! Last time the blinking status LED was toggled (based on gTimer0_stamp).
uint32_t gStatusLedBlinkLastToggleTime = 0;

//! Last time a status message was sent via CAN (based on gTimer0_stamp).
uint32_t gCanLastStatusMsgTime = 0;

//! Last time a IqRef message was received via CAN (based on gTimer0_stamp).
uint32_t gCanLastReceivedIqRef_stamp = 0;

//! Timeout for incoming IqRef messages.  If exceeded, error is set.  To disable
//! timeout, set to 0.
uint32_t gCanReceiveIqRefTimeout = 0;

//! Mask for the CAN mailboxes. Only mailboxes that are enabled in the mask are
//! allowed to send.
uint32_t gEnabledCanMessages = 0;

//! Set to true when the CAN module is currently aborting a message that was not
//! acknowledged in time.
bool gCanAbortingMessages = false;

//! Errors that occured in the system.  gErrors.all == 0 if no errors occured.
Error_t gErrors;

//! QEP index watchdog data for both encoders.
QepIndexWatchdog_t gQepIndexWatchdog[2] = {
    {.isInitialized = false, .indexError_counts = 0},
    {.isInitialized = false, .indexError_counts = 0}};

// **************************************************************************
// the functions

// Little helper function
inline void setCanMboxStatus(const uint32_t mbox, const uint32_t status) {
    if (status) {
        gEnabledCanMessages |= mbox;
    } else {
        gEnabledCanMessages &= ~mbox;
    }
}

void main(void) {
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
    // To allow fast movement with lots-of-lines-encoders, the sampling period
    // of the GPIO qualification filter has to be reduced (otherwise encoder
    // pulses get rejected as noise).  The following lines overwrite the
    // settings done in HAL_setupGpio() (hal.c).
    // "period = 11" results in actual sampling period 11*2*(1/90MHz) = 0.24us
    // Note: Setting the period is done for blocks of GPIO pins.
    //
    // GPIO 16-23 (covering eQEP1)
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_16,
                                11);  // GPIO16-23
    // GPIO 50-55 and 56-58 (covering eQEP2)
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_50,
                                11);  // GPIO50-55
    GPIO_setQualificationPeriod(hal.gpioHandle, GPIO_Number_56,
                                11);  // GPIO56-58

    // Overwrite the settings for timer0 (we want it faster)
    uint32_t timerPeriod_cnts =
        ((uint32_t)gUserParams[0].systemFreq_MHz * 1e6l) / TIMER0_FREQ_Hz - 1;
    overwriteSetupTimer0(halHandle, timerPeriod_cnts);

    // initialize the estimator
    estHandle[HAL_MTR1] = EST_init((void *)USER_EST_HANDLE_ADDRESS, 0x200);
    estHandle[HAL_MTR2] = EST_init((void *)USER_EST_HANDLE_ADDRESS_1, 0x200);

    {
        uint_least8_t mtrNum;

        for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {
            // initialize the individual motor hal files
            halHandleMtr[mtrNum] =
                HAL_init_mtr(&halMtr[mtrNum], sizeof(halMtr[mtrNum]),
                             (HAL_MtrSelect_e)mtrNum);

            // Setup each motor board to its specific setting
            HAL_setParamsMtr(halHandleMtr[mtrNum], halHandle,
                             &gUserParams[mtrNum]);

            {
                // These function calls are used to initialize the estimator
                // with ROM function calls. It needs the specific address where
                // the controller object is declared by the ROM code.
                CTRL_Handle ctrlHandle =
                    CTRL_init((void *)USER_CTRL_HANDLE_ADDRESS, 0x200);
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

            // Compensates for the delay introduced
            // from the time when the system inputs are sampled to when the PWM
            // voltages are applied to the motor windings.
            angleCompHandle[mtrNum] =
                ANGLE_COMP_init(&angleComp[mtrNum], sizeof(angleComp[mtrNum]));
            ANGLE_COMP_setParams(angleCompHandle[mtrNum],
                                 gUserParams[mtrNum].iqFullScaleFreq_Hz,
                                 gUserParams[mtrNum].pwmPeriod_usec,
                                 gUserParams[mtrNum].numPwmTicksPerIsrTick);

            // initialize the Clarke modules
            // Clarke handle initialization for current signals
            clarkeHandle_I[mtrNum] =
                CLARKE_init(&clarke_I[mtrNum], sizeof(clarke_I[mtrNum]));

            // Clarke handle initialization for voltage signals
            clarkeHandle_V[mtrNum] =
                CLARKE_init(&clarke_V[mtrNum], sizeof(clarke_V[mtrNum]));

            // Park handle initialization for current signals
            parkHandle[mtrNum] = PARK_init(&park[mtrNum], sizeof(park[mtrNum]));

            //*** compute scaling factors

            gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum] =
                USER_computeTorque_Ls_Id_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);

            gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum] =
                USER_computeTorque_Flux_Iq_pu_to_Nm_sf(&gUserParams[mtrNum]);

            gSpeed_pu_to_krpm_sf[mtrNum] =
                _IQ((gUserParams[mtrNum].iqFullScaleFreq_Hz * 60.0) /
                    ((float_t)gUserParams[mtrNum].motor_numPolePairs * 1000.0));

            gCurrent_A_to_pu_sf[mtrNum] =
                _IQ(1.0 / gUserParams[mtrNum].iqFullScaleCurrent_A);

            // disable Rs recalculation
            EST_setFlag_enableRsRecalc(estHandle[mtrNum], false);

            // set the number of current sensors
            setupClarke_I(clarkeHandle_I[mtrNum],
                          gUserParams[mtrNum].numCurrentSensors);

            // set the number of voltage sensors
            setupClarke_V(clarkeHandle_V[mtrNum],
                          gUserParams[mtrNum].numVoltageSensors);

            // initialize the PID controllers

            // There are two PI controllers, one for Iq and one for Id.
            // This is for the Id current controller
            pidHandle[mtrNum][1] =
                PID_init(&pid[mtrNum][1], sizeof(pid[mtrNum][1]));
            // This is for the Iq current controller
            pidHandle[mtrNum][2] =
                PID_init(&pid[mtrNum][2], sizeof(pid[mtrNum][2]));
            // This sets up both controllers
            pidSetup(pidHandle[mtrNum], gUserParams[mtrNum]);

            // initialize the inverse Park module
            iparkHandle[mtrNum] =
                IPARK_init(&ipark[mtrNum], sizeof(ipark[mtrNum]));

            // initialize the space vector generator module
            svgenHandle[mtrNum] =
                SVGEN_init(&svgen[mtrNum], sizeof(svgen[mtrNum]));

            // initialize and configure offsets using filters
            {
                uint16_t cnt = 0;
                _iq b0 = _IQ(gUserParams[mtrNum].offsetPole_rps /
                             (float_t)gUserParams[mtrNum].ctrlFreq_Hz);
                _iq a1 = (b0 - _IQ(1.0));
                _iq b1 = _IQ(0.0);

                for (cnt = 0; cnt < 6; cnt++) {
                    filterHandle[mtrNum][cnt] = FILTER_FO_init(
                        &filter[mtrNum][cnt], sizeof(filter[mtrNum][0]));
                    FILTER_FO_setDenCoeffs(filterHandle[mtrNum][cnt], a1);
                    FILTER_FO_setNumCoeffs(filterHandle[mtrNum][cnt], b0, b1);
                    FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt],
                                                   _IQ(0.0), _IQ(0.0));
                }

                gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
            }

            // initialize the encoder module
            encHandle[mtrNum] = ENC_init(&enc[mtrNum], sizeof(enc[mtrNum]));

            // initialize the slip compensation module
            slipHandle[mtrNum] = SLIP_init(&slip[mtrNum], sizeof(slip[mtrNum]));
            // setup the SLIP module
            SLIP_setup(slipHandle[mtrNum],
                       _IQ(gUserParams[mtrNum].ctrlPeriod_sec));

            // setup faults
            HAL_setupFaults(halHandleMtr[mtrNum]);

            // initialize the SpinTAC Components
            stHandle[mtrNum] = ST_init(&st_obj[mtrNum], sizeof(st_obj[mtrNum]));

            // init virtual spring handle
            springHandle[mtrNum] =
                VIRTUALSPRING_init(&spring[mtrNum], sizeof(spring[mtrNum]));
            VIRTUALSPRING_setup(
                springHandle[mtrNum], 10, _IQ(2.0),
                STPOSCONV_getMRevMaximum_mrev(st_obj[mtrNum].posConvHandle));

        }  // End of for loop
    }

    // setup the encoder module
    ENC_setup(encHandle[HAL_MTR1], 1, USER_MOTOR_NUM_POLE_PAIRS,
              USER_MOTOR_ENCODER_LINES, 0, USER_IQ_FULL_SCALE_FREQ_Hz,
              USER_ISR_FREQ_Hz, 8000.0);
    ENC_setup(encHandle[HAL_MTR2], 1, USER_MOTOR_NUM_POLE_PAIRS_2,
              USER_MOTOR_ENCODER_LINES_2, 0, USER_IQ_FULL_SCALE_FREQ_Hz_2,
              USER_ISR_FREQ_Hz_2, 8000.0);

    // setup encoder index interrupts
    setupQepIndexInterrupt(halHandle, halHandleMtr, &qep1IndexISR,
                           &qep2IndexISR);

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
    PIE_registerTimer0IntHandler(hal.pieHandle, &timer0_ISR);

    // disable the PWM
    HAL_disablePwm(halHandleMtr[HAL_MTR1]);
    HAL_disablePwm(halHandleMtr[HAL_MTR2]);

    // reconfigure GPIO pins for LEDs and button
    HAL_overwriteSetupGpio(halHandle);

    // Setup everything related to CAN communication
    setupCan(halHandle, &can1_ISR);

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

    // Begin the background loop
    for (;;) {
        // Waiting for enable system flag to be set
        // Motor 1 Flag_enableSys is the master control.
        while (!(gMotorVars[HAL_MTR1].Flag_enableSys)) {
            LED_run(halHandle);

            // checkErrors();
            maybeSendCanStatusMsg();
        }

        // loop while the enable system flag is true
        // Motor 1 Flag_enableSys is the master control.
        while (gMotorVars[HAL_MTR1].Flag_enableSys) {
            uint_least8_t mtrNum = HAL_MTR1;

            /*** Error Checks ***/
            checkErrors();

            LED_run(halHandle);

            // When there is an error, shut down the system to be safe
            if (gErrors.all) {
                gMotorVars[HAL_MTR1].Flag_enableSys = false;
                break;  // immediately exit the enabled == true loop
            }

            // Send status message via CAN
            maybeSendCanStatusMsg();

            // Set the position reset flag via button on a GPIO
            // gFlag_resetZeroPositionOffset = BUTTON_isPressed(hal.gpioHandle);
            // We don't really need the position offset. Rather use the button
            // as a soft emergency stop (i.e. disable system)
            if (BUTTON_isPressed(hal.gpioHandle)) {
                gMotorVars[HAL_MTR1].Flag_enableSys = false;
            }

            for (mtrNum = HAL_MTR1; mtrNum <= HAL_MTR2; mtrNum++) {
                // If the flag is set, set current position as zero offset
                // if (gFlag_resetZeroPositionOffset) {
                //  gZeroPositionOffset[mtrNum] =
                //STPOSCONV_getPosition_mrev(st_obj[mtrNum].posConvHandle);
                //}

                // If Flag_enableSys is set AND Flag_Run_Identify is set THEN
                // enable PWMs and set the speed reference
                if (gMotorVars[mtrNum].Flag_Run_Identify) {
                    bool vspringChanged;

                    // update estimator state
                    EST_updateState(estHandle[mtrNum], 0);

#ifdef FAST_ROM_V1p6
                    // call this function to fix 1p6. This is only used for
                    // F2806xF/M implementation of InstaSPIN (version 1.6 of
                    // ROM), since the inductance calculation is not done
                    // correctly in ROM, so this function fixes that ROM bug.
                    softwareUpdate1p6(estHandle[mtrNum], &gUserParams[mtrNum]);
#endif

                    // Update status of the virtual spring
                    vspringChanged = VIRTUALSPRING_setEnabled(
                        springHandle[mtrNum],
                        gFlag_enableVirtualSpring[mtrNum]);
                    // ...and make some adjustments if it changed
                    if (vspringChanged) {
                        if (VIRTUALSPRING_isEnabled(springHandle[mtrNum])) {
                            // if virtual spring mode is just activated, reset
                            // position offset
                            VIRTUALSPRING_scheduleResetOffset(
                                springHandle[mtrNum]);
                        } else {
                            // if it is just disabled, set IqRef to 0
                            gMotorVars[mtrNum].IqRef_A = 0;
                        }
                    }

                    // enable the PWM
                    HAL_enablePwm(halHandleMtr[mtrNum]);
                } else  // Flag_enableSys is set AND Flag_Run_Identify is not
                        // set
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
                }

                // update the global variables
                updateGlobalVariables(mtrNum);

                // enable/disable the forced angle
                EST_setFlag_enableForceAngle(
                    estHandle[mtrNum],
                    gMotorVars[mtrNum].Flag_enableForceAngle);

#ifdef DRV8301_SPI
                HAL_writeDrvData(halHandleMtr[mtrNum],
                                 &gDrvSpi8301Vars[mtrNum]);
                HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8301Vars[mtrNum]);
#endif
#ifdef DRV8305_SPI
                HAL_writeDrvData(halHandleMtr[mtrNum],
                                 &gDrvSpi8305Vars[mtrNum]);
                HAL_readDrvData(halHandleMtr[mtrNum], &gDrvSpi8305Vars[mtrNum]);
#endif

            }  // end of for loop
        }      // end of while(gFlag_enableSys) loop

        // disable the PWM
        HAL_disablePwm(halHandleMtr[HAL_MTR1]);
        HAL_disablePwm(halHandleMtr[HAL_MTR2]);

        gMotorVars[HAL_MTR1].Flag_Run_Identify = false;
        gMotorVars[HAL_MTR2].Flag_Run_Identify = false;

    }  // end of for(;;) loop
}  // end of main() function

//! \brief     The main ISR that implements the motor control.
interrupt void motor1_ISR(void) {
    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_1);

    generic_motor_ISR(HAL_MTR1);

    return;
}  // end of motor1_ISR() function

interrupt void motor2_ISR(void) {
    // acknowledge the ADC interrupt
    HAL_acqAdcInt(halHandle, ADC_IntNumber_2);

    generic_motor_ISR(HAL_MTR2);

    return;
}  // end of motor2_ISR() function

void generic_motor_ISR(const HAL_MtrSelect_e mtrNum) {
    // Declaration of local variables
    static _iq angle_pu[2] = {_IQ(0.0), _IQ(0.0)};
    _iq speed_pu = _IQ(0.0);
    _iq oneOverDcBus;
    MATH_vec2 Iab_pu;
    MATH_vec2 Vab_pu;
    MATH_vec2 phasor;

    // convert the ADC data
    HAL_readAdcDataWithOffsets(halHandle, halHandleMtr[mtrNum],
                               &gAdcData[mtrNum]);

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
    ENC_calcElecAngle(encHandle[mtrNum],
                      HAL_getQepPosnCounts(halHandleMtr[mtrNum]));

    if (++stCntPosConv[mtrNum] >= gNumIsrTicksPerPosConvTick[mtrNum]) {
        stCntPosConv[mtrNum] = 0;
        // Calculate the feedback speed
        ST_runPosConv(stHandle[mtrNum], encHandle[mtrNum], slipHandle[mtrNum],
                      &gIdq_pu[mtrNum], gUserParams[mtrNum].motor_type);
    }

    // run the appropriate controller
    if (gMotorVars[mtrNum].Flag_Run_Identify) {
        // Declaration of local variables.
        _iq refValue;
        _iq fbackValue;
        _iq outMax_pu;

        // check if the motor should be forced into encoder alignment
        if (gMotorVars[mtrNum].Flag_enableAlignment == false) {
            // When appropriate, update the current reference.
            // This mechanism provides the decimation for the upper level
            // control loop.
            if (++stCntSpeed[mtrNum] >=
                gUserParams[mtrNum].numCtrlTicksPerSpeedTick) {
                // Reset the Speed execution counter.
                stCntSpeed[mtrNum] = 0;

                // If spring is enabled, set IqRef based on it
                if (VIRTUALSPRING_isEnabled(springHandle[mtrNum])) {
                    VIRTUALSPRING_run(springHandle[mtrNum],
                                      STPOSCONV_getPosition_mrev(
                                          st_obj[mtrNum].posConvHandle));
                    gMotorVars[mtrNum].IqRef_A =
                        VIRTUALSPRING_getIqRef_A(springHandle[mtrNum]);
                } else if (gFlag_enableCan) {
                    gMotorVars[mtrNum].IqRef_A = CAN_getIqRef(mtrNum);
                }
                // else: do nothing. This allows setting IqRef_A from a GUI or
                // debug session.

                gIdq_ref_pu[mtrNum].value[0] = _IQmpy(
                    gMotorVars[mtrNum].IdRef_A, gCurrent_A_to_pu_sf[mtrNum]);
                gIdq_ref_pu[mtrNum].value[1] = _IQmpy(
                    gMotorVars[mtrNum].IqRef_A, gCurrent_A_to_pu_sf[mtrNum]);
            }

            // generate the motor electrical angle
            if (gUserParams[mtrNum].motor_type == MOTOR_Type_Induction) {
                // update the electrical angle for the SLIP module
                SLIP_setElectricalAngle(slipHandle[mtrNum],
                                        ENC_getElecAngle(encHandle[mtrNum]));
                // compute the amount of slip
                SLIP_run(slipHandle[mtrNum]);
                // set magnetic angle
                angle_pu[mtrNum] = SLIP_getMagneticAngle(slipHandle[mtrNum]);
            } else {
                angle_pu[mtrNum] = ENC_getElecAngle(encHandle[mtrNum]);
            }

            speed_pu = STPOSCONV_getVelocity(st_obj[mtrNum].posConvHandle);
        } else {  // the alignment procedure is in effect

            // force motor angle and speed to 0
            angle_pu[mtrNum] = _IQ(0.0);
            speed_pu = _IQ(0.0);

            // set D-axis current to Rs estimation current
            gIdq_ref_pu[mtrNum].value[0] =
                _IQmpy(_IQ(gUserParams[mtrNum].maxCurrent_resEst),
                       gCurrent_A_to_pu_sf[mtrNum]);
            // set Q-axis current to 0
            gIdq_ref_pu[mtrNum].value[1] = _IQ(0.0);

            // save encoder reading when forcing motor into alignment
            if (gUserParams[mtrNum].motor_type == MOTOR_Type_Pm) {
                ENC_setZeroOffset(
                    encHandle[mtrNum],
                    (uint32_t)(HAL_getQepPosnMaximum(halHandleMtr[mtrNum]) -
                               HAL_getQepPosnCounts(halHandleMtr[mtrNum])));
            }

            // if alignment counter exceeds threshold, exit alignment
            if (gAlignCount[mtrNum]++ >=
                gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
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
        PID_run(pidHandle[mtrNum][1], refValue, fbackValue,
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
        outMax_pu = _IQsqrt(
            _IQ(gUserParams[mtrNum].maxVsMag_pu *
                gUserParams[mtrNum].maxVsMag_pu) -
            _IQmpy(gVdq_out_pu[mtrNum].value[0], gVdq_out_pu[mtrNum].value[0]));

        // Set the limits to +/- outMax_pu
        PID_setMinMax(pidHandle[mtrNum][2], -outMax_pu, outMax_pu);

        // The next instruction executes the PI current controller for the
        // q axis and places its output in Vdq_pu.value[1], which is the
        // control voltage vector along the q-axis (Vq)
        PID_run(pidHandle[mtrNum][2], refValue, fbackValue,
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
    } else if (gMotorVars[mtrNum].Flag_enableOffsetcalc == true) {
        runOffsetsCalculation(mtrNum);
    } else  // gMotorVars.Flag_Run_Identify = 0
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

interrupt void can1_ISR() {
    // The same ISR is used by the eCAN module, independent of the source of the
    // interrupt.  This means, we have to distinguish the various cases here,
    // based on the values of certain registers (see SPRUH18f, section 16.13)

    // This ISR is used by GIF1

    // NOTE: SPRU074F, sec. 3.4.3.2 describes how to correctly handle all cases
    // (I don't fully understand what is meant by a "half-word read", though).

    // Since this ISR is currently only used for mailbox 0 receives, we only
    // check for this here and simply ignore other cases that call this ISR
    // (there shouldn't be any).
    // Note: ECanaRegs.CANGIF1.bit.MIV1 contains the number of the mailbox that
    // caused this interrupt (this should always be 0 for now).
    if (CAN_checkReceivedMessagePending(CAN_MBOX_IN_COMMANDS)) {
        CAN_Command_t cmd = CAN_getCommand();

        switch (cmd.id) {
            case CAN_CMD_ENABLE_SYS:  // enable system
                gMotorVars[HAL_MTR1].Flag_enableSys = cmd.value;
                break;
            case CAN_CMD_ENABLE_MTR1:  // run motor 1
                gMotorVars[HAL_MTR1].Flag_Run_Identify = cmd.value;
                break;
            case CAN_CMD_ENABLE_MTR2:  // run motor 2
                gMotorVars[HAL_MTR2].Flag_Run_Identify = cmd.value;
                break;
            case CAN_CMD_ENABLE_VSPRING1:  // motor 1 enable spring
                spring[HAL_MTR1].enabled = cmd.value;
                break;
            case CAN_CMD_ENABLE_VSPRING2:  // motor 2 enable spring
                spring[HAL_MTR2].enabled = cmd.value;
                break;

            case CAN_CMD_SEND_CURRENT:
                setCanMboxStatus(CAN_MBOX_OUT_Iq, cmd.value);
                break;
            case CAN_CMD_SEND_POSITION:
                setCanMboxStatus(CAN_MBOX_OUT_ENC_POS, cmd.value);
                break;
            case CAN_CMD_SEND_VELOCITY:
                setCanMboxStatus(CAN_MBOX_OUT_SPEED, cmd.value);
                break;
            case CAN_CMD_SEND_ADC6:
                setCanMboxStatus(CAN_MBOX_OUT_ADC6, cmd.value);
                break;
            case CAN_CMD_SEND_ENC_INDEX:
                setCanMboxStatus(CAN_MBOX_OUT_ENC_INDEX, cmd.value);
                break;
            case CAN_CMD_SEND_ALL:
                if (cmd.value) {
                    gEnabledCanMessages =
                        (CAN_MBOX_OUT_Iq | CAN_MBOX_OUT_ENC_POS |
                         CAN_MBOX_OUT_SPEED | CAN_MBOX_OUT_ADC6 |
                         CAN_MBOX_OUT_ENC_INDEX);
                } else {
                    gEnabledCanMessages = 0;
                }
                break;
            case CAN_CMD_SET_CAN_RECV_TIMEOUT:
                gCanReceiveIqRefTimeout = cmd.value;
                break;
            case CAN_CMD_ENABLE_POS_ROLLOVER_ERROR:
                gFlag_enablePosRolloverError = cmd.value;
        }

        // Acknowledge interrupt
        CAN_clearReceivedMessagePending(CAN_MBOX_IN_COMMANDS);
    }

    // acknowledge interrupt from PIE
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_9);
}

interrupt void timer0_ISR() {
    ++gTimer0_stamp;

    uint32_t mbox_mask = gEnabledCanMessages;

    // Encoder Index messages are not sent here but in the index ISR. Exclude it
    // from the mask.
    mbox_mask &= ~CAN_MBOX_OUT_ENC_INDEX;

    // If no mailbox is enabled, skip this (mostly to not disturb CAN error
    // detection (gCanAbortingMessages would always be set to false when nothing
    // is sent here, independent of errors with the status message).
    if (mbox_mask != 0) {
        // TODO: better abortion handling
        // If there is still an old message waiting for transmission, abort it
        if (CAN_checkTransmissionPending(mbox_mask)) {
            gCanAbortingMessages = true;
            CAN_abort(mbox_mask);
            // is it okay to block here (or at least wait for a while)?
        } else {
            gCanAbortingMessages = false;
        }

        setCanMotorData(HAL_MTR1);
        setCanMotorData(HAL_MTR2);

        CAN_setAdcIn6Values(HAL_readAdcResult(halHandle, POTI_RESULT1),
                            HAL_readAdcResult(halHandle, POTI_RESULT2));

        // send messages via CAN
        CAN_send(mbox_mask);
    }

    // acknowledge interrupt
    HAL_acqTimer0Int(halHandle);
}

void runOffsetsCalculation(HAL_MtrSelect_e mtrNum) {
    uint16_t cnt;

    // enable the PWM
    HAL_enablePwm(halHandleMtr[mtrNum]);

    for (cnt = 0; cnt < 3; cnt++) {
        // Set the PWMs to 50% duty cycle
        gPwmData[mtrNum].Tabc.value[cnt] = _IQ(0.0);

        // reset offsets used
        gOffsets_I_pu[mtrNum].value[cnt] = _IQ(0.0);
        gOffsets_V_pu[mtrNum].value[cnt] = _IQ(0.0);

        // run offset estimation
        FILTER_FO_run(filterHandle[mtrNum][cnt], gAdcData[mtrNum].I.value[cnt]);
        FILTER_FO_run(filterHandle[mtrNum][cnt + 3],
                      gAdcData[mtrNum].V.value[cnt]);
    }

    if (gOffsetCalcCount[mtrNum]++ >=
        gUserParams[mtrNum].ctrlWaitTime[CTRL_State_OffLine]) {
        gMotorVars[mtrNum].Flag_enableOffsetcalc = false;
        gOffsetCalcCount[mtrNum] = 0;

        for (cnt = 0; cnt < 3; cnt++) {
            // get calculated offsets from filter
            gOffsets_I_pu[mtrNum].value[cnt] =
                FILTER_FO_get_y1(filterHandle[mtrNum][cnt]);
            gOffsets_V_pu[mtrNum].value[cnt] =
                FILTER_FO_get_y1(filterHandle[mtrNum][cnt + 3]);

            // clear filters
            FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt], _IQ(0.0),
                                           _IQ(0.0));
            FILTER_FO_setInitialConditions(filterHandle[mtrNum][cnt + 3],
                                           _IQ(0.0), _IQ(0.0));
        }
    }

    return;
}  // end of runOffsetsCalculation() function

//! \brief     Update the global variables (gMotorVars).
//! \param[in] handle  The estimator (EST) handle
void updateGlobalVariables(const uint_least8_t mtrNum) {
    // get the speed estimate
    gMotorVars[mtrNum].Speed_krpm =
        _IQmpy(STPOSCONV_getVelocityFiltered(st_obj[mtrNum].posConvHandle),
               gSpeed_pu_to_krpm_sf[mtrNum]);

    // Get the DC buss voltage
    gMotorVars[mtrNum].VdcBus_kV =
        _IQmpy(gAdcData[mtrNum].dcBus,
               _IQ(gUserParams[mtrNum].iqFullScaleVoltage_V / 1000.0));

    // read Vd and Vq vectors per units
    gMotorVars[mtrNum].Vd = gVdq_out_pu[mtrNum].value[0];
    gMotorVars[mtrNum].Vq = gVdq_out_pu[mtrNum].value[1];

    // calculate vector Vs in per units: (Vs = sqrt(Vd^2 + Vq^2))
    gMotorVars[mtrNum].Vs =
        _IQsqrt(_IQmpy(gMotorVars[mtrNum].Vd, gMotorVars[mtrNum].Vd) +
                _IQmpy(gMotorVars[mtrNum].Vq, gMotorVars[mtrNum].Vq));

    // read Id and Iq vectors in amps
    gMotorVars[mtrNum].Id_A =
        _IQmpy(gIdq_pu[mtrNum].value[0],
               _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));
    gMotorVars[mtrNum].Iq_A =
        _IQmpy(gIdq_pu[mtrNum].value[1],
               _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));

    // calculate vector Is in amps:  (Is_A = sqrt(Id_A^2 + Iq_A^2))
    gMotorVars[mtrNum].Is_A =
        _IQsqrt(_IQmpy(gMotorVars[mtrNum].Id_A, gMotorVars[mtrNum].Id_A) +
                _IQmpy(gMotorVars[mtrNum].Iq_A, gMotorVars[mtrNum].Iq_A));

    // get the torque estimate
    gMotorVars[mtrNum].Torque_Nm = UTILS_computeTorque_Nm(
        estHandle[mtrNum], gIdq_pu[mtrNum], gTorque_Flux_Iq_pu_to_Nm_sf[mtrNum],
        gTorque_Ls_Id_Iq_pu_to_Nm_sf[mtrNum]);

    // get the Position Converter error
    gMotorVars[mtrNum].SpinTAC.PosConvErrorID =
        STPOSCONV_getErrorID(st_obj[mtrNum].posConvHandle);

    return;
}  // end of updateGlobalVariables() function

void setCanStatusMsg() {
    CAN_StatusMsg_t status;

    // Send status message via CAN
    status.all = 0;
    status.bit.system_enabled = gMotorVars[HAL_MTR1].Flag_enableSys;
    status.bit.motor1_enabled = gMotorVars[HAL_MTR1].Flag_Run_Identify;
    status.bit.motor1_ready = !gMotorVars[HAL_MTR1].Flag_enableAlignment;
    status.bit.motor2_enabled = gMotorVars[HAL_MTR2].Flag_Run_Identify;
    status.bit.motor2_ready = !gMotorVars[HAL_MTR2].Flag_enableAlignment;
    if (gErrors.bit.qep_error) {
        status.bit.error_code = CAN_ERROR_ENCODER;
    } else if (gErrors.bit.can_error) {
        // There is not really a point in reporting this error, the message
        // most likely won't come through.  So do it here for completeness but
        // don't waste a separate error code on this.
        status.bit.error_code = CAN_ERROR_OTHER;
    } else if (gErrors.bit.can_recv_timeout) {
        status.bit.error_code = CAN_ERROR_CAN_RECV_TIMEOUT;
    } else if (gErrors.bit.posconv_error) {
        status.bit.error_code = CAN_ERROR_POSCONV;
    } else if (gErrors.bit.pos_rollover) {
        status.bit.error_code = CAN_ERROR_POS_ROLLOVER;
    } else {
        status.bit.error_code = CAN_ERROR_NO_ERROR;
    }

    CAN_setStatusMsg(status);
}

void setCanMotorData(const HAL_MtrSelect_e mtrNum) {
    _iq current_iq, position, speed;
    ST_Obj *st = (ST_Obj *)stHandle[mtrNum];

    // take last current measurement and convert to Ampere
    current_iq = _IQmpy(gIdq_pu[mtrNum].value[1],
                        _IQ(gUserParams[mtrNum].iqFullScaleCurrent_A));

    // take the current position
    position = STPOSCONV_getPosition_mrev(st->posConvHandle);
    // remove zero position offset (feature disabled)
    //_iq mrev_rollover = STPOSCONV_getMRevMaximum_mrev(st->posConvHandle);
    // position = removePositionOffset(position, gZeroPositionOffset[mtrNum],
    //      mrev_rollover);

    // take current velocity and convert to krpm
    speed = _IQmpy(STPOSCONV_getVelocityFiltered(st->posConvHandle),
                   gSpeed_pu_to_krpm_sf[mtrNum]);

    if (mtrNum == HAL_MTR1) {
        CAN_setDataMotor1(current_iq, position, speed);
    } else {
        CAN_setDataMotor2(current_iq, position, speed);
    }

    return;
}

void maybeSendCanStatusMsg() {
    if (gCanLastStatusMsgTime <
        (gTimer0_stamp - TIMER0_FREQ_Hz / CAN_STATUSMSG_TRANS_FREQ_Hz)) {
        // If there is still an old message waiting for transmission, abort it
        if (CAN_checkTransmissionPending(CAN_MBOX_OUT_STATUSMSG)) {
            gCanAbortingMessages = true;
            CAN_abort(CAN_MBOX_OUT_STATUSMSG);
        } else {
            gCanAbortingMessages = false;
        }

        setCanStatusMsg();
        CAN_send(CAN_MBOX_OUT_STATUSMSG);

        gCanLastStatusMsgTime = gTimer0_stamp;
    }
}

interrupt void qep1IndexISR() { genericQepIndexISR(HAL_MTR1); }

interrupt void qep2IndexISR() { genericQepIndexISR(HAL_MTR2); }

inline void genericQepIndexISR(const HAL_MtrSelect_e mtrNum) {
    HAL_Obj *obj = (HAL_Obj *)halHandle;
    HAL_Obj_mtr *halMtrObj = (HAL_Obj_mtr *)halHandleMtr[mtrNum];

    uint32_t index_posn = QEP_read_posn_index_latch(halMtrObj->qepHandle);

    if (gEnabledCanMessages & CAN_MBOX_OUT_ENC_INDEX) {
        // Convert index position from counts to mrev by dividing by the max.
        // number of counts.
        // QEP_Obj *qep = (QEP_Obj *)halMtrObj->qepHandle;
        //_iq index_pos_mrev = _IQ((float_t) index_posn / qep->QPOSMAX);

        // NOTE: The above led to slighly varying results when compared to
        // PosConv (maybe because PosConv does some adjustment when aligning
        // motors?). Just using the current PosConv position is theoretically
        // not as precise but led to much better results in practice.
        _iq index_pos_mrev =
            STPOSCONV_getPosition_mrev(st_obj[mtrNum].posConvHandle);
        CAN_setEncoderIndex(mtrNum, index_pos_mrev);
        CAN_send(CAN_MBOX_OUT_ENC_INDEX);
    }

    // Compute position error compared to first index
    if (gQepIndexWatchdog[mtrNum].isInitialized) {
        gQepIndexWatchdog[mtrNum].indexError_counts =
            index_posn - gQepIndexWatchdog[mtrNum].indexPosition_counts;
    } else {
        gQepIndexWatchdog[mtrNum].isInitialized = true;
        gQepIndexWatchdog[mtrNum].indexPosition_counts = index_posn;
    }

    // acknowledge QEP interrupt
    // for some reason I have to clear *all* flags, not only Iel
    QEP_clear_all_interrupt_flags(halMtrObj->qepHandle);
    // acknowledge interrupt from PIE group 5
    PIE_clearInt(obj->pieHandle, PIE_GroupNumber_5);
}

bool checkEncoderError(const QepIndexWatchdog_t qiwd) {
    return abs(qiwd.indexError_counts) > QEP_MAX_INDEX_ERROR;
}

void checkErrors() {
    //*** CAN Timout

    // If new IqRef message was received via CAN, store the current time, so we
    // can detect connection loss.
    if (CAN_checkAndClearRMP(CAN_MBOX_IN_IqRef)) {
        gCanLastReceivedIqRef_stamp = gTimer0_stamp;
    }

    gErrors.bit.can_recv_timeout =
        (gFlag_enableCan                  // only check if CAN is enabled
         && gCanReceiveIqRefTimeout != 0  // and timeout is enabled
         // check if one of the motors is enabled and has a IqRef != 0
         && ((gMotorVars[HAL_MTR1].Flag_Run_Identify &&
              gMotorVars[HAL_MTR1].IqRef_A != 0) ||
             (gMotorVars[HAL_MTR2].Flag_Run_Identify &&
              gMotorVars[HAL_MTR2].IqRef_A != 0))
         // finally check if last message exceeds timeout
         && (gCanLastReceivedIqRef_stamp <
             gTimer0_stamp - gCanReceiveIqRefTimeout));

    //*** Encoder Error
    gErrors.bit.qep_error = (checkEncoderError(gQepIndexWatchdog[0]) ||
                             checkEncoderError(gQepIndexWatchdog[1]));

    //*** POSCONV error
    gErrors.bit.posconv_error =
        ((STPOSCONV_getErrorID(st_obj[HAL_MTR1].posConvHandle) != 0) ||
         (STPOSCONV_getErrorID(st_obj[HAL_MTR2].posConvHandle) != 0));

    //*** Position Rollover (only check if enabled)
    gErrors.bit.pos_rollover =
        gFlag_enablePosRolloverError &&
        ((STPOSCONV_getPositionRollOver(st_obj[HAL_MTR1].posConvHandle) != 0) ||
         (STPOSCONV_getPositionRollOver(st_obj[HAL_MTR2].posConvHandle) != 0));
}

void LED_run(HAL_Handle halHandle) {
    //*** GREEN/BLUE LED
    // Off = system disabled
    // On = system enabled
    // Slow Blinking = motor enabled
    // Fast Blinking = aligning motor
    if (gMotorVars[0].Flag_enableSys) {
        // Show system and motor status using the blue LED
        if (gMotorVars[0].Flag_Run_Identify ||
            gMotorVars[1].Flag_Run_Identify) {
            uint32_t blink_duration = TIMER0_FREQ_Hz / LED_BLINK_FREQ_Hz;

            // blink faster while motors are aligned
            if ((gMotorVars[0].Flag_Run_Identify &&
                 gMotorVars[0].Flag_enableAlignment) ||
                (gMotorVars[1].Flag_Run_Identify &&
                 gMotorVars[1].Flag_enableAlignment)) {
                blink_duration /= 4;
            }

            // toggle status LED
            if (gStatusLedBlinkLastToggleTime <
                (gTimer0_stamp - blink_duration)) {
                HAL_toggleLed(halHandle, LED_ONBOARD_BLUE);
                HAL_toggleLed(halHandle, LED_EXTERN_GREEN);
                gStatusLedBlinkLastToggleTime = gTimer0_stamp;
            }
        } else {
            HAL_turnLedOn(halHandle, LED_ONBOARD_BLUE);
            HAL_turnLedOn(halHandle, LED_EXTERN_GREEN);
        }
    } else  // system disabled
    {
        HAL_turnLedOff(halHandle, LED_ONBOARD_BLUE);
        HAL_turnLedOff(halHandle, LED_EXTERN_GREEN);
    }

    //*** YELLOW LED
    // Turn on if CAN messages are aborted
    if (gCanAbortingMessages) {
        HAL_turnLedOn(halHandle, LED_EXTERN_YELLOW);
    } else {
        HAL_turnLedOff(halHandle, LED_EXTERN_YELLOW);
    }

    //*** RED LED
    // Turn on if there is an error
    if (gErrors.all) {
        HAL_turnLedOn(halHandle, LED_ONBOARD_RED);
        HAL_turnLedOn(halHandle, LED_EXTERN_RED);
    } else {
        HAL_turnLedOff(halHandle, LED_ONBOARD_RED);
        HAL_turnLedOff(halHandle, LED_EXTERN_RED);
    }
}

//@} //defgroup
// end of file
