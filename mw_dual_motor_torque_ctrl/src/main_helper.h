// BSD 3-Clause License
//
// Copyright (c) 2019, Max Planck Gesellschaft, New York University
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * \brief Various functions used in the main file.
 *
 * Those functions are moved here, to make the main file less overloaded.
 */

#ifndef SRC_MAIN_HELPER_H_
#define SRC_MAIN_HELPER_H_

// **************************************************************************
// the includes
#include "hal_2mtr.h"
#include "sw/modules/clarke/src/32b/clarke.h"
#include "sw/modules/pid/src/32b/pid.h"


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
// the function prototypes

//! \brief Setup the PID controllers for the currents Id and Iq.
//! \param pidHandle Array of three PID handles (speed (unused), Id, Iq).
//! \param userParams User parameters.
void pidSetup(PID_Handle pidHandle[], USER_Params userParams);

//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numCurrentSensors  The number of current sensors
void setupClarke_I(CLARKE_Handle, const uint_least8_t);

//! \brief     Setup the Clarke transform for either 2 or 3 sensors.
//! \param[in] handle             The clarke (CLARKE) handle
//! \param[in] numVoltageSensors  The number of voltage sensors
void setupClarke_V(CLARKE_Handle, const uint_least8_t);


//! \brief Updates version 1p6 of library
//!
void softwareUpdate1p6(EST_Handle handle, USER_Params *pUserParams);


void PIE_registerTimer0IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr);
void PIE_registerCan1IntHandler(PIE_Handle pieHandle, PIE_IntVec_t isr);

//! \brief Setup the CAN module, interrupts, etc.
void setupCan(HAL_Handle halHandle, PIE_IntVec_t can1_ISR);

//! \brief Overwrite some of the default settings done in HAL_setupGpio().
extern void HAL_overwriteSetupGpio(HAL_Handle halHandle);


//! \brief Overwrite the stettings for timer0 done in HAL_setParams().
//!
//! Call this *after* HAL_setParams().
void overwriteSetupTimer0(HAL_Handle handle, const uint32_t timerPeriod_counts);

//! \brief Set up the interrupts for encoder indices
void setupQepIndexInterrupt(HAL_Handle halHandle,
		HAL_Handle_mtr halHandleMtr[2],
		PIE_IntVec_t qep1IndexIsr,
		PIE_IntVec_t qep2IndexIsr);


#endif /* SRC_MAIN_HELPER_H_ */
