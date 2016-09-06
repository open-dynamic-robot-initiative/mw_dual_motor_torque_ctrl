/*
 * API for more convenient use of the eCAN module
 *
 * The goal of this API is to provide a simple, clear interface to control the
 * motors via the eCAN module that wraps all the ugly register stuff. I.e. it
 * should be understandable by everyone, even if they did not read the eCAN
 * documentation.
 */

#ifndef SRC_CANAPI_H_
#define SRC_CANAPI_H_

// **************************************************************************
// the includes
#include "sw/drivers/can/src/32b/f28x/f2806x/can.h"
#include "hal_2mtr.h"


#ifdef __cplusplus
extern "C" {
#endif


// **************************************************************************
// the defines

// Bit masks specifying the mailboxes used for the various types of messages. We
// have enough mailboxes available so that we can use a separate one for every
// message type.
// `CAN_MBOX_IN|OUT_XY = 1 << n` means that mailbox n is used for message type
// XY.
//
// NOTE: Changing the mailbox number of a message also requires adjustments in
// the code below!
#define CAN_MBOX_OUT_STATUSMSG  (uint32_t) 1 << 15
#define CAN_MBOX_OUT_Iq         (uint32_t) 1 << 14
#define CAN_MBOX_OUT_ENC_POS    (uint32_t) 1 << 13
#define CAN_MBOX_OUT_SPEED      (uint32_t) 1 << 12
#define CAN_MBOX_OUT_ADC6       (uint32_t) 1 << 11
#define CAN_MBOX_IN_COMMANDS    (uint32_t) 1 << 0
#define CAN_MBOX_IN_IqRef       (uint32_t) 1 << 1

#define CAN_MBOX_ALL  CAN_MBOX_OUT_STATUSMSG \
	| CAN_MBOX_OUT_Iq \
	| CAN_MBOX_OUT_ENC_POS \
	| CAN_MBOX_OUT_SPEED \
	| CAN_MBOX_OUT_ADC6 \
	| CAN_MBOX_IN_COMMANDS \
	| CAN_MBOX_IN_IqRef


// Arbitration IDs of the different message types
#define CAN_ID_COMMANDS   0x00
#define CAN_ID_IqRef      0x05
#define CAN_ID_STATUSMSG  0x10
#define CAN_ID_Iq         0x20
#define CAN_ID_END_POS    0x30
#define CAN_ID_SPEED      0x40
#define CAN_ID_ADC6       0x50


// **************************************************************************
// the typedefs

//! \brief Status message bits.
struct CAN_STATUSMSG_BITS {      // bits   description
   uint16_t enable_system:1;     // 0
   uint16_t run_motor1:1;        // 1
   uint16_t ready_motor1:1;      // 2
   uint16_t run_motor2:1;        // 3
   uint16_t ready_motor2:1;      // 4
   uint16_t motor1_temp_alert:1; // 5
   uint16_t motor2_temp_alert:1; // 6
   uint16_t system_error:1;      // 7
   uint16_t rsvd:8;              // 8-15  reserved
};

//! \brief Status message that allows integer or bit access.
typedef union _CAN_StatusMsg_t_ {
   uint16_t              all;
   struct CAN_STATUSMSG_BITS  bit;
} CAN_StatusMsg_t;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


//! \brief Initialize the eCAN-A GPIOs
extern void CAN_initECanaGpio(HAL_Handle halHandle);

//! \brief Initialize eCAN-A module
extern void CAN_initECana();

//! \brief Setup mailboxes
extern void CAN_setupMboxes();

//! \brief Write status message to the corresponding transmission mailbox.
inline void CAN_setStatusMsg(CAN_StatusMsg_t statusmsg)
{
	ECanaMboxes.MBOX15.MDL.byte.BYTE0 = statusmsg.all;

	return;
}

//! \brief Write motor data of motor 1 to the corresponding transmission mailboxes
//! \param current_iq  The current Iq of the motor in A.
//! \param encoder_position  Current position of the motor in mechanical revolutions (mrev).
//! \param velocity  Velocity of the motor in krpm.
inline void CAN_setDataMotor1(_iq current_iq, _iq position, _iq velocity)
{
	ECanaMboxes.MBOX14.MDL.all = current_iq;
	ECanaMboxes.MBOX13.MDL.all = position;
	ECanaMboxes.MBOX12.MDL.all = velocity;

	return;
}

//! \brief Write motor data of motor 2 to the corresponding transmission mailboxes
//! \param current_iq  The current Iq of the motor in A.
//! \param encoder_position  Current position of the motor in mechanical revolutions (mrev).
//! \param velocity  Velocity of the motor in krpm.
inline void CAN_setDataMotor2(_iq current_iq, _iq encoder_position, _iq velocity)
{
	ECanaMboxes.MBOX14.MDH.all = current_iq;
	ECanaMboxes.MBOX13.MDH.all = encoder_position;
	ECanaMboxes.MBOX12.MDH.all = velocity;

	return;
}

//! \brief Send data of the specified mailboxes
//!
//! To specify the mailboxes, use the CAN_MBOX_OUT_xy defines for this. Example:
//! to send current, position and velocity of motor 1 call
//! `CAN_send(CAN_MBOX_OUT_IqPos_mtr1 | CAN_MBOX_OUT_SPEED_mtr1);`
//!
//! \param mailboxes  A bitmap specifying the mailboxes to be send.
inline void CAN_send(uint32_t mailboxes)
{
	// Always access whole register, not bitfield.  Therefore no shadow register
	// should be necessary.

	// Set TRS for all specified mailboxes
	ECanaRegs.CANTRS.all |= mailboxes;
	// Wait for all TAn bits of specified mailboxes to be set
//	while ((ECanaRegs.CANTA.all & mailboxes) != mailboxes);
//	// Clear all TAn (we have to set it to 1 so it becomes 0...)
//	ECanaRegs.CANTA.all = mailboxes;
//	// wait for all TAn bits to become zero
//	while ((ECanaRegs.CANTA.all & mailboxes) != 0);

	return;
}


inline void CAN_abort(uint32_t mailboxes)
{
	// request transmission reset
	ECanaRegs.CANTRR.all |= mailboxes;
	// wait until TRS is cleared
	//while ((ECanaRegs.CANTRS.all & mailboxes) != 0);

	return;
}


#ifdef __cplusplus
}
#endif // extern "C"


#endif /* SRC_CANAPI_H_ */
