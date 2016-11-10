/**
 * \brief API for more convenient use of the eCAN module
 *
 * The goal of this API is to provide a simple, clear interface to control the
 * motors via the eCAN module that wraps all the ugly register stuff. I.e. it
 * should be understandable by everyone, even if they did not read the eCAN
 * documentation.
 *
 * \author Felix Widmaier <fwidmaier@tue.mpg.de>
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

//! \name Mailbox Bitmasks
//! \brief Bit masks specifying the mailboxes used for the various types of
//! messages.
//!
//! We have enough mailboxes available so that we can use a separate one for
//! every message type.
//! `CAN_MBOX_IN|OUT_XY = 1 << n` means that mailbox n is used for message type
//! XY.
//!
//! NOTE: Changing the mailbox number of a message also requires some
//! adjustments in the code below!
//! \{
#define CAN_MBOX_OUT_STATUSMSG  (uint32_t) 1 << 15
#define CAN_MBOX_OUT_Iq         (uint32_t) 1 << 14
#define CAN_MBOX_OUT_ENC_POS    (uint32_t) 1 << 13
#define CAN_MBOX_OUT_SPEED      (uint32_t) 1 << 12
#define CAN_MBOX_OUT_ADC6       (uint32_t) 1 << 11
#define CAN_MBOX_OUT_ENC_INDEX  (uint32_t) 1 << 10
#define CAN_MBOX_IN_COMMANDS    (uint32_t) 1 << 0
#define CAN_MBOX_IN_IqRef       (uint32_t) 1 << 1

#define CAN_MBOX_ALL  CAN_MBOX_OUT_STATUSMSG \
	| CAN_MBOX_OUT_Iq \
	| CAN_MBOX_OUT_ENC_POS \
	| CAN_MBOX_OUT_SPEED \
	| CAN_MBOX_OUT_ADC6 \
	| CAN_MBOX_OUT_ENC_INDEX \
	| CAN_MBOX_IN_COMMANDS \
	| CAN_MBOX_IN_IqRef
//! \}


//! \name Arbitration IDs
//! \brief Arbitration IDs of the different message types
//! \{
#define CAN_ID_COMMANDS   0x00
#define CAN_ID_IqRef      0x05
#define CAN_ID_STATUSMSG  0x10
#define CAN_ID_Iq         0x20
#define CAN_ID_POS        0x30
#define CAN_ID_SPEED      0x40
#define CAN_ID_ADC6       0x50
#define CAN_ID_ENC_INDEX  0x60
//! \}


//! Command IDs
//! \brief IDs of the various commands that can be sent to the COMMANDS mailbox
//!
//! The ID is expected to be in the high bytes (MDH) of the frame.
//! \{
#define CAN_CMD_ENABLE_SYS 1
#define CAN_CMD_ENABLE_MTR1 2
#define CAN_CMD_ENABLE_MTR2 3
#define CAN_CMD_ENABLE_VSPRING1 4
#define CAN_CMD_ENABLE_VSPRING2 5
#define CAN_CMD_SEND_CURRENT 12
#define CAN_CMD_SEND_POSITION 13
#define CAN_CMD_SEND_VELOCITY 14
#define CAN_CMD_SEND_ADC6 15
#define CAN_CMD_SEND_ENC_INDEX 16
#define CAN_CMD_SEND_ALL 20
#define CAN_CMD_SET_CAN_RECV_TIMEOUT 30
#define CAN_CMD_ENABLE_POS_ROLLOVER_ERROR 31
//! \}


//! \name Error Codes
//! \anchor ErrorCodes
//! \brief Possible Error Codes for the status message
//! \{

//! \brief No error
#define CAN_ERROR_NO_ERROR 0
//! \brief Encoder error too high
#define CAN_ERROR_ENCODER 1
//! \brief Timeout for receiving current references exceeded
#define CAN_ERROR_CAN_RECV_TIMEOUT 2
//! \brief Motor temperature reached critical value
//! \note This is currently unused as no temperature sensing is done.
#define CAN_ERROR_CRIT_TEMP 3  // currently unused
//! \brief Some error in the SpinTAC Position Convert module
#define CAN_ERROR_POSCONV 4
//! \brief Position Rollover occured
#define CAN_ERROR_POS_ROLLOVER 5
//! \brief Some other error
#define CAN_ERROR_OTHER 7
//! \}



// **************************************************************************
// the typedefs

//! \brief Status message bits.
struct CAN_STATUSMSG_BITS
{                              // bits
   uint16_t system_enabled:1;  // 0
   uint16_t motor1_enabled:1;  // 1
   uint16_t motor1_ready:1;    // 2
   uint16_t motor2_enabled:1;  // 3
   uint16_t motor2_ready:1;    // 4
   //! \see \ref ErrorCodes
   uint16_t error_code:3;      // 5-7
   uint16_t rsvd:8;            // 8-15
};

//! \brief Status message that allows integer or bit access.
typedef union _CAN_StatusMsg_t_
{
   uint16_t              all;
   struct CAN_STATUSMSG_BITS  bit;
} CAN_StatusMsg_t;


//! \brief Command message.
typedef struct _CAN_Command_t_
{
	//! \brief Command ID
	uint32_t id;
	//! \brief Value of the command
	uint32_t value;
} CAN_Command_t;


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


//! \brief Write motor data of motor 1 to the corresponding transmission
//! 	mailboxes.
//!
//! \param current_iq  The current Iq of the motor in A.
//! \param encoder_position  Current position of the motor in mechanical
//!                          revolutions (mrev).
//! \param velocity  Velocity of the motor in krpm.
inline void CAN_setDataMotor1(_iq current_iq, _iq position, _iq velocity)
{
	ECanaMboxes.MBOX14.MDL.all = current_iq;
	ECanaMboxes.MBOX13.MDL.all = position;
	ECanaMboxes.MBOX12.MDL.all = velocity;

	return;
}


//! \brief Write motor data of motor 2 to the corresponding transmission
//! 	mailboxes.
//!
//! \param current_iq  The current Iq of the motor in A.
//! \param encoder_position  Current position of the motor in mechanical
//!                          revolutions (mrev).
//! \param velocity  Velocity of the motor in krpm.
inline void CAN_setDataMotor2(_iq current_iq, _iq encoder_position,
		_iq velocity)
{
	ECanaMboxes.MBOX14.MDH.all = current_iq;
	ECanaMboxes.MBOX13.MDH.all = encoder_position;
	ECanaMboxes.MBOX12.MDH.all = velocity;

	return;
}


//! \brief Write readings of ADCINA6 and B6 to the corresponding transmission
//! 	mailbox.
//!
//! \param adcin_a6 Result of ADCINA6
//! \param adcin_b6 Result of ADCINB6
inline void CAN_setAdcIn6Values(_iq adcin_a6, _iq adcin_b6)
{
	ECanaMboxes.MBOX11.MDL.all = adcin_a6;
	ECanaMboxes.MBOX11.MDH.all = adcin_b6;
}


//! \brief Write encoder index position to the corresponding transmission
//! 	mailbox.
//!
//! \param mtrNum Number of the motor (either 0 or 1).
//! \param index_position Position of the motor when the index occured [mrev].
inline void CAN_setEncoderIndex(uint16_t mtrNum, _iq index_position)
{
	ECanaMboxes.MBOX10.MDH.byte.BYTE4 = mtrNum & 0xFF;
	ECanaMboxes.MBOX10.MDL.all = index_position;
}


//! \brief Get the last command message that was received.
//!
//! \returns Last received command message.
inline CAN_Command_t CAN_getCommand()
{
	CAN_Command_t cmd;

	cmd.id = ECanaMboxes.MBOX0.MDH.all;
	cmd.value = ECanaMboxes.MBOX0.MDL.all;

	return cmd;
}


//! \brief Get the last Iq reference that was received.
//!
//! \param mtrNum Number of the motor
//! \returns Last received IqRef value for the specified motor.
inline _iq CAN_getIqRef(uint16_t mtrNum)
{
	if (mtrNum == HAL_MTR1) {
		return ECanaMboxes.MBOX1.MDL.all;
	} else {
		return ECanaMboxes.MBOX1.MDH.all;
	}
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


//! \brief Abort pending transmissions of the specified mailboxes.
//!
//! \param mailboxes  A bitmap specifying the mailboxes to be send.
inline void CAN_abort(uint32_t mailboxes)
{
	// request transmission reset
	ECanaRegs.CANTRR.all |= mailboxes;
	// wait until TRS is cleared
	//while ((ECanaRegs.CANTRS.all & mailboxes) != 0);

	return;
}


//! \brief Check if received message pending flag is set.
//!
//! Check if the "received message pending" flag is set for the specified
//! mailbox.  To clear the flag, use CAN_clearReceivedMessagePending().
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, this function will return true if a
//! 	message is pending in at least one of the mailboxes.
//! \returns True if a new message is pending in the specified mailbox.
inline bool CAN_checkReceivedMessagePending(uint32_t mailbox_mask)
{
	return ECanaRegs.CANRMP.all & mailbox_mask;
}


//! \brief Clear received message pending flag.
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, the flag is reset for each of them.
inline void CAN_clearReceivedMessagePending(uint32_t mailbox_mask)
{
	// reset bit (have to write a 1 to get a 0)
	ECanaRegs.CANRMP.all = mailbox_mask;
}


//! \brief Check if new message arrived and acknowledge if yes.
//!
//! Check if the "received message pending" flag for the specified mailbox.  If
//! yes, the message is acknowledged (i.e. the flag is reset).
//!
//! \param mailbox_mask Bitmask that specifies the mailbox.  If you specify more
//! 	than one mailbox in the mask, this function will return true if a
//! 	message is pending in at least one of the mailboxes.
//! \returns True if a new message is pending in the specified mailbox.
inline bool CAN_checkAndClearRMP(uint32_t mailbox_mask)
{
	if (CAN_checkReceivedMessagePending(mailbox_mask))
	{
		CAN_clearReceivedMessagePending(mailbox_mask);
		return true;
	}
	else
	{
		return false;
	}
}


//! \brief Check if a message is waiting for transmission
//!
//! Check if a message is waiting for transmission (i.e. the TRS bit is set) in
//! the specified mailbox.  If more then one mailbox is specified, it is checked
//! if at least one of them has a pending message.
//!
//! \param mailbox_mask Bitmask that specifies the mailbox(es).  See the
//! 	CAN_MBOX_OUT_* defines.
//! \returns True if a message is waiting for transmission in one of the
//! 	specified mailboxes.
inline bool CAN_checkTransmissionPending(uint32_t mailbox_mask)
{
	return ECanaRegs.CANTRS.all & mailbox_mask;
}


#ifdef __cplusplus
}
#endif // extern "C"


#endif /* SRC_CANAPI_H_ */
