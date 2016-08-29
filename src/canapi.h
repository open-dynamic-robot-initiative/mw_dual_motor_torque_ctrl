/*
 * canapi.h
 *
 *  Created on: 25 Aug 2016
 *      Author: fwidmaier
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

#define CAN_MBOX_STATUSMSG  (uint32_t) 1 << 15
#define CAN_MBOX_IqPos_mtr1  (uint32_t) 1 << 14
#define CAN_MBOX_IqPos_mtr2  (uint32_t) 1 << 13
#define CAN_MBOX_SPEED_mtr1  (uint32_t) 1 << 12
#define CAN_MBOX_SPEED_mtr2  (uint32_t) 1 << 11
#define CAN_MBOX_COMMANDS (uint32_t) 1 << 0

#define CAN_MBOX_ALL  CAN_MBOX_STATUSMSG \
	| CAN_MBOX_IqPos_mtr1 \
	| CAN_MBOX_IqPos_mtr2 \
	| CAN_MBOX_SPEED_mtr1 \
	| CAN_MBOX_SPEED_mtr2 \
	| CAN_MBOX_COMMANDS


#define CAN_ID_COMMANDS 0
#define CAN_ID_STATUSMSG  0x10
#define CAN_ID_IqPos_mtr1  0x21
#define CAN_ID_IqPos_mtr2  0x22
#define CAN_ID_SPEED_mtr1  0x31
#define CAN_ID_SPEED_mtr2  0x32


// **************************************************************************
// the typedefs

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

typedef union _CAN_StatusMsg_t_ {
   uint16_t              all;
   struct CAN_STATUSMSG_BITS  bit;
} CAN_StatusMsg_t;


// **************************************************************************
// the globals


// **************************************************************************
// the function prototypes


extern void InitECana();        // Initialize eCAN-A module
extern void InitECanaGpio(HAL_Handle halHandle);
extern void CAN_setupMboxes();


inline void CAN_setStatusMsg(CAN_StatusMsg_t statusmsg)
{
	ECanaMboxes.MBOX15.MDL.byte.BYTE0 = statusmsg.all;

	return;
}


inline void CAN_setDataMotor1(_iq current_iq, _iq encoder_position, _iq velocity)
{
	ECanaMboxes.MBOX14.MDL.all = current_iq;
	ECanaMboxes.MBOX14.MDH.all = encoder_position;
	ECanaMboxes.MBOX12.MDL.all = velocity;

	return;
}


inline void CAN_setDataMotor2(_iq current_iq, _iq encoder_position, _iq velocity)
{
	ECanaMboxes.MBOX13.MDL.all = current_iq;
	ECanaMboxes.MBOX13.MDH.all = encoder_position;
	ECanaMboxes.MBOX11.MDL.all = velocity;

	return;
}


inline void CAN_send(uint32_t mailboxes)
{
	ECanaRegs.CANTRS.all |= mailboxes;  // Set TRS for all specified mailboxes
	while (ECanaRegs.CANTA.all & mailboxes != mailboxes);  // Wait for all TAn bits of specified mailboxes to be set..
	ECanaRegs.CANTA.all = mailboxes;   // Clear all TAn (I have to set it to 1 so it becomes 0...)
	while (ECanaRegs.CANTA.all & mailboxes != 0); // wait for all TAn bits to become zero

	return;
}



#ifdef __cplusplus
}
#endif // extern "C"


#endif /* SRC_CANAPI_H_ */
