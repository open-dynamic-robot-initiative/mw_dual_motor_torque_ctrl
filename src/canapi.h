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

#define CAN_MBOX_STATUSMSG  1 << 15
#define CAN_MBOX_Iq  1 << 14
#define CAN_MBOX_POSITION  1 << 13
#define CAN_MBOX_SPEED  1 << 12
#define CAN_MBOX_ALL  0x0000F000

#define CAN_ID_STATUSMSG  0
#define CAN_ID_Iq  1
#define CAN_ID_POSITION  2
#define CAN_ID_SPEED  3


// **************************************************************************
// the typedefs

struct CAN_STATUSMSG_BITS {          // bits   description
   uint32_t enable_system:1;     // 0
   uint32_t run_motor1:1;        // 1
   uint32_t run_motor2:1;        // 2
   uint32_t motor1_temp_alert:1; // 3
   uint32_t motor2_temp_alert:1; // 4
   uint32_t qvalue:5;            // 5:9
   uint32_t rsvd:22;             // 10:31   reserved
};

typedef union _CAN_StatusMsg_t_ {
   uint32_t              all;
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
	ECanaMboxes.MBOX15.MDL.all = statusmsg.all;
	// TODO: dont send full 8 bytes but only what is actually used.

	return;
}


inline void CAN_setIq(_iq Iq_mtr1, _iq Iq_mtr2)
{
	ECanaMboxes.MBOX14.MDL.all = (uint32_t)Iq_mtr1;
	ECanaMboxes.MBOX14.MDH.all = (uint32_t)Iq_mtr2;

	return;
}


inline void CAN_setPosition(_iq pos_mtr1, _iq pos_mtr2)
{
	ECanaMboxes.MBOX13.MDL.all = (uint32_t)pos_mtr1;
	ECanaMboxes.MBOX13.MDH.all = (uint32_t)pos_mtr2;

	return;
}


inline void CAN_setSpeed(_iq speed_mtr1, _iq speed_mtr2)
{
	ECanaMboxes.MBOX12.MDL.all = (uint32_t)speed_mtr1;
	ECanaMboxes.MBOX12.MDH.all = (uint32_t)speed_mtr2;

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
