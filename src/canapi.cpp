/*
 * canapi.cpp
 *
 *  Created on: 25 Aug 2016
 *      Author: fwidmaier
 */

#include "canapi.h"


void CAN_initECanaGpio(HAL_Handle halHandle)
{
	HAL_Obj *obj = (HAL_Obj *)halHandle;

	/* Enable internal pull-up for the selected CAN pins */
	// Pull-ups can be enabled or disabled by the user.
	// This will enable the pullups for the specified pins.
	// Comment out other unwanted lines.
	GPIO_setPullup(obj->gpioHandle, GPIO_Number_30, GPIO_Pullup_Enable); // CANRXA
	GPIO_setPullup(obj->gpioHandle, GPIO_Number_31, GPIO_Pullup_Enable); // CANTXA

	/* Set qualification for selected CAN pins to asynch only */
	// Inputs are synchronized to SYSCLKOUT by default.
	// This will select asynch (no qualification) for the selected pins.
	//gpio->GPAQSEL2.bit.GPIO30 = 3;   // Asynch qual for GPIO30 (CANRXA)
	GPIO_setQualification(obj->gpioHandle, GPIO_Number_30, GPIO_Qual_ASync);

	/* Configure eCAN-A pins using GPIO regs*/
	// This specifies which of the possible GPIO pins will be eCAN functional pins.
	//gpio->GPAMUX2.bit.GPIO30 = 1;    // Configure GPIO30 for CANRXA operation
	//gpio->GPAMUX2.bit.GPIO31 = 1;    // Configure GPIO31 for CANTXA operation
	GPIO_setMode(obj->gpioHandle, GPIO_Number_30, GPIO_30_Mode_CANRXA);
	GPIO_setMode(obj->gpioHandle, GPIO_Number_31, GPIO_31_Mode_CANTXA);
}


void CAN_initECana()
{
	// Create a shadow register structure for the CAN control registers. This is
	// needed, since only 32-bit access is allowed to these registers. 16-bit
	// access to these registers could potentially corrupt the register contents
	// or return false data.
	struct ECAN_REGS ECanaShadow;

	ENABLE_PROTECTED_REGISTER_WRITE_MODE;


	/* Configure eCAN RX and TX pins for CAN operation using eCAN regs*/
	ECanaShadow.CANTIOC.all = ECanaRegs.CANTIOC.all;
	ECanaShadow.CANTIOC.bit.TXFUNC = 1;
	ECanaRegs.CANTIOC.all = ECanaShadow.CANTIOC.all;

	ECanaShadow.CANRIOC.all = ECanaRegs.CANRIOC.all;
	ECanaShadow.CANRIOC.bit.RXFUNC = 1;
	ECanaRegs.CANRIOC.all = ECanaShadow.CANRIOC.all;


	/* Specify if standard compatible or extended CAN is used */
	// use SSC mode
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.SCB = 0; // 0 = SSC, 1 = eCAN
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;


	/* Initialize all bits of 'Message Control Register' to zero */
	// Some bits of MSGCTRL register come up in an unknown state. For proper
	// operation, all bits (including reserved bits) of MSGCTRL must be
	// initialized to zero
	ECanaMboxes.MBOX0.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX1.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX2.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX3.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX4.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX5.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX6.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX7.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX8.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX9.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX10.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX11.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX12.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX13.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX14.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX15.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX16.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX17.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX18.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX19.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX20.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX21.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX22.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX23.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX24.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX25.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX26.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX27.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX28.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX29.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX30.MSGCTRL.all = 0x00000000;
	ECanaMboxes.MBOX31.MSGCTRL.all = 0x00000000;


	// TAn, RMPn, GIFn bits are all zero upon reset and are cleared again as a
	// matter of precaution.
	ECanaRegs.CANTA.all = 0xFFFFFFFF;   // Clear all TAn bits
	ECanaRegs.CANRMP.all = 0xFFFFFFFF;  // Clear all RMPn bits
	ECanaRegs.CANGIF0.all = 0xFFFFFFFF; // Clear all interrupt flag bits
	ECanaRegs.CANGIF1.all = 0xFFFFFFFF;


	/* Configure bit timing parameters for eCANA*/

	// requesting reconfiguration of CANBTC by setting CCR = 1
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 1;            // Set CCR = 1
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	// Wait until the CPU has been granted permission to change the
	// configuration registers
	do
	{
		ECanaShadow.CANES.all = ECanaRegs.CANES.all;
	} while(ECanaShadow.CANES.bit.CCE != 1 );       // Wait for CCE bit to be set..

	// first set erverything to 0
	ECanaShadow.CANBTC.all = 0;

	// The following block is for 90 MHz SYSCLKOUT (45 MHz CAN module clock Bit).

	// Useful Tool to calculate configuration values: http://www.bittiming.can-wiki.info
	// Note that values written to registers here have to be desired value - 1
	// because for some reason they are incremented by one before use...

	// 500 kBit/s
	//ECanaShadow.CANBTC.bit.BRPREG = 4;
	//ECanaShadow.CANBTC.bit.TSEG1REG = 12;
	//ECanaShadow.CANBTC.bit.TSEG2REG = 3;
	// 1 MBit/s
	ECanaShadow.CANBTC.bit.BRPREG = 2;
	ECanaShadow.CANBTC.bit.TSEG1REG = 11;
	ECanaShadow.CANBTC.bit.TSEG2REG = 1;
	ECanaShadow.CANBTC.bit.SJWREG = 1;
	ECanaShadow.CANBTC.bit.SAM = 1;
	ECanaRegs.CANBTC.all = ECanaShadow.CANBTC.all;

	// Configuration finished, so set CCR = 0 again
	ECanaShadow.CANMC.all = ECanaRegs.CANMC.all;
	ECanaShadow.CANMC.bit.CCR = 0 ;            // Set CCR = 0
	ECanaRegs.CANMC.all = ECanaShadow.CANMC.all;

	// Wait until the CPU no longer has permission to change the configuration registers
	do
	{
		ECanaShadow.CANES.all = ECanaRegs.CANES.all;
	} while(ECanaShadow.CANES.bit.CCE != 0 );       // Wait for CCE bit to be  cleared..


	/* Disable all Mailboxes  */
	ECanaRegs.CANME.all = 0;        // Required before writing the MSGIDs

	// Disable all mailbox interrupts at startup
    ECanaRegs.CANMIM.all = 0;

	DISABLE_PROTECTED_REGISTER_WRITE_MODE;

	return;
}


void CAN_setupMboxes()
{
	// Mailboxes can be written to 16-bits or 32-bits at a time

	/// Set arbitration IDs for all used mailboxes
	// Since we are using the standard mode, only the bits 28:18 are used.
    ECanaMboxes.MBOX15.MSGID.all = (uint32_t)CAN_ID_STATUSMSG << 18;
    ECanaMboxes.MBOX14.MSGID.all = (uint32_t)CAN_ID_Iq << 18;
    ECanaMboxes.MBOX13.MSGID.all = (uint32_t)CAN_ID_POS << 18;
    ECanaMboxes.MBOX12.MSGID.all = (uint32_t)CAN_ID_SPEED << 18;
    ECanaMboxes.MBOX11.MSGID.all = (uint32_t)CAN_ID_ADC6 << 18;
    ECanaMboxes.MBOX10.MSGID.all = (uint32_t)CAN_ID_ENC_INDEX << 18;
    ECanaMboxes.MBOX0.MSGID.all = (uint32_t)CAN_ID_COMMANDS << 18;
    ECanaMboxes.MBOX1.MSGID.all = (uint32_t)CAN_ID_IqRef << 18;

    // TODO: Don't use mbox0 for commands. Those commands should have highest
    // priority of all messages (needed to disable motor in case of some
    // failure), so it should get the highest priority receive mailbox).

    // Clear value of receive mailboxes to avoid false values
    ECanaMboxes.MBOX0.MDL.all = 0;
    ECanaMboxes.MBOX0.MDH.all = 0;
    ECanaMboxes.MBOX1.MDL.all = 0;
    ECanaMboxes.MBOX1.MDH.all = 0;

    /// Configure Mailboxes 0-4 for receiving, rest for transmitting
    // Since this write is to the entire register (instead of a bit field) a
    // shadow register is not required.
    ECanaRegs.CANMD.all = 0x0000000F;

    /// Enable all used Mailboxes
    // Since this write is to the entire register (instead of a bit
    // field) a shadow register is not required.
    ECanaRegs.CANME.all = CAN_MBOX_ALL;

    // Specify the number of bytes that will be sent/received by each mailbox
    ECanaMboxes.MBOX0.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX1.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX10.MSGCTRL.bit.DLC = 5;
    ECanaMboxes.MBOX11.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX12.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX13.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX14.MSGCTRL.bit.DLC = 8;
    ECanaMboxes.MBOX15.MSGCTRL.bit.DLC = 1;


    /// Setup interrupts
    // MBOX0 is receive mailbox for commands which should be executed
    // immediately. Use interrupt for this one.
	ENABLE_PROTECTED_REGISTER_WRITE_MODE;
    // Enable interrupt for this mailbox
    ECanaRegs.CANMIM.bit.MIM0 = 1;
    // Use ECAN1INT (which has lower priority than ECAN0INT)
    ECanaRegs.CANMIL.bit.MIL0 = 1;
    // Enable ECAN1INT
    ECanaRegs.CANGIM.bit.I1EN = 1;
	DISABLE_PROTECTED_REGISTER_WRITE_MODE;
}
