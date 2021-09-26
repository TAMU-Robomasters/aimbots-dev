/*
 * Copyright (c) 2011-2012, 2017, Fabian Greif
 * Copyright (c) 2012-2014, 2016-2017, Sascha Schade
 * Copyright (c) 2013-2014, 2016, Kevin Läufer
 * Copyright (c) 2014, Georgi Grinshpun
 * Copyright (c) 2014, 2016-2018, Niklas Hauser
 * Copyright (c) 2018, Christopher Durand
 *
 * This file is part of the modm project.
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */
// ----------------------------------------------------------------------------

#include <modm/architecture/driver/atomic/queue.hpp>
#include <modm/utils.hpp>
#include <modm/architecture/interface/assert.hpp>
#include <modm/architecture/interface/interrupt.hpp>
#include <modm/architecture/interface/delay.hpp>
#include <modm/platform/clock/rcc.hpp>
#include <cstring>

#include "can_1.hpp"
// ----------------------------------------------------------------------------
// CAN bit timing register (CAN_BTR)
#define CAN_BTR_SJW_POS		24
#define CAN_BTR_TS2_POS		20
#define CAN_BTR_TS1_POS		16

// ----------------------------------------------------------------------------
static modm::atomic::Queue<modm::can::Message, 32> txQueue;
struct RxMessage {
    modm::can::Message message;
    uint8_t filter_id;
};
static modm::atomic::Queue<RxMessage, 32> rxQueue;
// ----------------------------------------------------------------------------
bool
modm::platform::Can1::initializeWithPrescaler(
		uint16_t prescaler, uint8_t bs1, uint8_t bs2,
		uint32_t interruptPriority, Mode startupMode, bool overwriteOnOverrun)
{
	// skip enable and reset if device has already been enabled by slave
	if (not Rcc::isEnabled<Peripheral::Can1>()) {
		Rcc::enable<Peripheral::Can1>();
	}
	// CAN Master Reset
	// FMP bits and CAN_MCR register are initialized to the reset values
	CAN1->MCR |= CAN_MCR_RESET;
	while (CAN1->MCR & CAN_MCR_RESET)
		;

	// Exit from sleep mode
	CAN1->MCR &= (~(uint32_t)CAN_MCR_SLEEP);

	// Bus off is left automatically by the hardware after 128 occurrences
	// of 11 recessive bits, TX Order depends on the order of request and
	// not on the CAN priority.
	if (overwriteOnOverrun) {
		CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_TXFP;
	}
	else {
		// No overwrite at RX FIFO: Once a receive FIFO is full the next
		// incoming message will be discarded
		CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_RFLM | CAN_MCR_TXFP;
	}

	// Request initialization
	CAN1->MCR |= CAN_MCR_INRQ;
	int deadlockPreventer = 10'000; // max ~10ms
	while (((CAN1->MSR & CAN_MSR_INAK) == 0) and (deadlockPreventer-- > 0)) {
		modm::delay_us(1);
		// Wait until the initialization mode is entered.
		// The CAN hardware waits until the current CAN activity (transmission
		// or reception) is completed before entering the initialization mode.
	}
	if (deadlockPreventer == 0)
		return false;

	// Enable Interrupts:
	// FIFO1 Overrun, FIFO0 Overrun
	CAN1->IER = CAN_IER_FOVIE1 | CAN_IER_FOVIE0;

	CAN1->IER |= CAN_IER_TMEIE;
	// Set vector priority
	NVIC_SetPriority(CAN1_RX0_IRQn, interruptPriority);
	NVIC_SetPriority(CAN1_RX1_IRQn, interruptPriority);

	// Register Interrupts at the NVIC
	NVIC_EnableIRQ(CAN1_RX0_IRQn);
	NVIC_EnableIRQ(CAN1_RX1_IRQn);

	NVIC_EnableIRQ(CAN1_TX_IRQn);
	NVIC_SetPriority(CAN1_TX_IRQn, interruptPriority);
	CAN1->IER |= CAN_IER_FMPIE1 | CAN_IER_FMPIE0;
	CAN1->BTR =
			  ((1 - 1) << CAN_BTR_SJW_POS) |		// SJW (1 to 4 possible)
			((bs2 - 1) << CAN_BTR_TS2_POS) |		// BS2 Samplepoint
			((bs1 - 1) << CAN_BTR_TS1_POS) |		// BS1 Samplepoint
			static_cast<uint32_t>(startupMode) |
			(prescaler - 1);

	// Request leave initialization
	CAN1->MCR &= ~(uint32_t)CAN_MCR_INRQ;
	deadlockPreventer = 10'000; // max ~10ms
	while (((CAN1->MSR & CAN_MSR_INAK) == CAN_MSR_INAK) and (deadlockPreventer-- > 0))  {
		// wait for the normal mode
	}
	return deadlockPreventer > 0;
}

// ----------------------------------------------------------------------------
// Configure the mailbox to send a CAN message.
// Low level function called by sendMessage and by Tx Interrupt.
static void
sendMailbox(const modm::can::Message& message, uint32_t mailboxId)
{
	CAN_TxMailBox_TypeDef* mailbox = &CAN1->sTxMailBox[mailboxId];

	if (message.isExtended()) {
		mailbox->TIR = message.identifier << 3 | CAN_TI0R_IDE;
	}
	else {
		mailbox->TIR = message.identifier << 21;
	}

	if (message.isRemoteTransmitRequest()) {
		mailbox->TIR |= CAN_TI0R_RTR;
	}

	// Set up the DLC
	mailbox->TDTR = message.getLength();

	// Set up the data field (copy the 8x8-bits into two 32-bit registers)
	const uint8_t * modm_may_alias data = message.data;
	mailbox->TDLR = reinterpret_cast<const uint32_t *>(data)[0];
	mailbox->TDHR = reinterpret_cast<const uint32_t *>(data)[1];

	// Request transmission
	mailbox->TIR |= CAN_TI0R_TXRQ;
}

// ----------------------------------------------------------------------------
// Low level function to receive a message from mailbox.
// Called by Rx Interrupt or by getMessage.
static void
readMailbox(modm::can::Message& message, uint32_t mailboxId, uint8_t* filter_id)
{
	CAN_FIFOMailBox_TypeDef* mailbox = &CAN1->sFIFOMailBox[mailboxId];

	uint32_t rir = mailbox->RIR;
	if (rir & CAN_RI0R_IDE) {
		message.identifier = rir >> 3;
		message.setExtended();
	}
	else {
		message.identifier = rir >> 21;
		message.setExtended(false);
	}
	message.setRemoteTransmitRequest(rir & CAN_RI0R_RTR);

	message.length = mailbox->RDTR & CAN_TDT1R_DLC;
	if(filter_id != nullptr)
        (*filter_id) = (mailbox->RDTR & CAN_RDT1R_FMI) >> CAN_RDT1R_FMI_Pos;

	uint8_t * modm_may_alias data = message.data;
	reinterpret_cast<uint32_t *>(data)[0] = mailbox->RDLR;
	reinterpret_cast<uint32_t *>(data)[1] = mailbox->RDHR;
}

// ----------------------------------------------------------------------------
/* Transmit Interrupt
 *
 * Generated when Transmit Mailbox 0..2 becomes empty.
 */

MODM_ISR(CAN1_TX)
{
	uint32_t mailbox;
	uint32_t tsr = CAN1->TSR;

	if (tsr & CAN_TSR_RQCP2) {
		mailbox = 2;
		CAN1->TSR = CAN_TSR_RQCP2;
	}
	else if (tsr & CAN_TSR_RQCP1) {
		mailbox = 1;
		CAN1->TSR = CAN_TSR_RQCP1;
	}
	else {
		mailbox = 0;
		CAN1->TSR = CAN_TSR_RQCP0;
	}

	if (txQueue.isNotEmpty())
	{
		sendMailbox(txQueue.get(), mailbox);
		txQueue.pop();
	}
}

// ----------------------------------------------------------------------------
/* FIFO0 Interrupt
 *
 * Generated on a new received message, FIFO0 full condition and Overrun
 * Condition.
 */
MODM_ISR(CAN1_RX0)
{
	if (not modm_assert_continue_ignore(not (CAN1->RF0R & CAN_RF0R_FOVR0),
			"can.rx.hw0", "CAN receive hardware buffer overflowed!", 1))
	{
		// release overrun flag & access the next message
		CAN1->RF0R = CAN_RF0R_FOVR0 | CAN_RF0R_RFOM0;
	}

	RxMessage rxMessage;
	readMailbox(rxMessage.message, 0, &(rxMessage.filter_id));

	// Release FIFO (access the next message)
	CAN1->RF0R = CAN_RF0R_RFOM0;

	modm_assert_continue_ignore(rxQueue.push(rxMessage), "can.rx.sw0",
		"CAN receive software buffer overflowed!", 1);
}

// ----------------------------------------------------------------------------
/* FIFO1 Interrupt
 *
 * See FIFO0 Interrupt
 */
MODM_ISR(CAN1_RX1)
{
	if (not modm_assert_continue_ignore(not (CAN1->RF1R & CAN_RF1R_FOVR1),
			"can.rx.hw1", "CAN receive hardware buffer overflowed!", 1))
	{
		// release overrun flag & access the next message
		CAN1->RF1R = CAN_RF1R_FOVR1 | CAN_RF1R_RFOM1;
	}

	RxMessage rxMessage;
	readMailbox(rxMessage.message, 1, &(rxMessage.filter_id));

	// Release FIFO (access the next message)
	CAN1->RF1R = CAN_RF1R_RFOM1;

	modm_assert_continue_ignore(rxQueue.push(rxMessage), "can.rx.sw1",
		"CAN receive software buffer overflowed!", 1);
}

// ----------------------------------------------------------------------------
void
modm::platform::Can1::setMode(Mode mode)
{
	// Request initialization
	CAN1->MCR |= CAN_MCR_INRQ;
	while ((CAN1->MSR & CAN_MSR_INAK) == 0) {
		// Wait until the initialization mode is entered.
		// The CAN hardware waits until the current CAN activity (transmission
		// or reception) is completed before entering the initialization mode.
	}

	CAN1->BTR = (CAN1->BTR & ~(CAN_BTR_SILM | CAN_BTR_LBKM))
						| static_cast<uint32_t>(mode);

	// Leave initialization mode
	CAN1->MCR &= ~CAN_MCR_INRQ;
}

// ----------------------------------------------------------------------------
void
modm::platform::Can1::setAutomaticRetransmission(bool retransmission)
{
	if (retransmission) {
		// Enable retransmission
		CAN1->MCR = (CAN1->MCR & ~CAN_MCR_NART);
	} else {
		// Disable retransmission
		CAN1->MCR = (CAN1->MCR | CAN_MCR_NART);
	}
}

// ----------------------------------------------------------------------------
bool
modm::platform::Can1::isMessageAvailable()
{
	return rxQueue.isNotEmpty();
}

// ----------------------------------------------------------------------------
bool
modm::platform::Can1::getMessage(can::Message& message, uint8_t *filter_id)
{
	if (rxQueue.isEmpty())
	{
		// no message in the receive buffer
		return false;
	}
	else {
        auto& rxMessage = rxQueue.get();
		memcpy(&message, &rxMessage.message, sizeof(message));
        if(filter_id != nullptr) (*filter_id) = rxMessage.filter_id;
		rxQueue.pop();
		return true;
	}
}

// ----------------------------------------------------------------------------
bool
modm::platform::Can1::isReadyToSend()
{
	return txQueue.isNotFull();
}

// ----------------------------------------------------------------------------
bool
modm::platform::Can1::sendMessage(const can::Message& message)
{
	// This function is not reentrant. If one of the mailboxes is empty it
	// means that the software buffer is empty too. Therefore the mailbox
	// will stay empty and won't be taken by an interrupt.
	if ((CAN1->TSR & (CAN_TSR_TME0 | CAN_TSR_TME1 | CAN_TSR_TME2)) == 0)
	{
		// All mailboxes used at the moment
		if (not modm_assert_continue_ignore(txQueue.push(message), "can.tx",
				"CAN transmit software buffer overflowed!", 1)) {
			return false;
		}
		return true;
	}
	else {
		// Get number of the first free mailbox
		uint32_t mailbox = (CAN1->TSR & CAN_TSR_CODE) >> 24;

		sendMailbox(message, mailbox);
		return true;
	}
}

// ----------------------------------------------------------------------------
modm::platform::Can1::BusState
modm::platform::Can1::getBusState()
{
	if (CAN1->ESR & CAN_ESR_BOFF) {
		return BusState::Off;
	}
	else if (CAN1->ESR & CAN_ESR_EPVF) {
		return BusState::ErrorPassive;
	}
	else if (CAN1->ESR & CAN_ESR_EWGF) {
		return BusState::ErrorWarning;
	}
	else {
		return BusState::Connected;
	}
}

// ----------------------------------------------------------------------------
void
modm::platform::Can1::enableStatusChangeInterrupt(
		uint32_t interruptEnable,
		uint32_t interruptPriority
)
{
	NVIC_SetPriority(CAN1_SCE_IRQn, interruptPriority);
	NVIC_EnableIRQ(CAN1_SCE_IRQn);
	CAN1->IER = interruptEnable | (CAN1->IER & 0x000000ff);
}