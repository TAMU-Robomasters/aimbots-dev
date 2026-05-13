#include "vtm_can.hpp"

#include <cstring>

#include "tap/architecture/clock.hpp"
#include "tap/drivers.hpp"
#include "modm/architecture/interface/can_message.hpp"

namespace src::communicators::vtm_can
{
using tap::arch::clock::getTimeMilliseconds;

VtmCan::RespListener::RespListener(
    tap::Drivers* drivers,
    uint32_t id,
    tap::can::CanBus bus,
    VtmCan* parent)
    : tap::can::CanRxListener(drivers, id, bus),
      parent(parent)
{
}

void VtmCan::RespListener::processMessage(const modm::can::Message& rxMessage)
{
    parent->handleRespFrame(rxMessage);
}

VtmCan::VtmCan(tap::Drivers* drivers)
    : drivers(drivers),
      respListener(drivers, CAN_ID_RESP, BUS, this)
{
}

void VtmCan::initialize()
{
    respListener.attachSelfToRxHandler();
    lastPollTimeMs = getTimeMilliseconds();
    lastPacketTimeMs = 0;
    updateCounter = 0;
    for (auto &b : payload) b = 0;
    for (int i = 0; i < NUM_SEGS; i++) segReceived[i] = false;
}

void VtmCan::read()
{
    // only call this in one place
    // in all fairness, this should be in main and not here
    drivers->canRxHandler.pollCanData();

    sendPollRtrIfDue();
}

bool VtmCan::hasFreshPacket(uint32_t timeoutMs) const
{
    if (updateCounter == 0) return false;
    const uint32_t now = getTimeMilliseconds();
    return (now - lastPacketTimeMs) <= timeoutMs;
}

void VtmCan::sendPollRtrIfDue()
{
    const uint32_t now = getTimeMilliseconds();
    if (now - lastPollTimeMs < pollPeriodMs) return;
    lastPollTimeMs = now;

    modm::can::Message msg(CAN_ID_POLL_RTR, 0);
    msg.setExtended(false);
    msg.setRemoteTransmitRequest(true);

    if (drivers->can.isReadyToSend(BUS))
    {
        (void)drivers->can.sendMessage(BUS, msg);
    }
}

void VtmCan::handleRespFrame(const modm::can::Message& rxMessage)
{
    // DLC=8: seq, segIndex, 6 bytes
    if (rxMessage.getLength() != 8) return;

    const uint8_t seq = rxMessage.data[0];
    const uint8_t seg = rxMessage.data[1];
    if (seg >= NUM_SEGS) return;

    // clear reassembly state
    if (seq != currentSeq)
    {
        currentSeq = seq;
        for (int i = 0; i < NUM_SEGS; i++) segReceived[i] = false;
    }

    // copy 6 payload bytes
    memcpy(segPayload[seg], &rxMessage.data[2], BYTES_PER_SEG);
    segReceived[seg] = true;

    // assemble 30B if all segments present
    bool all = true;
    for (int i = 0; i < NUM_SEGS; i++) all = all && segReceived[i];
    if (!all) return;

    for (int i = 0; i < NUM_SEGS; i++)
    {
        memcpy(&payload[i * BYTES_PER_SEG], segPayload[i], BYTES_PER_SEG);
    }

    updateCounter++;
    lastPacketTimeMs = getTimeMilliseconds();

    // ready for next packet
    for (int i = 0; i < NUM_SEGS; i++) segReceived[i] = false;
}

}  // namespace src::communicators::vtm_can
