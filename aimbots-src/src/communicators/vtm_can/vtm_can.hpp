#ifndef VTM_CAN_HPP_
#define VTM_CAN_HPP_

#include <array>
#include <cstdint>

#include "tap/communication/can/can_bus.hpp"
#include "tap/communication/can/can_rx_listener.hpp"

namespace src::communicators::vtm_can
{
/**
 * VTM <-> CAN bridge helper.
 *
 *  - Dev board sends an RTR "poll" frame at ~30 Hz (or slower).
 *  - ESP32 replies with 5 data frames containing a combined 30-byte payload.
 *
 * Response format:
 *   CAN_ID_RESP (standard 11-bit), DLC=8
 *     byte0: seq (0..255)
 *     byte1: segment index (0..4)
 *     byte2..7: 6 payload bytes
 *
 * segment 0 -> payload[0..5]
 * segment 1 -> payload[6..11]
 * segment 2 -> payload[12..17]
 * segment 3 -> payload[18..23]
 * segment 4 -> payload[24..29]
 */
class VtmCan
{
public:
    static constexpr tap::can::CanBus BUS = tap::can::CanBus::CAN_BUS1;

    // 11-bit CAN id's
    static constexpr uint16_t CAN_ID_POLL_RTR = 0x350;  // dev board -> ESP32 (RTR)
    static constexpr uint16_t CAN_ID_RESP     = 0x351;  // ESP32 -> dev board (data)

    static constexpr uint32_t PAYLOAD_LEN = 30;
    static constexpr uint8_t  NUM_SEGS = 5;
    static constexpr uint8_t  BYTES_PER_SEG = 6;

    explicit VtmCan(tap::Drivers* drivers);

    void initialize();

    void read();

    bool hasFreshPacket(uint32_t timeoutMs = 250) const;

    uint32_t getUpdateCounter() const { return updateCounter; }
    const std::array<uint8_t, PAYLOAD_LEN>& getPayload() const { return payload; }

    void setPollPeriodMs(uint32_t ms) { pollPeriodMs = ms; }

private:
    class RespListener : public tap::can::CanRxListener
    {
    public:
        RespListener(tap::Drivers* drivers, uint32_t id, tap::can::CanBus bus, VtmCan* parent);
        void processMessage(const modm::can::Message& rxMessage) override;
    private:
        VtmCan* parent;
    };

    void handleRespFrame(const modm::can::Message& rxMessage);
    void sendPollRtrIfDue();

private:
    tap::Drivers* drivers;

    RespListener respListener;

    uint8_t currentSeq = 0;
    bool segReceived[NUM_SEGS] = {false, false, false, false, false};
    uint8_t segPayload[NUM_SEGS][BYTES_PER_SEG] = {{0}};

    std::array<uint8_t, PAYLOAD_LEN> payload = {};
    uint32_t updateCounter = 0;
    uint32_t lastPacketTimeMs = 0;

    uint32_t lastPollTimeMs = 0;
    uint32_t pollPeriodMs = 33;  // ~30 Hz
};

}
#endif  // namespace src::communicators::vtm_can
