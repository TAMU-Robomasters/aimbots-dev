// #pragma once

// #include "tap/algorithms/math_user_utils.hpp"
// #include "tap/communication/referee/state_hud_indicator.hpp"
// #include "tap/communication/serial/ref_serial_data.hpp"


// #include "modm/processing/resumable.hpp"

// #include "hud_indicator.hpp"
// #include "informants/hit_Tracker.hpp"

// namespace tap {
// class Drivers;
// }

// namespace src::Utils::ClientDisplay{
// class HitIndicator : public HudIndicatpr, protected modm::Resumble<2> {
// public:
//     modm::ResumableResult<bool> sendInitialGraphics() override final;

//     modm::ResumableResult<bool> update() override final;

//     void initialize() override final;
// };
// }