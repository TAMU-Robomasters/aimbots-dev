#pragma once
#include "utils/math/matrix_helpers.hpp"
#include "utils/tools/common_types.hpp"

//FROM reticle_indicator.hpp
#include "tap/algorithms/math_user_utils.hpp"
#include "tap/communication/referee/state_hud_indicator.hpp"
#include "tap/communication/serial/ref_serial_data.hpp"
#include "modm/processing/resumable.hpp"
#include "subsystems/display/basic_commands/hud_indicator.hpp"
/////////////////

using tap::communication::serial::RefSerialData;


RefSerialData::Tx::GraphicColor const PRIMARY_COLOR = RefSerialData::Tx::GraphicColor::GREEN;
RefSerialData::Tx::GraphicColor const SECONDARY_COLOR = RefSerialData::Tx::GraphicColor::RED_AND_BLUE;

static constexpr uint16_t NUM_RANGE_LINES = 5;
static constexpr uint16_t RANGE_LINE_WIDTH = 40;
static constexpr uint16_t RANGE_LINE_OFFSET = 10;

// looks smth like this
//      ______
//     ________
//      ______
//       ____
//      ______
//       ____
//      ______
//       ____
using ReticleTuple = std::tuple<int16_t, int16_t, tap::communication::serial::RefSerialData::Tx::GraphicColor>;
static constexpr ReticleTuple TURRET_RETICLE_X_WIDTH_AND_Y_POS_COORDINATES[]{ // This is for rangefinding reticle
    ReticleTuple(40, 545, PRIMARY_COLOR),  // 2 m
    ReticleTuple(60, 540, PRIMARY_COLOR),
    ReticleTuple(40, 535, PRIMARY_COLOR),
    ReticleTuple(10, 500, PRIMARY_COLOR),  // 4 m
    ReticleTuple(40, 495, PRIMARY_COLOR),
    ReticleTuple(10, 490, PRIMARY_COLOR),
    ReticleTuple(10, 530, PRIMARY_COLOR),  // 8 m
    ReticleTuple(30, 525, PRIMARY_COLOR),
    ReticleTuple(10, 520, PRIMARY_COLOR),
    ReticleTuple(10, 475, PRIMARY_COLOR),  // next
    ReticleTuple(40, 470, PRIMARY_COLOR),
    ReticleTuple(10, 465, PRIMARY_COLOR),
    ReticleTuple(10, 445, PRIMARY_COLOR),  // next
    ReticleTuple(40, 440, PRIMARY_COLOR),
    ReticleTuple(10, 435, PRIMARY_COLOR),
    ReticleTuple(10, 415, PRIMARY_COLOR),  // next
    ReticleTuple(40, 410, PRIMARY_COLOR),
    ReticleTuple(10, 405, PRIMARY_COLOR),
    ReticleTuple(10, 385, PRIMARY_COLOR),  // next
    ReticleTuple(40, 380, PRIMARY_COLOR),
    ReticleTuple(10, 375, PRIMARY_COLOR)
};

