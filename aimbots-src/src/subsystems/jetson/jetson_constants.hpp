#pragma once

#include "utils/tools/robot_specific_defines.hpp"

#if defined(ALL_STANDARDS)
#include "robots/standard/constants/standard_jetson_constants.hpp"

#elif defined(ALL_HEROES)
#include "robots/hero/constants/hero_jetson_constants.hpp"

#elif defined(ALL_SENTRIES)
#include "robots/sentry/constants/sentry_jetson_constants.hpp"

#elif defined(ALL_ENGINEERS)
#include "robots/engineer/constants/engineer_jetson_constants.hpp"

#elif defined(ALL_AERIALS)
#include "robots/aerial/constants/aerial_jetson_constants.hpp"

#elif defined(ALL_DARTS)
#include "robots/dart/constants/dart_jetson_constants.hpp"

#elif defined(ALL_TESTBENCHES)
#include "robots/testbench/constants/testbench_jetson_constants.hpp"

#elif defined(ALL_TURRETS)
#include "robots/turret/constants/turret_jetson_constants.hpp"

#endif

#ifdef JETSON_COMPATIBLE // Currently all Jetson compatible robots use the same UART port

#include <drivers.hpp>
#include <utils/tools/common_types.hpp>

#define READ(data, length) drivers->uart.read(JETSON_UART_PORT, data, length)
#define WRITE(data, length) drivers->uart.write(JETSON_UART_PORT, data, length)
#define INIT_UART() drivers->uart.init<JETSON_UART_PORT, JETSON_BAUD_RATE>();

constexpr UartPort JETSON_UART_PORT = UartPort::Uart1;
constexpr uint32_t JETSON_BAUD_RATE = 115200;
constexpr uint8_t jetsonMessageHeader = 'j';

#endif