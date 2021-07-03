/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of aruwlib.
 *
 * aruwlib is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * aruwlib is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with aruwlib.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef ERROR_CONTROLLER_HPP_
#define ERROR_CONTROLLER_HPP_

#include "aruwlib/architecture/timeout.hpp"
#include "aruwlib/communication/serial/terminal_serial.hpp"
#include "aruwlib/util_macros.hpp"

#include "modm/container.hpp"

#include "system_error.hpp"

namespace aruwlib
{
class Drivers;
namespace errors
{
/**
 * The ErrorController stores the errors that are currently active and displays errors
 * via the MCB's LEDs.
 *
 * Use the `RAISE_ERROR` macro to add errors to the main ErrorController.
 *
 * LED blink Protocol description:
 * - The 8 LEDs on the MCB are used to indicate the location of an error. LED A is the LSB
 *   and LED H is the MSB.
 * - The other green LED (next to the red LED) comes on when you have added an error that
 *   is invalid. The red LED is not used by the ErrorController.
 * - By default, LEDs A-H are always off if no errors are detected.
 */
class ErrorController : public aruwlib::communication::serial::ITerminalSerialCallback
{
public:
    static constexpr std::size_t ERROR_LIST_MAX_SIZE = 16;
    using error_index_t = modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE>::Index;

    /**
     * Constrcuts an ErrorController with a display time for each error specified
     * by `ERROR_ROTATE_TIME`.
     */
    ErrorController(Drivers* drivers)
        : drivers(drivers),
          prevLedErrorChangeWait(ERROR_ROTATE_TIME),
          currentDisplayIndex(0)
    {
    }
    DISALLOW_COPY_AND_ASSIGN(ErrorController)
    mockable ~ErrorController() = default;

    /**
     * Adds the passed in error to the ErrorController if no identical errors are already in
     * the ErrorController.
     *
     * @param[in] error The SystemError to add to the ErrorController.
     */
    mockable void addToErrorList(const SystemError& error);

    /**
     * Updates the errors displayed via the LEDs. Cycles through the `SystemErrors` in the
     * queue of errors, switching to a new error every `ERROR_ROTATE_TIME`.
     */
    mockable void updateLedDisplay();

    void init();

    /**
     * @param[in] inputLine The user input to be processed.
     * @param[out] outputStream The stream to write information to.
     * @return `true` if the inputLine was valid and was parsed correctly, `false` otherwise.
     */
    bool terminalSerialCallback(char* inputLine, modm::IOStream& outputStream, bool) override;

    void terminalSerialStreamCallback(modm::IOStream&) override {}

private:
    friend class ErrorControllerTester;

    static constexpr int ERROR_ROTATE_TIME = 5000;
    static constexpr error_index_t NUM_LEDS = 8;

    Drivers* drivers;

    modm::BoundedDeque<SystemError, ERROR_LIST_MAX_SIZE> errorList;

    aruwlib::arch::MilliTimeout prevLedErrorChangeWait;

    error_index_t currentDisplayIndex;

    bool removeSystemErrorAtIndex(error_index_t index);

    void removeAllSystemErrors();

    void displayAllErrors(modm::IOStream& outputStream);

    void removeTerminalError(int index, modm::IOStream& outputStream);

    void clearAllTerminalErrors(modm::IOStream& outputStream);

    void help(modm::IOStream& outputStream);

    /**
     * Displays the `binaryRep` with the LEDs A-H, with LED A as the LSB and LED H as the MSB.
     */
    void displayBinaryNumberViaLeds(uint8_t binaryRep);

    bool validateErrorTypeAndLocation(const SystemError& error);
};  // class ErrorController
}  // namespace errors
}  // namespace aruwlib

#endif  // ERROR_CONTROLLER_HPP_
