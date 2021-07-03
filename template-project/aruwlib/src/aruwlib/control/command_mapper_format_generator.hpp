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

#ifndef COMMAND_MAPPER_FORMAT_GENERATOR_HPP_
#define COMMAND_MAPPER_FORMAT_GENERATOR_HPP_

#include <string>
#include <string_view>
#include <vector>

#include "aruwlib/communication/remote.hpp"

namespace aruwlib
{
namespace control
{
class CommandMapper;
class RemoteMapState;
class Command;

/**
 * A utility for generating a readable format of the current command mappings in a particular
 * CommandMapper.
 */
class CommandMapperFormatGenerator
{
public:
    explicit CommandMapperFormatGenerator(const CommandMapper &mapper) : mapper(mapper) {}
    ~CommandMapperFormatGenerator() = default;

    /**
     * @return A list of mappings in string format, parsed from the CommandMapper
     *      passed into the class.
     * @note This is very slow because of the necessary std::string parsing. Never
     *      call this while performance matters.
     */
    const std::vector<std::string> generateMappings() const;

private:
    const CommandMapper &mapper;

    const std::string formattedRemoteMapState(const RemoteMapState &ms) const;
    const std::string formattedMappedCommands(const std::vector<Command *> mc) const;
    constexpr std::string_view switchStateToString(Remote::SwitchState state) const;
    const std::string keyMapToString(uint16_t keys) const;
};  // class CommandMapperFormatGenerator
}  // namespace control
}  // namespace aruwlib

#endif  // COMMAND_MAPPER_FORMAT_GENERATOR_HPP_
