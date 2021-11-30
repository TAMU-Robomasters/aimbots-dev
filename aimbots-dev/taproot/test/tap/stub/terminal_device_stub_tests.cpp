/*
 * Copyright (c) 2020-2021 Advanced Robotics at the University of Washington <robomstr@uw.edu>
 *
 * This file is part of Taproot.
 *
 * Taproot is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Taproot is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Taproot.  If not, see <https://www.gnu.org/licenses/>.
 */

#include <vector>

#include <gtest/gtest.h>

#include "modm/io/iostream.hpp"

#include "terminal_device_stub.hpp"

using namespace tap::stub;

TEST(TerminalDeviceStub, read__empty_returns_eof)
{
    TerminalDeviceStub device(nullptr);
    char c;
    EXPECT_FALSE(device.read(c));
    EXPECT_EQ(modm::IOStream::eof, c);
}

TEST(TerminalDeviceStub, read__after_emplacing_items_works)
{
    TerminalDeviceStub device(nullptr);
    char c;
    std::vector<char> itemsToEmplace = {1, 2, 3, 4, 5};
    device.emplaceItemsInReadBuffer(itemsToEmplace);
    for (char item : itemsToEmplace)
    {
        EXPECT_TRUE(device.read(c));
        EXPECT_EQ(item, c);
    }
}

TEST(TerminalDeviceStub, readItemsFromWriteBuffer__empty_returns_0)
{
    TerminalDeviceStub device(nullptr);
    std::vector<char> readItems;
    EXPECT_EQ(0, device.readItemsFromWriteBuffer(readItems, 10));
}

TEST(TerminalDeviceStub, readItemsFromWriteBuffer__returns_items_after_writing)
{
    TerminalDeviceStub device(nullptr);
    std::vector<char> readItems;
    std::vector<char> itemsToWrite = {1, 2, 3, 4, 5};
    for (char item : itemsToWrite)
    {
        device.write(item);
    }
    EXPECT_EQ(1, device.readItemsFromWriteBuffer(readItems, 1));
    EXPECT_EQ(4, device.readItemsFromWriteBuffer(readItems, 10));
    EXPECT_EQ(itemsToWrite.size(), readItems.size());
    for (std::size_t i = 0; i < readItems.size(); i++)
    {
        EXPECT_EQ(itemsToWrite[i], readItems[i]);
    }
}
