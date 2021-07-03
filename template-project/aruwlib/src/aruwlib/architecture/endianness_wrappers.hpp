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

#ifndef ENDIANNESS_WRAPPERS
#define ENDIANNESS_WRAPPERS

#include <cstdint>
#include <cstring>

namespace aruwlib
{
namespace arch
{
/**
 * Reads a number and stores its byte array representation in
 * the given array reference.
 *
 * @tparam T the type to be read.
 * @param[in] data the number to be read.
 * @param[in] dataOut the reference to the array in which the
 *      bytes will be stored.
 * @param[in] forward an indication of if the number's bytes
 *      should be read forwards or not.
 */
template <typename T>
inline void dataToByteArray(T data, uint8_t *dataOut, bool forward)
{
    int numBytes = sizeof(T);
    for (int i = 0; i < numBytes; i++)
    {
        int index = forward ? i : numBytes - 1 - i;
        uint8_t byte = (data >> (8 * index)) & 0xff;
        dataOut[i] = byte;
    }
}

/**
 * Reads a byte array and stores its numeric value in
 * the given number reference.
 *
 * @tparam T the type to be read.
 * @param[in] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] dataIn the byte array to be read
 * @param[in] forward an indication of if the byte array should
 *      be read forwards or not
 */
template <typename T>
inline void byteArrayToData(T *data, const uint8_t *dataIn, bool forward)
{
    int numBytes = sizeof(T);
    for (int i = 0; i < numBytes; i++)
    {
        int index = forward ? i : numBytes - 1 - i;
        uint8_t byte = (*(dataIn + index));
        *(reinterpret_cast<uint8_t *>(data) + i) = byte;
    }
}

/**
 * Converts a number into a byte array in little endian.
 * If the current architecture is in little endian, the bytes
 * are maintained in the current order.
 * If the current architecture is in big endian, the bytes
 * are read and stored in reverse order.
 *
 * @tparam T the type to be read.
 * @param[in] data the number to be read
 * @param[in] dataOut the reference to the array in which the
 *      bytes will be stored
 */
template <typename T>
void convertToLittleEndian(T data, uint8_t *dataOut)
{
#ifdef LITTLE_ENDIAN
    memcpy(dataOut, &data, sizeof(T));
#else
    dataToByteArray(data, dataOut, false);
#endif
}

/**
 * Converts a number into a byte array in big endian.
 * If the current architecture is in big endian, the bytes
 * are maintained in the current order.
 * If the current architecture is in little endian, the bytes
 * are read and stored in reverse order.
 *
 * @tparam T the type to be read.
 * @param[in] data the number to be read
 * @param[in] dataOut the reference to the array in which the
 *      bytes will be stored
 */
template <typename T>
void convertToBigEndian(T data, uint8_t *dataOut)
{
#ifdef LITTLE_ENDIAN
    dataToByteArray(data, dataOut, false);
#else
    memcpy(dataOut, &data, sizeof(T));
#endif
}

/**
 * Converts a byte array from little endian and stores its
 * numeric value in the given number reference.
 * If the current architecture is in little endian, the bytes
 * are maintained in the current order.
 * If the current architecture is in big endian, the bytes
 * are read and stored in reverse order.
 *
 * @tparam T the type to be read.
 * @param[in] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] dataIn the byte array to be read from little endian
 */
template <typename T>
void convertFromLittleEndian(T *data, const uint8_t *dataIn)
{
#ifdef LITTLE_ENDIAN
    *data = *reinterpret_cast<const T *>(dataIn);
#else
    byteArrayToData(data, dataIn, false);
#endif
}

/**
 * Converts a byte array from big endian and stores its
 * numeric value in the given number reference.
 * If the current architecture is in big endian, the bytes
 * are maintained in the current order.
 * If the current architecture is in little endian, the bytes
 * are read and stored in reverse order.
 *
 * @tparam T the type to be read.
 * @param[in] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] dataIn the byte array to be read from big endian
 */
template <typename T>
void convertFromBigEndian(T *data, const uint8_t *dataIn)
{
#ifdef LITTLE_ENDIAN
    byteArrayToData(data, dataIn, false);
#else
    *data = *reinterpret_cast<const T *>(dataIn);
#endif
}

}  // namespace arch
}  // namespace aruwlib

#endif