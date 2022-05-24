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

#ifndef TAPROOT_ENDIANNESS_WRAPPERS_HPP_
#define TAPROOT_ENDIANNESS_WRAPPERS_HPP_

#include <cstdint>
#include <cstring>

#include "modm/architecture/detect.hpp"

namespace tap
{
namespace arch
{
/**
 * Reads a number and stores its byte array representation in
 * the given array reference.
 *
 * @tparam T the type to be read.
 * @param[in] data the number to be read.
 * @param[out] bytesOut the reference to the array in which the
 *      bytes will be stored.
 * @param[in] forward an indication of if the number's bytes
 *      should be read forwards or not.
 */
template <typename T>
inline void dataToByteArray(T data, uint8_t *bytesOut, bool forward)
{
    int numBytes = sizeof(T);
    for (int i = 0; i < numBytes; i++)
    {
        int index = forward ? i : numBytes - 1 - i;
        uint8_t byte = (data >> (8 * index)) & 0xff;
        bytesOut[i] = byte;
    }
}

/**
 * Reads a byte array and stores its numeric value in
 * the given number reference.
 *
 * @tparam T the type to be read.
 * @param[out] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] bytesIn the byte array to be read
 * @param[in] forward an indication of if the byte array should
 *      be read forwards or not
 */
template <typename T>
inline void byteArrayToData(T *data, const uint8_t *bytesIn, bool forward)
{
    int numBytes = sizeof(T);
    for (int i = 0; i < numBytes; i++)
    {
        int index = forward ? i : numBytes - 1 - i;
        uint8_t byte = (*(bytesIn + index));
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
 * @param[out] bytesOut the reference to the array in which the
 *      bytes will be stored
 */
template <typename T>
void convertToLittleEndian(T data, uint8_t *bytesOut)
{
#if MODM_IS_LITTLE_ENDIAN
    memcpy(bytesOut, &data, sizeof(T));
#else
    dataToByteArray(data, bytesOut, false);
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
 * @param[out] bytesOut the reference to the array in which the
 *      bytes will be stored
 */
template <typename T>
void convertToBigEndian(T data, uint8_t *bytesOut)
{
#if MODM_IS_LITTLE_ENDIAN
    dataToByteArray(data, bytesOut, false);
#else
    memcpy(bytesOut, &data, sizeof(T));
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
 * @param[out] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] bytesIn the byte array to be read from little endian
 */
template <typename T>
void convertFromLittleEndian(T *data, const uint8_t *bytesIn)
{
#if MODM_IS_LITTLE_ENDIAN
    *data = *reinterpret_cast<const T *>(bytesIn);
#else
    byteArrayToData(data, bytesIn, false);
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
 * @param[out] data the reference to the number in which the
 *      byte array's numeric representation will be stored
 * @param[in] bytesIn the byte array to be read from big endian
 */
template <typename T>
void convertFromBigEndian(T *data, const uint8_t *bytesIn)
{
#if MODM_IS_LITTLE_ENDIAN
    byteArrayToData(data, bytesIn, false);
#else
    *data = *reinterpret_cast<const T *>(bytesIn);
#endif
}

/**
 * Convert int16_t stored in big endian format in buff to a floating point value.
 *
 * @param[in] buff Buffer containing two bytes representing an int16_t in big endian format.
 * @return A float, the converted int16_t in floating point form.
 */
inline float bigEndianInt16ToFloat(const uint8_t *buff)
{
    return static_cast<float>(static_cast<int16_t>((*(buff)) | (*(buff + 1) << 8)));
}

}  // namespace arch
}  // namespace tap

#endif  // TAPROOT_ENDIANNESS_WRAPPERS_HPP_
