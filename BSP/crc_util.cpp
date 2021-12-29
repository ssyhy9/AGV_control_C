/*
 * crc_util.cpp
 *
 *  Created on: Apr 5, 2021
 *      Author: Jason
 */
#include "CRCcrc_util.h"

#include "crc.hpp"

probability::CRC crc8(0x31, 8, 0xFF, 0x00, true, true);
probability::CRC crc16(0x1021, 16, 0xFFFF, 0x0000, true, true);

int32_t crc8compute(const void *buffer, const uint64_t size,
		uint32_t *const crc) {
	return crc8.compute(buffer, size, crc);
}

int32_t crc16compute(const void *buffer, const uint64_t size,
		uint32_t *const crc) {
	return crc16.compute(buffer, size, crc);
}
