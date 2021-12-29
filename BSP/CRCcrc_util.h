/*
 * crc_util.h
 *
 *  Created on: Apr 5, 2021
 *      Author: Jason
 */

#ifndef CRC_CRC_UTIL_H_
#define CRC_CRC_UTIL_H_

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int32_t crc8compute(const void *buffer, const uint64_t size,
		uint32_t *const crc);

int32_t crc16compute(const void *buffer, const uint64_t size,
		uint32_t *const crc);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* CRC_CRC_UTIL_H_ */
