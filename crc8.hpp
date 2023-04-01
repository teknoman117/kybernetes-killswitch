/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/. */

#ifndef CRC8_H
#define CRC8_H

#include <avr/pgmspace.h>

extern const uint8_t crc8LookupTable[256] PROGMEM;

uint8_t calculateCRC8(uint8_t crc8, uint8_t *data, size_t size);

#endif /* CRC8_H */