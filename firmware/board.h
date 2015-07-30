#ifndef __BOARD_H__
#define __BOARD_H__

/*
 * This file contains definitions of board-specific variables and functions to support the
 * ArduinoPebbleSerial library.
 */

#include "application.h"
#include <stdint.h>


#define BOARD_SERIAL Serial1
static inline void board_set_tx_enabled(bool enabled) {
}
static inline void board_set_even_parity(bool enabled) {
}

#endif // __BOARD_H__
