#ifndef __BOARD_H__
#define __BOARD_H__

/*
 * This file contains definitions of board-specific variables and functions to support the
 * ArduinoPebbleSerial library.
 */

#include <stdint.h>

#define BOARD_SERIAL Serial1
static inline void board_set_tx_enabled(bool enabled) {
  if (enabled) {
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXE, DISABLE);
  } else {
    USART_ITConfig(USART2, USART_IT_TXE, ENABLE);
    USART_ITConfig(USART2, USART_IT_RXE, DISABLE);
#if 0
    bitClear(UCSR1B, TXEN1);
    bitClear(DDRD, 3);
    bitSet(PORTD, 3);
    bitSet(UCSR1B, RXEN1);
#endif
  }
}
static inline void board_set_even_parity(bool enabled) {
#if 0
  if (enabled) {
    bitSet(UCSR1C, UPM11);
  } else {
    bitClear(UCSR1C, UPM11);
  }
#endif
}

#endif // __BOARD_H__
