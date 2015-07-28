#ifndef __BOARD_H__
#define __BOARD_H__

/*
 * This file contains definitions of board-specific variables and functions to support the
 * ArduinoPebbleSerial library.
 */

#include <stdint.h>

#define BOARD_SERIAL Serial1
static inline void board_set_tx_enabled(bool enabled) {
  // USART default configuration
  // USART configured as follow:
  // - BaudRate = (set baudRate as 9600 baud)
  // - Word Length = 8 Bits
  // - One Stop Bit
  // - No parity
  // - Hardware flow control disabled (RTS and CTS signals)
  // - Receive and transmit enabled
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  if (enabled)
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
  else
    USART_InitStructure.USART_Mode = USART_Mode_Rx;

  // Configure USART
  USART_Init(usartMap->usart_peripheral, &USART_InitStructure);

  // Enable USART Receive and Transmit interrupts
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

  // Enable the USART
  USART_Cmd(USART2, ENABLE);
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
