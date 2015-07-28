#ifndef __BOARD_H__
#define __BOARD_H__

/*
 * This file contains definitions of board-specific variables and functions to support the
 * ArduinoPebbleSerial library.
 */

#include "application.h"
#include <stdint.h>
#include "hw_config.h"
#include "platform_config.h"

static bool tx_enabled = false;
static bool parity = false;
static void do_update(void) {
  pinMode(TX, INPUT_PULLUP);
  // USART default configuration
  // USART configured as follow:
  // - BaudRate = (set baudRate as 9600 baud)
  // - Word Length = 8 Bits
  // - One Stop Bit
  // - No parity
  // - Hardware flow control disabled (RTS and CTS signals)
  // - Receive and transmit enabled
  USART_InitTypeDef USART_InitStructure;
  USART_InitStructure.USART_BaudRate = 9600;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  if (parity)
    USART_InitStructure.USART_Parity = USART_Parity_No;
  else
    USART_InitStructure.USART_Parity = USART_Parity_Even;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

  if (tx_enabled)
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
  else
    USART_InitStructure.USART_Mode = USART_Mode_Rx;

  // Configure USART
  USART_Init(USART2, &USART_InitStructure);

  // Enable USART Receive and Transmit interrupts
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);
  USART_ITConfig(USART2, USART_IT_TXE, ENABLE);

  // Enable the USART
  USART_Cmd(USART2, ENABLE);
}
#define BOARD_SERIAL Serial1
static inline void board_set_tx_enabled(bool enabled) {
  tx_enabled = enabled;
  do_update();
}
static inline void board_set_even_parity(bool enabled) {
  parity = enabled;
  do_update();
}

#endif // __BOARD_H__
