/*
 * This is an Arduino library wrapper around the PebbleSerial library.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ParticlePebble.h"
#include "board.h"
#include "PebbleSerial.h"

static USARTSerial *s_serial = &(BOARD_SERIAL);
static uint8_t *s_buffer;
static size_t s_buffer_length;

static void prv_control_cb(PebbleControl cmd) {
  switch (cmd) {
  case PebbleControlEnableTX:
    board_set_tx_enabled(true);
    break;
  case PebbleControlDisableTX:
    board_set_tx_enabled(false);
    break;
  case PebbleControlFlushTX:
    s_serial->flush();
    delay(1);
    break;
  case PebbleControlSetParityEven:
    board_set_even_parity(true);
    break;
  case PebbleControlSetParityNone:
    board_set_even_parity(false);
    break;
  default:
    break;
  }
}

static void prv_write_byte_cb(uint8_t data) {
  s_serial->write(data);
}

void ParticlePebble::begin(uint8_t *buffer, size_t length) {
  s_buffer = buffer;
  s_buffer_length = length;
  s_serial->begin(PEBBLE_DEFAULT_BAUDRATE);

  PebbleCallbacks callbacks = {
    .write_byte = prv_write_byte_cb,
    .control = prv_control_cb
  };
  pebble_init(callbacks);
  pebble_prepare_for_read(s_buffer, s_buffer_length);
}

bool ParticlePebble::feed(size_t *length, bool *is_read) {
  while (s_serial->available()) {
    uint8_t data = (uint8_t)s_serial->read();
    if (pebble_handle_byte(data, length, is_read)) {
      // we have a full frame
      pebble_prepare_for_read(s_buffer, s_buffer_length);
      return true;
    }
  }
  return false;
}

bool ParticlePebble::write(const uint8_t *payload, size_t length) {
  return pebble_write(payload, length);
}

void ParticlePebble::notify(void) {
  pebble_notify();
}

bool ParticlePebble::is_connected(void) {
  return pebble_is_connected();
}
