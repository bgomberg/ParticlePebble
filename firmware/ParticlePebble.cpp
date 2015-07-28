/*
 * This is an Arduino library wrapper around the PebbleSerial library.
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "ParticlePebble.h"
#include "board.h"

extern "C" {

#define PEBBLE_DEFAULT_BAUDRATE 9600
#define PEBBLE_PROTOCOL_VERSION 1
#define PEBBLE_MAX_PAYLOAD      80

typedef enum {
  PebbleControlEnableTX,
  PebbleControlDisableTX,
  PebbleControlFlushTX,
  PebbleControlSetParityEven,
  PebbleControlSetParityNone
} PebbleControl;

typedef void (*PebbleWriteByteCallback)(uint8_t data);
typedef void (*PebbleControlCallback)(PebbleControl cmd);

typedef struct {
  PebbleWriteByteCallback write_byte;
  PebbleControlCallback control;
} PebbleCallbacks;


static const uint8_t HDLC_FLAG = 0x7E;
static const uint8_t HDLC_ESCAPE = 0x7D;
static const uint8_t HDLC_ESCAPE_MASK = 0x20;

typedef struct {
  bool escape;
  bool is_valid;
} HdlcStreamingContext;


void hdlc_streaming_decode_start(HdlcStreamingContext *ctx) {
  ctx->escape = false;
  ctx->is_valid = true;
}

bool hdlc_streaming_decode(HdlcStreamingContext *ctx, uint8_t *data) {
  if (!ctx->is_valid) {
    // if this stream is invalid, no need to process any more
    return false;
  }
  if (*data == HDLC_FLAG) {
    // there shouldn't be any flags in the data
    ctx->is_valid = false;
    return false;
  } else if (*data == HDLC_ESCAPE) {
    if (ctx->escape) {
      // invalid sequence
      ctx->is_valid = false;
      return false;
    } else {
      // next character must be escaped and this one should be ignored
      ctx->escape = true;
      return false;
    }
  } else {
    // this is a valid character
    if (ctx->escape) {
      // unescape the current character before processing it
      *data ^= HDLC_ESCAPE_MASK;
      ctx->escape = false;
    }
    return true;
  }
}

bool hdlc_streaming_decode_finish(HdlcStreamingContext *ctx) {
  return ctx->is_valid && !ctx->escape;
}

bool hdlc_encode(uint8_t *data) {
  if (*data == HDLC_FLAG || *data == HDLC_ESCAPE) {
    *data ^= HDLC_ESCAPE_MASK;
    return true;
  }
  return false;
}

#define UNLIKELY(x) (__builtin_expect(!!(x), 0))

#define FLAGS_VERSION_MASK     ((uint8_t)0x0F)
#define FLAGS_IS_READ_MASK     ((uint8_t)0x10)
#define FLAGS_IS_MASTER_MASK   ((uint8_t)0x20)
#define FLAGS_RESERVED_MASK    ((uint8_t)0xC0)
#define FLAGS_GET(flags, mask) (((flags) & mask) >> __builtin_ctz(mask))
#define FLAGS_SET(flags, mask, value) \
  (flags) = ((flags) & ~mask) | (((value) << __builtin_ctz(mask)) & mask)

#define FRAME_MIN_LENGTH              3
#define FRAME_FLAGS_OFFSET            0
#define FRAME_PROTOCOL_OFFSET         1
#define FRAME_PAYLOAD_OFFSET          2
#define LINK_CONTROL_VERSION          1

typedef enum {
  PebbleProtocolInvalid = 0x00,
  PebbleProtocolLinkControl = 0x01,
  PebbleProtocolRawData = 0x02,
  PebbleProtocolNum
} PebbleProtocol;

typedef enum {
  LinkControlConnectionRequest = 0x01
} LinkControlType;

typedef struct {
  uint8_t *payload;
  uint8_t protocol;
  uint8_t parity;
  size_t length;
  size_t read_length;
  uint8_t parity_buffer;
  bool should_drop;
  bool read_ready;
  bool is_read;
} PebbleFrameInfo;

static HdlcStreamingContext s_hdlc_ctx;
static PebbleFrameInfo s_frame;
static PebbleCallbacks s_callbacks;
static bool s_connected;
static bool s_can_respond;

void pebble_init(PebbleCallbacks callbacks) {
  s_callbacks = callbacks;
  s_callbacks.control(PebbleControlSetParityNone);
  s_callbacks.control(PebbleControlEnableTX);
  s_callbacks.control(PebbleControlDisableTX);
}

void pebble_prepare_for_read(uint8_t *buffer, size_t length) {
  hdlc_streaming_decode_start(&s_hdlc_ctx);
  s_frame = (PebbleFrameInfo) { 0 };
  s_frame.payload = buffer;
  s_frame.read_length = length;
  s_frame.read_ready = true;
}

static void prv_send_flag(void) {
  s_callbacks.write_byte(HDLC_FLAG);
}

static void prv_send_byte(uint8_t data, uint8_t *parity) {
  *parity ^= data;
  if (hdlc_encode(&data)) {
    s_callbacks.write_byte(HDLC_ESCAPE);
  }
  s_callbacks.write_byte(data);
}

static void prv_write_internal(PebbleProtocol protocol, const uint8_t *data, size_t length) {
  uint8_t parity = 0;

  // enable tx
  s_callbacks.control(PebbleControlEnableTX);

  // send flag
  prv_send_flag();

  // send header flags
  uint8_t flags = 0;
  FLAGS_SET(flags, FLAGS_VERSION_MASK, PEBBLE_PROTOCOL_VERSION);
  prv_send_byte(flags, &parity);

  // send protocol
  prv_send_byte((uint8_t)protocol, &parity);

  // send data
  size_t i;
  for (i = 0; i < length; i++) {
    prv_send_byte(data[i], &parity);
  }

  // send parity
  prv_send_byte(parity, &parity);

  // send flag
  prv_send_flag();

  // flush and disable tx
  s_callbacks.control(PebbleControlFlushTX);
  s_callbacks.control(PebbleControlDisableTX);
}

static void prv_handle_link_control(uint8_t *buffer) {
  LinkControlType type = (LinkControlType) buffer[0];
  if (type == LinkControlConnectionRequest) {
    uint8_t version = buffer[1];
    if (version == LINK_CONTROL_VERSION) {
      // re-use the buffer for the response
      buffer[1] = PebbleProtocolRawData;
      prv_write_internal(PebbleProtocolLinkControl, buffer, 2);
      s_connected = true;
    }
  }
}

bool pebble_handle_byte(uint8_t data, size_t *length, bool *is_read) {
  if (!s_frame.read_ready) {
    // we aren't ready to read from the device yet
    return false;
  }

  s_can_respond = false;
  if (UNLIKELY(data == HDLC_FLAG)) {
    if (s_frame.length == 0) {
      // this is the starting flag, so just ignore it
      return false;
    } else if (UNLIKELY(s_frame.should_drop)) {
      // we already know it's invalid
    } else if (UNLIKELY(hdlc_streaming_decode_finish(&s_hdlc_ctx) == false)) {
      // decoding error
      s_frame.should_drop = true;
    } else if (UNLIKELY(s_frame.length < FRAME_MIN_LENGTH)) {
      // this frame was too small
      s_frame.should_drop = true;
    } else if (UNLIKELY(s_frame.parity != 0)) {
      // parity error
      s_frame.should_drop = true;
    }

    if (s_frame.should_drop) {
      // drop this frame and prepare for the next one
      pebble_prepare_for_read(s_frame.payload, s_frame.read_length);
    } else if (s_frame.protocol == PebbleProtocolLinkControl) {
      // handle this link control frame
      prv_handle_link_control(s_frame.payload);
      // prepare for the next frame
      pebble_prepare_for_read(s_frame.payload, s_frame.read_length);
    } else {
      // this is a valid frame
      s_frame.read_ready = false;
      *length = s_frame.length - FRAME_MIN_LENGTH;
      *is_read = s_frame.is_read;
      s_can_respond = *is_read;
      return true;
    }
  } else if (UNLIKELY(s_frame.should_drop)) {
    // this frame has already been determined to be invalid
  } else if (hdlc_streaming_decode(&s_hdlc_ctx, &data)) {
    // process this byte
    s_frame.parity ^= data;
    if (s_frame.length == FRAME_FLAGS_OFFSET) {
      // this is the flags octet
      if (UNLIKELY(FLAGS_GET(data, FLAGS_VERSION_MASK) != PEBBLE_PROTOCOL_VERSION)) {
        // invalid version
        s_frame.should_drop = true;
      } else if (UNLIKELY(FLAGS_GET(data, FLAGS_IS_MASTER_MASK) != 1)) {
        // we should only be geting frames from the slave
        s_frame.should_drop = true;
      } else if (UNLIKELY(FLAGS_GET(data, FLAGS_RESERVED_MASK) != 0)) {
        // the reserved flags should be set to 0
        s_frame.should_drop = true;
      } else {
        // the header flags are valid
        s_frame.is_read = FLAGS_GET(data, FLAGS_IS_READ_MASK);
      }
    } else if (s_frame.length == FRAME_PROTOCOL_OFFSET) {
      // this is the protocol octet
      switch (data) {
      case PebbleProtocolLinkControl:
      case PebbleProtocolRawData:
        // valid protocol
        s_frame.protocol = data;
        break;
      case PebbleProtocolInvalid:
      default:
        // invalid protocol
        s_frame.should_drop = true;
        break;
      }
    } else {
      const size_t payload_length = s_frame.length - FRAME_PAYLOAD_OFFSET;
      if (payload_length > s_frame.read_length) {
        // no space in the receive buffer for this byte
        s_frame.should_drop = true;
      } else {
        // keep a 1 byte buffer in memory so we don't write the parity into the payload
        if (payload_length > 0) {
          // put the previous byte into the payload buffer
          s_frame.payload[payload_length-1] = s_frame.parity_buffer;
        }
        s_frame.parity_buffer = data;
      }
    }
    // increment the length
    s_frame.length++;
  }
  return false;
}

bool pebble_write(const uint8_t *data, size_t length) {
  if (!s_can_respond) {
    return false;
  }
  prv_write_internal(PebbleProtocolRawData, data, length);
  s_can_respond = false;
  return true;
}

void pebble_notify(void) {
  s_callbacks.control(PebbleControlEnableTX);
  s_callbacks.control(PebbleControlSetParityEven);
  s_callbacks.write_byte(0x00);
  s_callbacks.write_byte(0x00);
  s_callbacks.write_byte(0x00);
  // we must flush before changing the parity back
  s_callbacks.control(PebbleControlFlushTX);
  s_callbacks.control(PebbleControlDisableTX);
  s_callbacks.control(PebbleControlSetParityNone);
}

bool pebble_is_connected(void) {
  return s_connected;
}






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
};

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
