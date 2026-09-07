#pragma once

#include <Arduino.h>

namespace osmo::wire {

// A non-owning view of one CRC-valid DUML frame. The payload points into the
// caller's buffer and remains valid only while that buffer does.
struct FrameView {
    uint8_t sender = 0;
    uint8_t receiver = 0;
    uint16_t seq = 0;
    uint8_t flags = 0;
    uint8_t cmdSet = 0;
    uint8_t cmdId = 0;
    const uint8_t *payload = nullptr;
    size_t payloadLen = 0;
    size_t totalLen = 0;
};

uint8_t crc8(const uint8_t *data, size_t len);
uint16_t crc16(const uint8_t *data, size_t len);

// Encode one DUML frame. Returns zero if the output buffer is too small or the
// 10-bit DUML length field cannot represent the frame.
size_t encodeFrame(uint8_t *out, size_t capacity,
                   uint8_t sender, uint8_t receiver, uint16_t seq,
                   uint8_t flags, uint8_t cmdSet, uint8_t cmdId,
                   const uint8_t *payload = nullptr, size_t payloadLen = 0);

// Decode a frame at data[0]. Both CRCs are checked. Incomplete, malformed and
// corrupt input all return false and never expose a partial frame.
bool decodeFrame(const uint8_t *data, size_t len, FrameView &out);

// Find the first valid frame inside a UDP datagram or BLE receive buffer.
// `offset` is set to the frame's first byte.
bool scanFrame(const uint8_t *data, size_t len, size_t &offset, FrameView &out);

}  // namespace osmo::wire
