#include "duml_codec.h"

#include <cstring>

namespace osmo::wire {

uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t c = 0x77;
    for (size_t i = 0; i < len; ++i) {
        c ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            c = (c & 1) ? static_cast<uint8_t>((c >> 1) ^ 0x8C)
                        : static_cast<uint8_t>(c >> 1);
        }
    }
    return c;
}

uint16_t crc16(const uint8_t *data, size_t len) {
    uint16_t c = 0x3692;
    for (size_t i = 0; i < len; ++i) {
        c ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit) {
            c = (c & 1) ? static_cast<uint16_t>((c >> 1) ^ 0x8408)
                        : static_cast<uint16_t>(c >> 1);
        }
    }
    return c;
}

size_t encodeFrame(uint8_t *out, size_t capacity,
                   uint8_t sender, uint8_t receiver, uint16_t seq,
                   uint8_t flags, uint8_t cmdSet, uint8_t cmdId,
                   const uint8_t *payload, size_t payloadLen) {
    const size_t total = 13 + payloadLen;
    if (!out || total > capacity || total > 0x3FF ||
        (payloadLen && !payload)) {
        return 0;
    }

    out[0] = 0x55;
    out[1] = static_cast<uint8_t>(total);
    out[2] = static_cast<uint8_t>((1u << 2) | ((total >> 8) & 0x03));
    out[3] = crc8(out, 3);
    out[4] = sender;
    out[5] = receiver;
    out[6] = static_cast<uint8_t>(seq);
    out[7] = static_cast<uint8_t>(seq >> 8);
    out[8] = flags;
    out[9] = cmdSet;
    out[10] = cmdId;
    if (payloadLen) std::memcpy(out + 11, payload, payloadLen);
    const uint16_t check = crc16(out, total - 2);
    out[total - 2] = static_cast<uint8_t>(check);
    out[total - 1] = static_cast<uint8_t>(check >> 8);
    return total;
}

bool decodeFrame(const uint8_t *data, size_t len, FrameView &out) {
    if (!data || len < 13 || data[0] != 0x55) return false;
    const size_t total = data[1] | ((data[2] & 0x03u) << 8);
    if ((data[2] >> 2) != 1 || total < 13 || total > len) return false;
    if (crc8(data, 3) != data[3]) return false;
    const uint16_t got = data[total - 2] | (data[total - 1] << 8);
    if (crc16(data, total - 2) != got) return false;

    out.sender = data[4];
    out.receiver = data[5];
    out.seq = data[6] | (data[7] << 8);
    out.flags = data[8];
    out.cmdSet = data[9];
    out.cmdId = data[10];
    out.payload = data + 11;
    out.payloadLen = total - 13;
    out.totalLen = total;
    return true;
}

bool scanFrame(const uint8_t *data, size_t len, size_t &offset,
               FrameView &out) {
    if (!data) return false;
    for (size_t i = 0; i + 13 <= len; ++i) {
        if (data[i] == 0x55 && decodeFrame(data + i, len - i, out)) {
            offset = i;
            return true;
        }
    }
    return false;
}

}  // namespace osmo::wire
