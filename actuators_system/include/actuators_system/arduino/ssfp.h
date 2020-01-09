// Inspired by Serial Framing Protocol [https://github.com/BaroboRobotics/libsfp/wiki/Serial-Framing-Protocol],
// this library tries to make serial communication even simpler, hence its name: Simple Serial Framing Protocol
// It uses SSFP_FLAG_BYTE to start and finish a message, and SSFP_ESCAPE_BYTE to scape both SSFP_FLAG_BYTE
// and SSFP_ESCAPE_BYTE itself. First byte is always a sequence id that allows missing frames checking. Two last
// bytes are a CRC that allows data corruption checking.
// freal@us.es
#ifndef SSFP_H
#define SSFP_H

#include <stdio.h>
#include <stdint.h>

#define SSFP_FLAG_BYTE   0x7E
#define SSFP_ESCAPE_BYTE 0x7D
// #def SSFP_ESCAPING_XOR_BYTE   0x20
#define SSFP_ESCAPED_FLAG_BYTE   0x5E
#define SSFP_ESCAPED_ESCAPE_BYTE 0x5D
#define SSFP_SEQ_AND_CRC_SIZE 3
#define SSFP_CRC_INIT 0xFFFF
#define SSFP_CRC_GOOD 0xF0B8
// A CRC updated over its bitwise complement,
// least significant byte first, results in
// this SSFP_CRC_GOOD value.

// There are smarter ways, but imply struct packing, 
// padding, endianess... sensitive stuff!
inline size_t t8_to_buffer(uint8_t data, uint8_t *buffer) {
    *(buffer++) = data;
    return 1;
}
inline size_t t8_from_buffer(uint8_t *data, uint8_t *buffer) {
    *data = buffer[0];
    return 1;
}

inline size_t t16_to_buffer(uint16_t data, uint8_t *buffer) {
    *(buffer++) = (uint8_t)(data & 0xFF);
    *(buffer++) = (uint8_t)(data >> 8);
    return 2;
}
inline size_t t16_from_buffer(uint16_t *data, uint8_t *buffer) {
    *data = buffer[0] | (buffer[1] << 8);
    return 2;
}
// Automate n bytes de/marshalling in for loop?
// for (size_t i = 0; i < sizeof(type); ++i) {
//     *(buffer++) = (uint8_t)((data >> 8*i) & (0xFF));
//     *data |= (uint8_t)(data << 8*i);
// }
// return sizeof(data);
// return sizeof(type);

// From avr-libc:
inline uint16_t crc_ccitt_update (uint16_t crc, uint8_t octet) {
  octet ^= crc & 0xff;
  octet ^= octet << 4;
  return ((((uint16_t)octet << 8) | ((crc >> 8) & 0xff)) ^ (uint8_t)(octet >> 4) ^ ((uint16_t)octet << 3));
}

class OnReadInterface {
public:
    // return true if succeeded, false otherwise:
    virtual bool call(uint8_t *buffer, size_t size) = 0;
};

struct DeframerError {
    bool crc      : 1;  // crcs do not match
    bool lost     : 1;  // cannot sync
    bool sequence : 1;  // unexpected sequence, probably missing msgs
    bool nullback : 1;  // callback is null, call connect on Deframer
    bool callback : 1;  // connected callback did not succeed

    bool any() {
        return crc || lost || sequence || nullback || callback;
    }

    size_t size() {
        return 1;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data = 0;
        packed_data |= crc?      (uint8_t)(1 << 0): 0;
        packed_data |= lost?     (uint8_t)(1 << 1): 0;
        packed_data |= sequence? (uint8_t)(1 << 2): 0;
        packed_data |= nullback? (uint8_t)(1 << 3): 0;
        packed_data |= callback? (uint8_t)(1 << 4): 0;
        index += t8_to_buffer(packed_data, &buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data;
        index += t8_from_buffer(&packed_data, &buffer[index]);
        crc      = packed_data & (uint8_t)(1 << 0);
        lost     = packed_data & (uint8_t)(1 << 1);
        sequence = packed_data & (uint8_t)(1 << 2);
        nullback = packed_data & (uint8_t)(1 << 3);
        callback = packed_data & (uint8_t)(1 << 4);
        return index;
    }

    void print() {
        printf("crc:      %s\n", crc?      "true": "false");
        printf("lost:     %s\n", lost?     "true": "false");
        printf("sequence: %s\n", sequence? "true": "false");
        printf("nullback: %s\n", nullback? "true": "false");
        printf("callback: %s\n", callback? "true": "false");

    }
};

class Framer {
public:
    Framer() = default;

    inline size_t load(const uint8_t byte, uint8_t *tx_buffer) {
        size_t loaded_bytes = 0;
        if (byte == SSFP_FLAG_BYTE) {
            tx_buffer[loaded_bytes++] = SSFP_ESCAPE_BYTE;
            tx_buffer[loaded_bytes++] = SSFP_ESCAPED_FLAG_BYTE;
        } else if (byte == SSFP_ESCAPE_BYTE) {
            tx_buffer[loaded_bytes++] = SSFP_ESCAPE_BYTE;
            tx_buffer[loaded_bytes++] = SSFP_ESCAPED_ESCAPE_BYTE;            
        } else {
            tx_buffer[loaded_bytes++] = byte;
        }
        return loaded_bytes;
    }

    size_t frame(const uint8_t *msg_buffer, size_t size, uint8_t *tx_buffer) {
        tx_buffer[0] = SSFP_FLAG_BYTE;
        size_t loaded_bytes = 1;

        loaded_bytes += load(sequence, &tx_buffer[loaded_bytes]);
        uint16_t crc = crc_ccitt_update(SSFP_CRC_INIT, sequence);
        sequence++;
 
        for (size_t i = 0; i < size; i++) {
            uint8_t tx = msg_buffer[i];
            loaded_bytes += load(tx, &tx_buffer[loaded_bytes]);
            crc = crc_ccitt_update(crc, tx);
        }        

        crc = ~crc;
        for (size_t i = 0; i < t16_to_buffer(crc, crc_buffer); i++) {
            loaded_bytes += load(crc_buffer[i], &tx_buffer[loaded_bytes]);
        }

        tx_buffer[loaded_bytes++] = SSFP_FLAG_BYTE;

        return loaded_bytes;
    }

protected:
    uint8_t sequence = 0;
    uint8_t crc_buffer[2];
};

class Deframer {
public:
    Deframer() = default;

    enum State { LOST, SYNCING, ESCAPING, LOADING };

    void connect(OnReadInterface *cb) {
        callback = cb;
    }

    DeframerError get_error() { return error; }

    void deframe(const uint8_t *msg_buffer, size_t size, uint8_t *rx_buffer) {
        for (size_t i = 0; i < size; i++) {
            uint8_t rx = msg_buffer[i];

            switch (state) {
                case LOST:
                    if (rx == SSFP_FLAG_BYTE) {
                        state = SYNCING;
                        error.lost = 0;
                    } else {
                        error.lost = 1;
                    }
                    break;

                case SYNCING:
                    if (rx == SSFP_FLAG_BYTE) {
                        state = SYNCING;  // Stay!
                    } else if (rx == SSFP_ESCAPE_BYTE) {
                        loaded_bytes = 0;
                        state = ESCAPING;
                    } else {
                        loaded_bytes = 0;
                        // Push into buffer
                        rx_buffer[loaded_bytes++] = rx;
                        state = LOADING;
                    }
                    break;

                case ESCAPING:
                    if (rx == SSFP_FLAG_BYTE) {
                        state = SYNCING;
                    } else if (rx == SSFP_ESCAPED_FLAG_BYTE) {
                        rx_buffer[loaded_bytes++] = SSFP_FLAG_BYTE;
                        state = LOADING;
                    } else if (rx == SSFP_ESCAPED_ESCAPE_BYTE) {
                        rx_buffer[loaded_bytes++] = SSFP_ESCAPE_BYTE;
                        state = LOADING;
                    } else {
                        state = LOST;
                    }
                    break;

                case LOADING:
                    if (rx == SSFP_FLAG_BYTE) {
                        uint8_t expected_sequence = last_sequence + 1;
                        error.sequence = (expected_sequence != rx_buffer[0]);
                        last_sequence = rx_buffer[0];

                        uint16_t crc = crc_ccitt_update(SSFP_CRC_INIT, rx_buffer[0]);
                        for (size_t j = 1; j < loaded_bytes; j++) {
                            crc = crc_ccitt_update(crc, rx_buffer[j]);
                        }
                        error.crc = (crc != SSFP_CRC_GOOD);

                        error.nullback = (!callback);                        
                        if (!error.nullback && !error.crc) {
                            error.callback = !callback->call(&rx_buffer[1], loaded_bytes - SSFP_SEQ_AND_CRC_SIZE);
                        }

                        state = SYNCING;
                    } else if (rx == SSFP_ESCAPE_BYTE) {
                        state = ESCAPING;
                    } else {
                        // Push into buffer
                        rx_buffer[loaded_bytes++] = rx;
                        state = LOADING;
                    }
                    break;

                default:
                    state = LOST;
            }
        }
    }

protected:
    State state = LOST;
    size_t loaded_bytes = 0;
    uint8_t last_sequence = 255;  // So first expected sequence is 0
    OnReadInterface *callback = nullptr;
    DeframerError error = {};
};

#endif  // SSFP_H
