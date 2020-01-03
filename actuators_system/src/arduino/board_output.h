#ifndef BOARD_OUTPUT_H
#define BOARD_OUTPUT_H

#include "ssfp.h"
#include "board_input.h"

// TODO: Auto generate MAX_MSG_SIZE and MIN_BUFFER_SIZE
#define BOARD_OUTPUT_MAX_MSG_SIZE (1 + BOARD_INPUT_MAX_MSG_SIZE + 1)
#define BOARD_OUTPUT_MIN_BUFFER_SIZE (2 + 2 + 2*BOARD_OUTPUT_MAX_MSG_SIZE + 4)

// TODO: Auto generate? e.g: from ROS msg file
// TODO: Base class to force: size, to_buffer, from_buffer, print...
struct BoardOutput {

    bool digital_in_0 : 1;
    bool digital_in_1 : 1;
    bool digital_in_2 : 1;
    bool digital_in_3 : 1;
    BoardInput input_echo;
    DeframerError rx_error;

    size_t size() {
        // TODO: Variable size? -> min_size!
        return BOARD_OUTPUT_MAX_MSG_SIZE;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data = 0;
        packed_data |= digital_in_0? (uint8_t)(1 << 0): 0;
        packed_data |= digital_in_1? (uint8_t)(1 << 1): 0;
        packed_data |= digital_in_2? (uint8_t)(1 << 2): 0;
        packed_data |= digital_in_3? (uint8_t)(1 << 3): 0;
        index += t8_to_buffer(packed_data, &buffer[index]);
        index += input_echo.to_buffer(&buffer[index]);
        index += rx_error.to_buffer(&buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data;
        index += t8_from_buffer(&packed_data, &buffer[index]);
        digital_in_0 = packed_data & (uint8_t)(1 << 0);
        digital_in_1 = packed_data & (uint8_t)(1 << 1);
        digital_in_2 = packed_data & (uint8_t)(1 << 2);
        digital_in_3 = packed_data & (uint8_t)(1 << 3);
        index += input_echo.from_buffer(&buffer[index]);
        index += rx_error.from_buffer(&buffer[index]);
        return index;
    }

    void print() {
        printf("digital_in_0: %s\n", digital_in_0? "true": "false");
        printf("digital_in_1: %s\n", digital_in_1? "true": "false");
        printf("digital_in_2: %s\n", digital_in_2? "true": "false");
        printf("digital_in_3: %s\n", digital_in_3? "true": "false");
        input_echo.print();
        rx_error.print();
    }
};

#endif  // BOARD_OUTPUT_H
