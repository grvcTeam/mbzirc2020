#ifndef ACTUATORS_OUTPUT_H
#define ACTUATORS_OUTPUT_H

#include "ssfp.h"
#include "actuators_input.h"

// TODO: Auto generate MAX_MSG_SIZE and MIN_BUFFER_SIZE
#define ACTUATORS_OUTPUT_MAX_MSG_SIZE (1 + ACTUATORS_INPUT_MAX_MSG_SIZE + 1)
#define ACTUATORS_OUTPUT_MIN_BUFFER_SIZE (2 + 2 + 2*ACTUATORS_OUTPUT_MAX_MSG_SIZE + 4)

// TODO: Auto generate? e.g: from ROS msg file
// TODO: Base class to force: size, to_buffer, from_buffer, print...
struct ActuatorsOutput {

    bool low_level  : 1;
    bool attached_0 : 1;
    bool attached_1 : 1;
    bool attached_2 : 1;
    ActuatorsInput input_echo;
    DeframerError rx_error;

    size_t size() {
        // TODO: Variable size?
        return ACTUATORS_OUTPUT_MAX_MSG_SIZE;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data = 0;
        packed_data |= low_level?  (uint8_t)(1 << 0): 0;
        packed_data |= attached_0? (uint8_t)(1 << 1): 0;
        packed_data |= attached_1? (uint8_t)(1 << 2): 0;
        packed_data |= attached_2? (uint8_t)(1 << 3): 0;
        index += t8_to_buffer(packed_data, &buffer[index]);
        index += input_echo.to_buffer(&buffer[index]);
        index += rx_error.to_buffer(&buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        uint8_t packed_data;
        index += t8_from_buffer(&packed_data, &buffer[index]);
        low_level  = packed_data & (uint8_t)(1 << 0);
        attached_0 = packed_data & (uint8_t)(1 << 1);
        attached_1 = packed_data & (uint8_t)(1 << 2);
        attached_2 = packed_data & (uint8_t)(1 << 3);
        index += input_echo.from_buffer(&buffer[index]);
        index += rx_error.from_buffer(&buffer[index]);
        return index;
    }

    void print() {
        printf("low_level:  %s\n", low_level?  "true": "false");
        printf("attached_0: %s\n", attached_0? "true": "false");
        printf("attached_1: %s\n", attached_1? "true": "false");
        printf("attached_2: %s\n", attached_2? "true": "false");
        input_echo.print();
        rx_error.print();
    }
};

#endif  //ACTUATORS_OUTPUT_H
