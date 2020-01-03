#ifndef ACTUATORS_OUTPUT_H
#define ACTUATORS_OUTPUT_H

#include "ssfp.h"
#include "actuators_input.h"

// TODO: Auto generate MAX_MSG_SIZE and MIN_BUFFER_SIZE
#define ACTUATORS_OUTPUT_MAX_MSG_SIZE (ACTUATORS_INPUT_MAX_MSG_SIZE + 1)
#define ACTUATORS_OUTPUT_MIN_BUFFER_SIZE (2 + 2 + 2*ACTUATORS_OUTPUT_MAX_MSG_SIZE + 4)

// TODO: Auto generate? e.g: from ROS msg file
// TODO: Base class to force: size, to_buffer, from_buffer, print...
struct ActuatorsOutput {

    ActuatorsInput input_echo;
    DeframerError rx_error;

    size_t size() {
        // TODO: Variable size?
        return ACTUATORS_OUTPUT_MAX_MSG_SIZE;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        index += input_echo.to_buffer(&buffer[index]);
        index += rx_error.to_buffer(&buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        index += input_echo.from_buffer(&buffer[index]);
        index += rx_error.from_buffer(&buffer[index]);
        return index;
    }

    void print() {
        input_echo.print();
        rx_error.print();
    }
};

#endif  //ACTUATORS_OUTPUT_H
