#ifndef ACTUATORS_INPUT_H
#define ACTUATORS_INPUT_H

#include "ssfp.h"

// TODO: Auto generate MAX_MSG_SIZE and MIN_BUFFER_SIZE
// MAX_MSG_SIZE: 3*sizeof(uint16_t) + sizeof(uint16_t) + sizeof(uint8_t)
// MIN_BUFFER_SIZE: 2 for framing, up to 2 for sequence, up to 4 for crc and up to 2*MAX_MSG_SIZE for message
#define ACTUATORS_INPUT_MAX_MSG_SIZE 9
#define ACTUATORS_INPUT_MIN_BUFFER_SIZE (2 + 2 + 2*ACTUATORS_INPUT_MAX_MSG_SIZE + 4)

// TODO: Auto generate? e.g: from ROS msg file
// TODO: Base class to force: size, to_buffer, from_buffer, print...
struct ActuatorsInput {

    uint16_t gripper_pwm[3];
    uint16_t extinguisher_pwm;
    bool pump_activation : 1;

    size_t size() {
        // TODO: Variable size?
        return ACTUATORS_INPUT_MAX_MSG_SIZE;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        for (int i = 0; i < 3; i++) {
            index += t16_to_buffer(gripper_pwm[i], &buffer[index]);
        }
        index += t16_to_buffer(extinguisher_pwm, &buffer[index]);
        uint8_t packed_data = 0;
        packed_data |= pump_activation? (uint8_t)(1 << 0): 0;
        index += t8_to_buffer(packed_data, &buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        for (int i = 0; i < 3; i++) {
            index += t16_from_buffer(&gripper_pwm[i], &buffer[index]);
        }
        index += t16_from_buffer(&extinguisher_pwm, &buffer[index]);
        uint8_t packed_data;
        index += t8_from_buffer(&packed_data, &buffer[index]);
        pump_activation = packed_data & (uint8_t)(1 << 0);
        return index;
    }

    void print() {
        for (int i = 0; i < 3; i++) {
            printf("gripper_pwm[%d]: %d\n", i, gripper_pwm[i]);
        }
        printf("extinguisher_pwm: %d\n", extinguisher_pwm);
        printf("pump_activation: %s\n", pump_activation? "true": "false");
    }
};

#endif  //ACTUATORS_INPUT_H
