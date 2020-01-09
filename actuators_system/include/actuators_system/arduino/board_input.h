//----------------------------------------------------------------------------------------------------------------------
// GRVC ACTUATORS SYSTEM
//----------------------------------------------------------------------------------------------------------------------
// The MIT License (MIT)
// 
// Copyright (c) 2020 GRVC University of Seville
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
// documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
// rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
// Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS
// OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
// OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//----------------------------------------------------------------------------------------------------------------------
#ifndef BOARD_INPUT_H
#define BOARD_INPUT_H

#include "ssfp.h"

// TODO: Auto generate MAX_MSG_SIZE and MIN_BUFFER_SIZE
// MAX_MSG_SIZE: 5*sizeof(uint16_t) + sizeof(uint8_t) = 11
// MIN_BUFFER_SIZE: 2 for framing, up to 2 for sequence, up to 4 for crc and up to 2*MAX_MSG_SIZE for message
#define BOARD_INPUT_MAX_MSG_SIZE 11
#define BOARD_INPUT_MIN_BUFFER_SIZE (2 + 2 + 2*BOARD_INPUT_MAX_MSG_SIZE + 4)

#define PWM_INI 1500
uint16_t def_pwm_ini[5] = {PWM_INI, PWM_INI, PWM_INI, PWM_INI, PWM_INI};

// TODO: Auto generate? e.g: from ROS msg file
// TODO: Base class to force: size, to_buffer, from_buffer, print...
struct BoardInput {

    uint16_t pwm[5];
    bool digital_out_0 : 1;

    BoardInput(uint16_t pwm_ini[5] = def_pwm_ini, bool digital_out_0_ini = false) {
        for (int i = 0; i < 5; i++) {
            pwm[i] = pwm_ini[i];
        }
        digital_out_0 = digital_out_0_ini;
    }

    size_t size() {
        // TODO: Variable size? -> min_size!
        return BOARD_INPUT_MAX_MSG_SIZE;
    }

    size_t to_buffer(uint8_t *buffer) {
        size_t index = 0;
        for (int i = 0; i < 5; i++) {
            index += t16_to_buffer(pwm[i], &buffer[index]);
        }
        uint8_t packed_data = 0;
        packed_data |= digital_out_0? (uint8_t)(1 << 0): 0;
        index += t8_to_buffer(packed_data, &buffer[index]);
        return index;
    }

    size_t from_buffer(uint8_t *buffer) {
        size_t index = 0;
        for (int i = 0; i < 5; i++) {
            index += t16_from_buffer(&pwm[i], &buffer[index]);
        }
        uint8_t packed_data;
        index += t8_from_buffer(&packed_data, &buffer[index]);
        digital_out_0 = packed_data & (uint8_t)(1 << 0);
        return index;
    }

    void print() {
        for (int i = 0; i < 5; i++) {
            printf("pwm[%d]: %d\n", i, pwm[i]);
        }
        printf("digital_out_0: %s\n", digital_out_0? "true": "false");
    }
};

#endif  // BOARD_INPUT_H
