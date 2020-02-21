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
#include "ssfp.h"
#include "board_input.h"
#include "board_output.h"
#include "Timer.h"
#include <Servo.h>

#define SERIAL_BAUDRATE 9600

#define PERIOD_IN_MS 20
#define INI_SERVO_PWM 1800

#define PIN_PWM_0   5
#define PIN_PWM_1   6
#define PIN_PWM_2   9
#define PIN_PWM_3  10
#define PIN_PWM_4  11

#define PIN_OUT_0  12

#define PIN_IN_0   A3  // This one needs pullup
#define PIN_IN_1   A2
#define PIN_IN_2   A1
#define PIN_IN_3   A0

uint8_t rx_buffer[2*BOARD_INPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[BOARD_INPUT_MAX_MSG_SIZE];

uint8_t tx_buffer[BOARD_OUTPUT_MIN_BUFFER_SIZE];
uint8_t aux_tx_buffer[BOARD_OUTPUT_MAX_MSG_SIZE];

class BoardInputReader: public OnReadInterface {
public:
    BoardInput data;
    bool has_new_data = false;

    bool call(uint8_t *buffer, size_t size) {
        if (size < data.size()) {
            return false;
        }
        data.from_buffer(buffer);
        has_new_data = true;
        return true;
    };
};

Deframer deframer;
DeframerError rx_error;
BoardInputReader board_input_reader;

Framer framer;    
BoardOutput board_output;

Timer timer = Timer(PERIOD_IN_MS);

Servo servo[5];
uint16_t pwm_min[5] = { 500,  900,  900,  900,  900};
uint16_t pwm_max[5] = {2500, 2100, 2100, 2100, 2100};

void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(5);
  while (!Serial) { ; }  // Wait for serial port to connect

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_OUT_0, OUTPUT);

  pinMode(PIN_IN_0, INPUT_PULLUP);
  pinMode(PIN_IN_1, INPUT);
  pinMode(PIN_IN_2, INPUT);
  pinMode(PIN_IN_3, INPUT);

  deframer.connect(&board_input_reader);
  board_input_reader.data.pwm[0] = 560;
  board_input_reader.data.pwm[1] = INI_SERVO_PWM;
  board_input_reader.data.pwm[2] = INI_SERVO_PWM;
  board_input_reader.data.pwm[3] = INI_SERVO_PWM;
  board_input_reader.data.pwm[4] = INI_SERVO_PWM;
  board_input_reader.data.digital_out_0  = false;

  board_output.digital_in_0 = false;
  board_output.digital_in_1 = false;
  board_output.digital_in_2 = false;
  board_output.digital_in_3 = false;
  board_output.input_echo = board_input_reader.data;
  board_output.rx_error = rx_error;
  
  servo[0].attach(PIN_PWM_0, pwm_min[0], pwm_max[0]);
  servo[1].attach(PIN_PWM_1, pwm_min[1], pwm_max[1]);
  servo[2].attach(PIN_PWM_2, pwm_min[2], pwm_max[2]);
  servo[3].attach(PIN_PWM_3, pwm_min[3], pwm_max[3]);
  servo[4].attach(PIN_PWM_4, pwm_min[4], pwm_max[4]);

  timer.begin();
}

void loop() {

  if (Serial.available() > 0) {
    size_t bytes_read = Serial.readBytes(rx_buffer, BOARD_INPUT_MIN_BUFFER_SIZE);
    deframer.deframe(rx_buffer, bytes_read, aux_rx_buffer);
    rx_error = deframer.get_error();
    digitalWrite(LED_BUILTIN, rx_error.any());  // Error LED
  }

  if (board_input_reader.has_new_data) {
    board_input_reader.has_new_data = false;

    board_output.digital_in_0 = digitalRead(PIN_IN_0);
    board_output.digital_in_1 = digitalRead(PIN_IN_1);
    board_output.digital_in_2 = digitalRead(PIN_IN_2);
    board_output.digital_in_3 = digitalRead(PIN_IN_3);
    board_output.input_echo = board_input_reader.data;
    board_output.rx_error = rx_error;

    size_t output_size = board_output.to_buffer(aux_tx_buffer);
    size_t frame_size = framer.frame(aux_tx_buffer, output_size, tx_buffer);
    Serial.write(tx_buffer, frame_size);
  }

  for (int i = 0; i < 5; i++) {
    servo[i].writeMicroseconds(board_input_reader.data.pwm[i]);
  }
  digitalWrite(PIN_OUT_0, board_input_reader.data.digital_out_0);

  timer.sleep();
}
