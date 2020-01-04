#include "ssfp.h"
#include "board_input.h"
#include "board_output.h"
#include "Timer.h"
#include <Servo.h>

#define SERIAL_BAUDRATE 9600

#define PERIOD_IN_MS 20
#define MIN_SERVO_PWM 900
#define INI_SERVO_PWM 1500
#define MAX_SERVO_PWM 2100

#define PIN_PWM_0   9
#define PIN_PWM_1   6
#define PIN_PWM_2   5
#define PIN_PWM_3  10
#define PIN_PWM_4  11

#define PIN_OUT_0  12

#define PIN_IN_0   A3
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

void setup() {

  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(5);
  while (!Serial) { ; }  // Wait for serial port to connect

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_OUT_0, OUTPUT);

  pinMode(PIN_IN_0, INPUT);
  pinMode(PIN_IN_1, INPUT);
  pinMode(PIN_IN_2, INPUT);
  pinMode(PIN_IN_3, INPUT);

  deframer.connect(&board_input_reader);
  board_input_reader.data.pwm[0] = INI_SERVO_PWM;
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
  
  servo[0].attach(PIN_PWM_0, MIN_SERVO_PWM, MAX_SERVO_PWM);
  servo[1].attach(PIN_PWM_1, MIN_SERVO_PWM, MAX_SERVO_PWM);
  servo[2].attach(PIN_PWM_2, MIN_SERVO_PWM, MAX_SERVO_PWM);
  servo[3].attach(PIN_PWM_3, MIN_SERVO_PWM, MAX_SERVO_PWM);
  servo[4].attach(PIN_PWM_4, MIN_SERVO_PWM, MAX_SERVO_PWM);

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
