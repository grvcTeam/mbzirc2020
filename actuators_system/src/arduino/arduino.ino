#include "ssfp.h"
#include "actuators_input.h"
#include "actuators_output.h"
#include "Timer.h"
#include <Servo.h>

#define SERIAL_BAUDRATE 9600

#define PERIOD_IN_MS 20
#define MIN_SERVO_PWM 900
#define INI_SERVO_PWM 1500
#define MAX_SERVO_PWM 2100

#define PIN_SERVO_GRIPPER_0     3
#define PIN_SERVO_GRIPPER_1     5
#define PIN_SERVO_GRIPPER_2     6
#define PIN_SERVO_EXTINGUISHER  9
#define PIN_PUMP_RELAY          2

#define PIN_LEVEL               4
#define PIN_GRIPPER_ATTACHED_0  7
#define PIN_GRIPPER_ATTACHED_1  8
#define PIN_GRIPPER_ATTACHED_2 12

uint8_t rx_buffer[ACTUATORS_INPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[ACTUATORS_INPUT_MAX_MSG_SIZE];

uint8_t tx_buffer[ACTUATORS_OUTPUT_MIN_BUFFER_SIZE];
uint8_t aux_tx_buffer[ACTUATORS_OUTPUT_MAX_MSG_SIZE];

class ActuatorsInputReader: public OnReadInterface {
public:
    ActuatorsInput data;
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
ActuatorsInputReader actuators_input_reader;

Framer framer;    
ActuatorsOutput actuators_output;

Timer timer = Timer(PERIOD_IN_MS);

Servo gripper[3];
Servo extinguisher;

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(5);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(PIN_PUMP_RELAY, OUTPUT);

  pinMode(PIN_LEVEL, INPUT);
  pinMode(PIN_GRIPPER_ATTACHED_0, INPUT);
  pinMode(PIN_GRIPPER_ATTACHED_1, INPUT);
  pinMode(PIN_GRIPPER_ATTACHED_2, INPUT);

  deframer.connect(&actuators_input_reader);
  actuators_input_reader.data.gripper_pwm[0]   = INI_SERVO_PWM;
  actuators_input_reader.data.gripper_pwm[1]   = INI_SERVO_PWM;
  actuators_input_reader.data.gripper_pwm[2]   = INI_SERVO_PWM;
  actuators_input_reader.data.extinguisher_pwm = INI_SERVO_PWM;
  actuators_input_reader.data.pump_activation  = false;

  actuators_output.low_level = false;
  actuators_output.attached_0 = false;
  actuators_output.attached_1 = false;
  actuators_output.attached_2 = false;
  actuators_output.input_echo = actuators_input_reader.data;
  actuators_output.rx_error = rx_error;
  
  gripper[0].attach(PIN_SERVO_GRIPPER_0, MIN_SERVO_PWM, MAX_SERVO_PWM);
  gripper[1].attach(PIN_SERVO_GRIPPER_1, MIN_SERVO_PWM, MAX_SERVO_PWM);
  gripper[2].attach(PIN_SERVO_GRIPPER_2, MIN_SERVO_PWM, MAX_SERVO_PWM);
  extinguisher.attach(PIN_SERVO_EXTINGUISHER, MIN_SERVO_PWM, MAX_SERVO_PWM);

  timer.begin();
}

void loop() {
  size_t bytes_read = Serial.readBytes(rx_buffer, ACTUATORS_INPUT_MIN_BUFFER_SIZE);
  deframer.deframe(rx_buffer, bytes_read, aux_rx_buffer);
  rx_error = deframer.get_error();
  digitalWrite(LED_BUILTIN, rx_error.any());  // Error LED

  if (actuators_input_reader.has_new_data) {
    actuators_input_reader.has_new_data = false;

    actuators_output.low_level = digitalRead(PIN_LEVEL);
    actuators_output.attached_0 = digitalRead(PIN_SERVO_GRIPPER_0);
    actuators_output.attached_1 = digitalRead(PIN_SERVO_GRIPPER_1);
    actuators_output.attached_2 = digitalRead(PIN_SERVO_GRIPPER_2);
    actuators_output.input_echo = actuators_input_reader.data;
    actuators_output.rx_error = rx_error;

    size_t output_size = actuators_output.to_buffer(aux_tx_buffer);
    size_t frame_size = framer.frame(aux_tx_buffer, output_size, tx_buffer);
    Serial.write(tx_buffer, frame_size);
  }

  for (int i = 0; i < 3; i++) {
    gripper[i].writeMicroseconds(actuators_input_reader.data.gripper_pwm[i]);
  }
  extinguisher.writeMicroseconds(actuators_input_reader.data.extinguisher_pwm);
  digitalWrite(PIN_PUMP_RELAY, actuators_input_reader.data.pump_activation);

  timer.sleep();
}
