//#include <Arduino.h>
#include "ssfp.h"
#include "actuators_input.h"
#include "actuators_output.h"
//#include "Timer.h"

//#define PERIOD_IN_MS 1000
#define SERIAL_BAUDRATE 9600

uint8_t rx_buffer[ACTUATORS_INPUT_MIN_BUFFER_SIZE];
uint8_t tx_buffer[ACTUATORS_OUTPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[ACTUATORS_INPUT_MAX_MSG_SIZE];
uint8_t aux_tx_buffer[ACTUATORS_OUTPUT_MAX_MSG_SIZE];

class ActuatorsInputReader: public OnReadInterface {
public:
    ActuatorsInput data;  // TODO: only through get, has_new_data = false
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
//Timer timer = Timer(PERIOD_IN_MS);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(10);
  deframer.connect(&actuators_input_reader);
  //timer.begin();
}

void loop() {
  size_t bytes_read = Serial.readBytes(rx_buffer, ACTUATORS_INPUT_MIN_BUFFER_SIZE);
  deframer.deframe(rx_buffer, bytes_read, aux_rx_buffer);
  rx_error = deframer.get_error();
  digitalWrite(LED_BUILTIN, rx_error.any());  // Error LED

  if (actuators_input_reader.has_new_data) {
    actuators_input_reader.has_new_data = false;

    actuators_output.input_echo = actuators_input_reader.data;
    actuators_output.rx_error = rx_error;

    size_t output_size = actuators_output.to_buffer(aux_tx_buffer);
    size_t frame_size = framer.frame(aux_tx_buffer, output_size, tx_buffer);
    Serial.write(tx_buffer, frame_size);
  }

//  timer.sleep();
}
