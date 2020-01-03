#include <serial_port.h>
#include <arduino/ssfp.h>
#include <arduino/actuators_input.h>
#include <arduino/actuators_output.h>

uint8_t rx_buffer[ACTUATORS_OUTPUT_MIN_BUFFER_SIZE];
uint8_t tx_buffer[ACTUATORS_INPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[ACTUATORS_OUTPUT_MAX_MSG_SIZE];
uint8_t aux_tx_buffer[ACTUATORS_INPUT_MAX_MSG_SIZE];

class ActuatorsOutputReader: public OnReadInterface {
public:
    ActuatorsOutput data;  // TODO: only through get, has_new_data = false
    bool has_new_data = false;

    bool call(uint8_t *buffer, size_t size) {
        // TODO: possibly make a meta-parser here
        if (size < data.size()) {
            return false;
        }
        data.from_buffer(buffer);
        has_new_data = true;
        return true;
    };
};

int main() {
    // Open the serial port. Change device path as needed
    int serial_port = open("/dev/ttyACM0", O_RDWR);
    if (serial_port == -1) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Error %i calling open: %s", errno, strerror(errno));
        throw std::runtime_error(e_buffer);
    }

    serial_port::configure(serial_port);
    serial_port::lock(serial_port);

    Framer framer;
    ActuatorsInput actuators_input;
    actuators_input.gripper_pwm[0] = 900;
    actuators_input.gripper_pwm[1] = 901;
    actuators_input.gripper_pwm[2] = 902;
    actuators_input.extinguisher_pwm = 1000;
    actuators_input.pump_activation = false;

    Deframer deframer;
    // ActuatorsOutput actuators_output;
    ActuatorsOutputReader actuators_output_reader;
    deframer.connect(&actuators_output_reader);

    for (int i = 0; i < 10; i++) {
        size_t msg_size = actuators_input.to_buffer(aux_tx_buffer);
        size_t frame_size = framer.frame(aux_tx_buffer, msg_size, tx_buffer);
        write(serial_port, tx_buffer, frame_size);

        size_t rx_bytes = read(serial_port, &rx_buffer, sizeof(rx_buffer));
        if (rx_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
        } else {
            deframer.deframe(rx_buffer, rx_bytes, aux_rx_buffer);
            DeframerError error = deframer.get_error();
            if (error.crc) { printf("ERROR: crc\n"); }
            if (error.lost) { printf("ERROR: lost\n"); }
            if (error.sequence) { printf("ERROR: sequence\n"); }
            if (error.nullback) { printf("ERROR: nullback\n"); }
            if (error.callback) { printf("ERROR: callback\n"); }
            // if (error.any()) { return 1; }
        }

        if (actuators_output_reader.has_new_data) {
            printf("\n________[%d]________\n\n", i);
            actuators_output_reader.data.print();
            actuators_output_reader.has_new_data = false;
        }

        sleep(1);
    }

    close(serial_port);
    return 0;
}
