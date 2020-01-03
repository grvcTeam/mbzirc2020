#include <serial_port.h>
#include <arduino/ssfp.h>
#include <arduino/board_input.h>
#include <arduino/board_output.h>

uint8_t rx_buffer[2*BOARD_OUTPUT_MIN_BUFFER_SIZE];
uint8_t tx_buffer[BOARD_INPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[BOARD_OUTPUT_MAX_MSG_SIZE];
uint8_t aux_tx_buffer[BOARD_INPUT_MAX_MSG_SIZE];

class BoardOutputReader: public OnReadInterface {
public:
    BoardOutput data;  // TODO: only through get, has_new_data = false
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
    BoardInput board_input;
    board_input.pwm[0] = 900;
    board_input.pwm[1] = 901;
    board_input.pwm[2] = 902;
    board_input.pwm[3] = 903;
    board_input.digital_out_0 = false;

    Deframer deframer;
    BoardOutputReader board_output_reader;
    deframer.connect(&board_output_reader);

    int i = 0;
    while (true) {
        size_t msg_size = board_input.to_buffer(aux_tx_buffer);
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

        if (board_output_reader.has_new_data) {
            printf("\n________[%d]________\n\n", i);
            board_output_reader.data.print();
            board_output_reader.has_new_data = false;
        }

        usleep(1e6);
        i++;
    }

    close(serial_port);
    return 0;
}
