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
#include <mutex>

#include <ros/ros.h>
#include <mbzirc_comm_objs/SetPWM.h>
#include <mbzirc_comm_objs/SetDigital.h>
#include <mbzirc_comm_objs/ActuatorsData.h>

#include <handy_tools/serial_port.h>
#include <actuators_system/arduino/ssfp.h>
#include <actuators_system/arduino/board_input.h>
#include <actuators_system/arduino/board_output.h>

#define PWM_COUNT 5

uint8_t rx_buffer[2*BOARD_OUTPUT_MIN_BUFFER_SIZE];
uint8_t tx_buffer[BOARD_INPUT_MIN_BUFFER_SIZE];
uint8_t aux_rx_buffer[BOARD_OUTPUT_MAX_MSG_SIZE];
uint8_t aux_tx_buffer[BOARD_INPUT_MAX_MSG_SIZE];

BoardInput board_input;
std::mutex input_mutex;

bool set_pwm(mbzirc_comm_objs::SetPWM::Request  &req, mbzirc_comm_objs::SetPWM::Response &res) {
    if (req.id >= PWM_COUNT) {
        ROS_ERROR("%s: PWM id [%d] out of bounds", __FUNCTION__, req.id);
        return false;
    }
    // TODO: Check PWM is in range?

    input_mutex.lock();
    board_input.pwm[req.id] = req.pwm;
    input_mutex.unlock();
    return true;
}

bool set_digital(mbzirc_comm_objs::SetDigital::Request  &req, mbzirc_comm_objs::SetDigital::Response &res) {
    if (req.id >= 1) {
        ROS_ERROR("%s: Digital id [%d] out of bounds", __FUNCTION__, req.id);
        return false;
    }

    input_mutex.lock();
    board_input.digital_out_0 = req.value;
    input_mutex.unlock();
    return true;
}

mbzirc_comm_objs::ActuatorsData from_board_output(const BoardOutput& board_output) {
    mbzirc_comm_objs::ActuatorsData actuators_data;
    actuators_data.digital_reading[0] = board_output.digital_in_0;
    actuators_data.digital_reading[1] = board_output.digital_in_1;
    actuators_data.digital_reading[2] = board_output.digital_in_2;
    actuators_data.digital_reading[3] = board_output.digital_in_3;
    for (int i = 0; i < PWM_COUNT; i++) {
        actuators_data.pwm_setpoint[i] = board_output.input_echo.pwm[i];
    }
    actuators_data.digital_setpoint = board_output.input_echo.digital_out_0;

    return actuators_data;
}

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "actuators_system_node");
    ros::NodeHandle n;

    std::string serial_path;
    int serial_baudrate;
    ros::param::param<std::string>("~serial_path", serial_path, "/dev/ttyUSB0");
    ros::param::param<int>("~serial_baudrate", serial_baudrate, 9600);

    // Open the serial port
    int serial_port = open(serial_path.c_str(), O_RDWR);
    if (serial_port == -1) {
        char e_buffer[256];
        snprintf(e_buffer, sizeof(e_buffer), "Error %i calling open: %s", errno, strerror(errno));
        throw std::runtime_error(e_buffer);
    }

    serial_port::configure(serial_port, serial_baudrate);
    serial_port::lock(serial_port);

    Framer framer;
    ros::ServiceServer set_pwm_service = n.advertiseService("set_pwm", set_pwm);
    ros::ServiceServer set_digital_service = n.advertiseService("set_digital", set_digital);

    Deframer deframer;
    BoardOutputReader board_output_reader;
    deframer.connect(&board_output_reader);
    ros::Publisher actuators_data_pub = n.advertise<mbzirc_comm_objs::ActuatorsData>("actuators_data", 10);

    ros::Rate rate(10);  // [Hz]
    while (ros::ok()) {

        size_t rx_bytes = read(serial_port, &rx_buffer, sizeof(rx_buffer));
        if (rx_bytes < 0) {
            printf("Error reading: %s", strerror(errno));
        } else if (rx_bytes > 0) {
            deframer.deframe(rx_buffer, rx_bytes, aux_rx_buffer);
            DeframerError error = deframer.get_error();
            if (error.crc)      { ROS_WARN("Deframer error at computer side: crc"); }
            if (error.lost)     { ROS_WARN("Deframer error at computer side: lost"); }
            if (error.sequence) { ROS_WARN("Deframer error at computer side: sequence"); }
            if (error.nullback) { ROS_WARN("Deframer error at computer side: nullback"); }
            if (error.callback) { ROS_WARN("Deframer error at computer side: callback"); }
            // if (error.any()) { return 1; }
        }

        if (board_output_reader.has_new_data) {
            board_output_reader.has_new_data = false;
            actuators_data_pub.publish(from_board_output(board_output_reader.data));
            // board_output_reader.data.print();

            if (board_output_reader.data.rx_error.crc)      { ROS_WARN("Deframer error at board side: crc"); }
            if (board_output_reader.data.rx_error.lost)     { ROS_WARN("Deframer error at board side: lost"); }
            if (board_output_reader.data.rx_error.sequence) { ROS_WARN("Deframer error at board side: sequence"); }
            if (board_output_reader.data.rx_error.nullback) { ROS_WARN("Deframer error at board side: nullback"); }
            if (board_output_reader.data.rx_error.callback) { ROS_WARN("Deframer error at board side: callback"); }
        }

        size_t msg_size = board_input.to_buffer(aux_tx_buffer);
        size_t frame_size = framer.frame(aux_tx_buffer, msg_size, tx_buffer);
        write(serial_port, tx_buffer, frame_size);

        ros::spinOnce();
        rate.sleep();
    }

    close(serial_port);
    return 0;
}
