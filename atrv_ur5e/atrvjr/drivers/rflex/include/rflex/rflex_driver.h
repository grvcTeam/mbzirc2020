#ifndef RFLEX_DRIVER_H
#define RFLEX_DRIVER_H

#include <rflex/rflex_info.h>
#include <pthread.h>
#include <boost/function.hpp>

/**
 * \brief RFLEX Driver to handle input and output to RFlex devices.
 *
 *  RFLEX Driver - 2/2010
 *  Modified from Player code by David Lu!!
 *  Original Input Output code by Bill Smart
 *
 *  Player - One Hell of a Robot Server
 *  Copyright (C) 2000
 *     Brian Gerkey, Kasper Stoy, Richard Vaughan, & Andrew Howard
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

class SimpleSignal{
    public:
        SimpleSignal():callback(NULL){}

        void set(boost::function<void()> callback){
            this->callback = callback;
        }

        void invoke(){
            if (callback !=NULL){
                callback();
            }
        }

    private:
        boost::function<void()> callback;
};

/**
 * Holds low-level configuration parameters for RFLEX.
 * These used to be constant definitions in rflex_info.h
 */
class RFLEXConfig {
private:
    unsigned long odo_distance_conversion;
    unsigned long odo_angle_conversion;

    double trans_acc;///< In m/s/s
    double rot_acc;///< In m/s/s
    double trans_torque; ///< In N*m
    double rot_torque; ///< In N*m

public:
    double power_offset;///< In V
    double plugged_threshold;///< In V
    int home_bearing;

public:
    RFLEXConfig() :
            odo_distance_conversion(93810), //
            odo_angle_conversion(38500), //
            trans_acc(0.7), //
            rot_acc(2.6), //
            trans_torque(0.3), //
            rot_torque(0.9), //
            power_offset(1.2),//
            plugged_threshold(29),//
            home_bearing(-32500)
    {
    }

    /**
     * Convert the real translational value into RFLEX units.
     * @param value
     */
    unsigned long realTrans2driver(double value) const {
        return value * odo_distance_conversion;
    }
    double driverTrans2real(int value) const{
        return value / (double) odo_distance_conversion;
    }

    /**
     * Convert the real angular value into RFLEX units.
     * @param value
     */
    unsigned long realAngle2driver(double value) const {
        return value * odo_angle_conversion;
    }
    double driverAngle2real(int value) const{
        return value / (double) odo_angle_conversion;
    }

    unsigned long getAdjustedTransAcc(){
        return realTrans2driver(trans_acc);
    }
    unsigned long getAdjustedRotAcc(){
        return realAngle2driver(rot_acc);
    }
    unsigned long getAdjustedTransTorque(){
        return realTrans2driver(trans_torque);
    }
    unsigned long getAdjustedRotTorque(){
        return realAngle2driver(rot_torque);
    }

    void setOdoAngleConversion(unsigned long odoAngleConversion) {
        odo_angle_conversion = odoAngleConversion;
    }

    void setOdoDistanceConversion(unsigned long odoDistanceConversion) {
        odo_distance_conversion = odoDistanceConversion;
    }

    void setRotAcc(double rotAcc) {
        rot_acc = rotAcc;
    }

    void setRotTorque(double rotTorque) {
        rot_torque = rotTorque;
    }

    void setTransAcc(double transAcc) {
        trans_acc = transAcc;
    }

    void setTransTorque(double transTorque) {
        trans_torque = transTorque;
    }

    void setPluggedThreshold(double pluggedThreshold) {
        plugged_threshold = pluggedThreshold;
    }

    void setPowerOffset(double powerOffset) {
        power_offset = powerOffset;
    }
};

class RFLEX {
    public:
        RFLEX();
        virtual ~RFLEX();

        /** Opens connection to serial port with specified device name
            \param devname Device name assigned to serial port
            \return -1 on error */
        int initialize(const char* devname);

        /** Configure the sonar parameters and send message to RFlex.
          * \param echoDelay Echo Delay
          * \param pingDelay Ping Delay
          * \param setDelay Set Delay
          * \param val Unknown
          * @todo Figure out unknown value's purpose.
          */
        void configureSonar(const unsigned long echoDelay, const unsigned long pingDelay,
                            const unsigned long setDelay, const unsigned long val);

        /** Turn IR on or off
          * \param power true for on, false for off */
        void setIrPower(const bool power);
        /** Turn Brake on or off
          * Note: Brake on means the controller cannot move the robot
          *    and external forces CAN move it.
          * \param power true for on, false for off */
        void setBrakePower(const bool power);

        /** Set the frequency that the Digital IO devices are checked.
          * \param period Period in milliseconds
          */
        void setDigitalIoPeriod(const long period);

        /** Set the frequency that the odometry is checked.
          * \param period Period in milliseconds
          */
        void setOdometryPeriod(const long period);

        /** Sends a set motion defaults message to the device. */
        void motionSetDefaults();

        /** Gets brake power
           * \return True if brake is engaged */
        bool getBrakePower() const {
            return brake;
        }

        /** Gets the number of IR sensors
         * \return Number of IR sensors
         */
        int  getIrCount() const {
            return numIr;
        }

        /** Sets the velocity
         * \param transVelocity Translational velocity in m/s
         * \param rotVelocity Rotational velocity in rad/s
         */
        void setVelocity(const double transVelocity, const double rotVelocity);

        /** Sends a system status command to the device.
         * Updates the brake and battery status. */
        void sendSystemStatusCommand();

        double getDistance();///< Translational odometry, m.
        double getBearing() const;///< Rotational odometry, rad.
        double getTransVelocity() const;///< Translational velocity, m/s.
        double getRotVelocity() const;///< Rotational velocity, rad/s.
        double getVoltage() const;
        bool isPluggedIn() const;


        /// Signals
        SimpleSignal systemStatusUpdateSignal;
        SimpleSignal motorUpdateSignal;
        SimpleSignal sonarUpdateSignal;

        RFLEXConfig config;
     //todo fix permissions 

    //protected:

        virtual void processDioEvent(unsigned char address, unsigned short data);

        int sonar_ranges[SONAR_MAX_COUNT];	///< Raw Sonar readings (including unconnected ports)
        long voltage;	///< Raw voltage reading
        bool brake;		///< Brake Status

        unsigned short dioData[24];	///< Storage for digital IO values

        int lcdX, lcdY;
        unsigned char * lcdData;

        int numIr; ///< Number of IR sensors
        unsigned char * irRanges; ///< Raw values from IR sensors
        int home_bearing_found;
        int odomReady;
     //todo fix permissions 
    //private:

        int distance;			///< Raw translational odometry
        int bearing;			///< Raw rotational odometry
        int transVelocity;		///< Raw translational velocity
        int rotVelocity;		///< Raw rotational velocity

        int first_distance;     ///< Raw odometry reading at start time
        bool found_distance;    ///< True when the first_distance was initialized

        void parsePacket(const unsigned char* buffer);
        void parseMotReport(const unsigned char* buffer);
        void parseDioReport(const unsigned char* buffer);
        void parseIrReport(const unsigned char* buffer);
        void parseSysReport(const unsigned char* buffer);
        void parseSonarReport(const unsigned char* buffer);
        void parseJoyReport(const unsigned char* buffer);


        // IO Stuff
        int fd;				///< File descriptor for serial port
        pthread_t thread;	///< Thread which reads input upon arrival
        pthread_mutex_t writeMutex; ///< Mutex around writing to port

        unsigned char readBuffer[BUFFER_SIZE];
        unsigned char writeBuffer[BUFFER_SIZE];

        bool found;
        int offset;

        static void *readThread(void *ptr); ///< Read Thread

        /**
          * Send a command to the serial port
         * \param port Should be one of these: SYS_PORT, MOT_PORT, JSTK_PORT, SONAR_PORT, DIO_PORT, IR_PORT
         * \param id
         * \param opcode See opcodes in rflex_info.h
         * \param length length of the data
         * \param data actual data */
        bool sendCommand(const unsigned char port, const unsigned char id, const unsigned char opcode, const int length, unsigned char* data);

        void readPacket(); ///< After reading the data, it checks for errors and then parses
        int readData();    ///< Reads in a packet until it finds and end of packet signal
        bool writePacket(const int length) const; ///< Writes packet currently in write buffer to device
        unsigned char computeCRC(const unsigned char *buffer, const int n); ///< Calculates error checking code for specified buffer

        // Not allowed to use these
        RFLEX(const RFLEX &rflex); 				///< Private constructor - Don't use
        RFLEX &operator=(const RFLEX &rflex);	///< Private constructor - Don't use
};
#endif
