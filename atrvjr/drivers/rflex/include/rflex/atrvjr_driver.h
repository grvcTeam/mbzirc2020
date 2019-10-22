#ifndef ATRVJR_DRIVER_H
#define ATRVJR_DRIVER_H

#include <rflex/rflex_driver.h>
#include <sensor_msgs/PointCloud.h>

/**
 * \brief ATRVJR Driver class
 *  Modified from
 *  B21 Driver - By David Lu!! 2/2010
 *  Modified from Player code
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
class ATRVJR : public RFLEX {
    public:
        ATRVJR();
        virtual ~ATRVJR();
        void setSonarPower(bool);
        int getNumBodySonars() const;
        int getNumBaseSonars() const;

       /** Get readings from the sonar on the base of the ATRVJR
         * in meters
         * \param readings Data structure into which the sonar readings are saved */
        void getBaseSonarReadings(double* readings) const;

        /** Gets a point cloud for sonar readings from base
         * \param cloud Data structure into which the sonar readings are saved */
        void getBaseSonarPoints(sensor_msgs::PointCloud* cloud) const;

        /** Gets a point cloud for the bump sensors on the base
         * \param cloud Data structure into which the bump readings are saved
         * \return number of active bump sensors */
//        int getBaseBumps(sensor_msgs::PointCloud* cloud) const;

        /**
         * Overriding rflex setVelocity to be able to watch when no
         * velocity command arrives within interval.
         */
        void setVelocity( const double tvel, const double rvel);

        /** Processes the DIO packets - called from RFflex Driver
         * \param address origin
         * \param data values */
        void processDioEvent(unsigned char address, unsigned short data);

        /** Detects whether the robot has all the necessary components
         * to calculate odometry
         * \return bool true if robot has read its distance, bearing and home bearing */
        bool isOdomReady() const {
            return odomReady==3;
        }

    private:
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param readings Data structure into which the sonar readings are saved */
        void getSonarReadings(const int ringi, double* readings) const;
        /**\param ringi BODY_INDEX or BASE_INDEX
         * \param cloud Data structure into which the sonar readings are saved */
        void getSonarPoints(const int ringi, sensor_msgs::PointCloud* cloud) const;

        /**\param index BODY_INDEX or BASE_INDEX
           \param cloud Data structure into which the bump sensors are saved
           \return number of active bump sensors
        */

//        int home_bearing; ///< Last home bearing (arbitrary units)

        ros::Time last_velocity_time;
        void watchdogThread();///< Thread that stops robot if no vel command received

        // Not allowed to use these
        ATRVJR(const ATRVJR &atrvjr); 				///< Private constructor - Don't use
        ATRVJR &operator=(const ATRVJR &atrvjr); 	///< Private constructor - Don't use
};

#endif

