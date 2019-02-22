# odom_calibration
Applies a previously obtained calibration matrix.

The calibration matrix is obtained using the method described [here](http://ais.informatik.uni-freiburg.de/teaching/ws10/robotics2/pdfs/rob2-05-odometry-calib-practical.pdf).

# Parameters
## odom_raw
Input odometry topic

## odom_calibrated
Output odometry topic

## odom_calib_matrix
Calibration matrix

## twist_linear_x_regression [m, b]
Linear velocity linear regression parameters: y = m x + b 

## twist_angular_yaw_regression [m, b]
Angular velocity linear regression parameters: y = m x + b 

# TODO
- Write code for obtaining calibration matrix
- Remove parameters odom_raw and odom_calibrated (remap can be used)

# Authors
- David Dias
