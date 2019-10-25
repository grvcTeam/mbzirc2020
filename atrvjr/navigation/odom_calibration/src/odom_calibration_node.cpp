#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <math.h>
#include <tf/transform_broadcaster.h>

using namespace Eigen;

class Calibration {
    public:
        ros::NodeHandle n;
        ros::Publisher pub;
        ros::Subscriber sub;
        // Message to publish
        nav_msgs::Odometry odom_calib;
    private:
        Matrix3d calib_mat;             // Estimated calibration matrix
        Matrix3d prev_odom;             // Message to publish
        Matrix3d prev_odom_calib;             // Message to publish
    public:
        Calibration();
        static Matrix3d v2t(Vector3d v);
        static Vector3d t2v(Matrix3d A);
        Vector3d applyCalib(Vector3d u);
        Matrix3d getPoseInc(Matrix3d A, Matrix3d B);
        void calibrationCallback(const nav_msgs::Odometry::ConstPtr& msg);
        bool has_first_msg;
        Vector3d prev_odom_calib_vec;
        ros::Time previousTime;
        // Linear x velocity parameters
        std::vector<double> twist_linear_x_regression;
        // Angular yaw velocity parameters
        std::vector<double> twist_angular_yaw_regression;
};

Calibration::Calibration() : n ("~")
{
    // Odometry calibration matrix
    std::vector<double> odom_calib_matrix;
    if( n.getParam("odom_calib_matrix", odom_calib_matrix) ) {
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                calib_mat(i, j) = odom_calib_matrix[i*3 + j];
            }
        }
    }

    // Linear x velocity parameters
    n.getParam("twist_linear_x_regression", twist_linear_x_regression);

    // Angular yaw velocity parameters
    n.getParam("twist_angular_yaw_regression", twist_angular_yaw_regression);

    has_first_msg = false;
}

Matrix3d Calibration::v2t(Vector3d v)
{
    Eigen::Transform<float, 2, Eigen::Affine> r ( Rotation2D<float>(v(2)) );

    Eigen::Transform<float, 2, Eigen::Affine> t ( Translation<float,2>(v(0), v(1)) );

    Matrix3d A = Affine2d(t*r).matrix();
    return A;
}

Vector3d Calibration::t2v(Matrix3d A)
{
    Vector3d v = Vector3d(A(0,2), A(1,2), atan2(A(1,0),A(0,0)));

    return v;
}

Vector3d Calibration::applyCalib(Vector3d u)
{
    return calib_mat.transpose() * u;
}

Matrix3d Calibration::getPoseInc(Matrix3d A, Matrix3d B)
{
    return A.inverse()*B;
}

void Calibration::calibrationCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    // Copy of fields
    odom_calib.header.seq = msg->header.seq;
    odom_calib.header.stamp = msg->header.stamp;
    odom_calib.header.frame_id = msg->header.frame_id;
    odom_calib.child_frame_id = msg->child_frame_id;

    if(!has_first_msg){
        // Gets first msg so the calibrated odometry starts in the same place
        prev_odom_calib_vec(0) = msg->pose.pose.position.x;
        prev_odom_calib_vec(1) = msg->pose.pose.position.y;
        prev_odom_calib_vec(2) = tf::getYaw( msg->pose.pose.orientation );

        // Initializes recursive fields
        prev_odom = v2t(prev_odom_calib_vec);
        prev_odom_calib = v2t(prev_odom_calib_vec);

        for(int i = 0; i < 36; i += 7)
            odom_calib.pose.covariance[i] = 5;

        // Updates
        has_first_msg = true;
    }else{
        // Get msg vector
        double msg_yaw = tf::getYaw( msg->pose.pose.orientation );
        Vector3d msg_vec = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg_yaw);

        // Get msg transform
        Matrix3d msg_trans = v2t(msg_vec);

        // Get transform from previous odometry to current
        Matrix3d msg_trans_inc = getPoseInc(prev_odom, msg_trans);
        Vector3d msg_vec_r_frame = t2v(msg_trans_inc);
        //std::cout << "Mldivide vector:\n" << msg_vec_r_frame << std::endl;

        // Calibrate transform
        Vector3d msg_vec_r_frame_calibrated = applyCalib(msg_vec_r_frame);
        //std::cout << "Mldivide vector calibrated:\n" << msg_vec_r_frame_calibrated << std::endl;

        // Get full calibrated odometry
        prev_odom_calib = prev_odom_calib * v2t(msg_vec_r_frame_calibrated);
        prev_odom_calib_vec = t2v(prev_odom_calib);

        prev_odom = msg_trans;

        double dt = msg->header.stamp.toSec() - previousTime.toSec();

        odom_calib.twist.twist.linear.x = msg_vec_r_frame_calibrated(0) / dt;
        odom_calib.twist.twist.angular.z = msg_vec_r_frame_calibrated(2) / dt;

        // Artificially inflate twist covariance
        odom_calib.twist.covariance[0] = twist_linear_x_regression[0] * abs(odom_calib.twist.twist.linear.x) + twist_linear_x_regression[1]; // x linear
        odom_calib.twist.covariance[7] = 1e-6;//0.00075265;
        odom_calib.twist.covariance[35] = twist_angular_yaw_regression[0] * abs(odom_calib.twist.twist.angular.z) + twist_angular_yaw_regression[1]; // z angular aka yaw
    }

    //std::cout << "Calibrated vector:\n" << prev_odom_calib_vec << std::endl;

    odom_calib.pose.pose.position.x = prev_odom_calib_vec(0);
    odom_calib.pose.pose.position.y = prev_odom_calib_vec(1);
    odom_calib.pose.pose.position.z = msg->pose.pose.position.z;
    odom_calib.pose.pose.orientation = tf::createQuaternionMsgFromYaw(prev_odom_calib_vec(2));

    pub.publish(odom_calib);

    previousTime = msg->header.stamp;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "apply_odom_calib");

    Calibration calib;

    // Set calibrated odometry topic
    std::string topic_odom_calib;
    calib.n.param("odom_calibrated", topic_odom_calib, std::string("/atrvjr/odom_calibration_node/odom"));
    calib.pub = calib.n.advertise<nav_msgs::Odometry>(topic_odom_calib, 1000);

    // Get raw odometry topic
    std::string topic_odom_raw;
    calib.n.param("odom_raw", topic_odom_raw, std::string("/rflex/atrvjr_node/odom"));
    calib.sub = calib.n.subscribe<nav_msgs::Odometry>(topic_odom_raw, 10, &Calibration::calibrationCallback, &calib);

    ros::spin();
    return 0;
}
