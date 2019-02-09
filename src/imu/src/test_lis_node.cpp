#include <ros/ros.h>
#include <tf/transform_listener.h>

#include "imu/Pitch.h"

// get roll, pitch and yaw expression of quaternion
// express by Pitch -> Roll -> Yaw order,
// instead of standard Yaw -> Pitch -> Roll order.
void getPRY(tf::Quaternion &q, double &roll, double &pitch, double &yaw) {
    // false q. swapping (x, y, z) -> (z, x, y)
    tf::Quaternion qf{q.z(), q.x(), q.y(), q.w()};
    // false fall of getRPY. swapping (Y, P, R) -> (P, R, Y)
    tf::Matrix3x3(qf).getRPY(yaw, roll, pitch);
}

// get roll, pitch and yaw expression of quaternion
// express by Roll -> Yaw -> Pitch order,
// instead of standard Yaw -> Pitch -> Roll order.
void getRYP(tf::Quaternion &q, double &roll, double &pitch, double &yaw) {
    // false q. swapping (x, y, z) -> (y, z, x)
    tf::Quaternion qf{q.y(), q.z(), q.x(), q.w()};
    // false fall of getRPY. swapping (Y, P, R) -> (R, Y, P)
    tf::Matrix3x3(qf).getRPY(pitch, yaw, roll);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "test_lis_node");

    ros::NodeHandle node;

    tf::TransformListener listener;
    ros::Publisher pitch_pub = node.advertise<imu::Pitch>("pitch", 10);

    ros::Rate rate(30.0);

    while (node.ok()){
        tf::StampedTransform tr;

        try{
            listener.lookupTransform("map", "imu",
                    ros::Time(0), tr);
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
        }

        tf::Quaternion rot = tr.getRotation();

        imu::Pitch msg;
        msg.header.stamp = ros::Time::now();

        double roll, pitch, yaw;
        tf::Matrix3x3(rot).getRPY(roll, pitch, yaw);
        ROS_INFO("YPR: %+3.1f %+3.1f %+3.1f",
                180.0 / M_PI * roll, 180.0 / M_PI * pitch, 180.0 / M_PI * yaw);
        msg.pitch_YPR = pitch;

        getPRY(rot, roll, pitch, yaw);
        ROS_DEBUG("PRY: %+3.1f %+3.1f %+3.1f",
                180.0 / M_PI * roll, 180.0 / M_PI * pitch, 180.0 / M_PI * yaw);
        msg.pitch_PRY = pitch;

        getRYP(rot, roll, pitch, yaw);
        ROS_INFO("RYP: %+3.1f %+3.1f %+3.1f",
                180.0 / M_PI * roll, 180.0 / M_PI * pitch, 180.0 / M_PI * yaw);
        msg.pitch_RYP = pitch;

        pitch_pub.publish(msg);

        rate.sleep();
    }
    return 0;
};

