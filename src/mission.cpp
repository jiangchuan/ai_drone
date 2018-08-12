/**
 * @file offb_node.cpp
 * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 * stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include "sensor_msgs/Imu.h"
#include "tf/tf.h"

#include <math.h>
#include <cmath>


#define ROS_RATE 20
#define DELTA_SECONDS 10
#define DELTA_METERS 1.0

#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

double x = 0.0, y = 0.0, z = 0.0;
double roll = 0.0, pitch = 0.0, yaw = 0.0;

double lambda = 0.8;

geometry_msgs::Quaternion current_qtn_msg;
tf::Quaternion current_qtn;
geometry_msgs::PoseStamped pose;


double getOffset(double x, double y) {
    return -x;
}


void getRPY() {
    // ROS_INFO("getRPY: x = %1.1f, y = %1.1f, z = %1.1f, w = %1.1f", current_qtn.x(), current_qtn.y(), current_qtn.z(), current_qtn.w());

    tf::Matrix3x3 m(current_qtn);
    m.getRPY(roll, pitch, yaw);
    // ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
}








void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    current_qtn_msg = msg->orientation;
    // ROS_INFO("Hello");
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    // ROS_INFO("Imu Angular Velocity roll: [%f], pitch: [%f], yaw: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    // ROS_INFO("Imu Linear Acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
}


void set_waypoint(double x, double y, double z, double roll, double pitch, double yaw) {

    // ROS_INFO("before: x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // ROS_INFO("after: x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

    current_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    current_qtn.normalize();

    // ROS_INFO("x = %1.1f, y = %1.1f, z = %1.1f, w = %1.1f", current_qtn.x(), current_qtn.y(), current_qtn.z(), current_qtn.w());
    quaternionTFToMsg(current_qtn, pose.pose.orientation);
}

void compute_waypoint(double dx, double dy, double dz, double droll, double dpitch, double dyaw) {
    pose.pose.position.x += dx;
    pose.pose.position.y += dy;
    pose.pose.position.z += dz;

    // ROS_INFO("x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);


    tf::Quaternion delta_qtn = tf::createQuaternionFromRPY(droll, dpitch, dyaw);

    quaternionMsgToTF(current_qtn_msg , current_qtn);
    current_qtn = delta_qtn * current_qtn;
    current_qtn.normalize();
    // ROS_INFO("x = %1.1f, y = %1.1f, z = %1.1f, w = %1.1f", current_qtn.x(), current_qtn.y(), current_qtn.z(), current_qtn.w());

    quaternionTFToMsg(current_qtn, pose.pose.orientation);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_callback);

    ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>
          ("mavros/imu/data", 100, imu_callback);

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>
      ("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate((double) ROS_RATE);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }


    // current_qtn = tf::createQuaternionFromRPY(0.0, 0.0, 0.0);
    current_qtn.normalize();
    quaternionTFToMsg(current_qtn, pose.pose.orientation);

    //send a few setpoints before starting
    for(int i = 100; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    // change to offboard mode and arm
    while(ros::ok() && !current_state.armed) {
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());  // Sometimes this does not show, and the drone does not fly.
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled"); // Sometimes this does not show, and the drone does not fly.
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("going to initial way point =====>");
    set_waypoint(1.0, 0.0, FLIGHT_ALTITUDE, 0.0, 0.0, M_PI / 6);
    for(int i = 0; ros::ok() && i < DELTA_SECONDS * ROS_RATE; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    getRPY(); // Get yaw
    ROS_INFO("  x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);    
    ROS_INFO("  roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);    
    ROS_INFO("=====> initial way point finished!\n");



    ROS_INFO("going to way point 0 =====>");
    // No need to adjust in the first step


    double dx = 0.0, dy = 0.0, dz = 0.0;
    double offset0 = 0.0, offset = 0.0;
    bool good_yaw = false;

    while (!good_yaw) {
        // Initialize
        offset0 = getOffset(pose.pose.position.x, pose.pose.position.y);
        getRPY(); // Get yaw 
        ROS_INFO("Current yaw = %1.1f degrees", yaw * 180.0 / M_PI);

        dx = DELTA_METERS * cos(yaw);
        dy = DELTA_METERS * sin(yaw);
        dz = 0.0;
        // TODO: Added upper limit on dz and dyaw

        compute_waypoint(dx, dy, dz, 0.0, 0.0, 0.0);
        //send setpoints for 10 seconds
        for(int i = 0; ros::ok() && i < DELTA_SECONDS * ROS_RATE; ++i) {
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        offset = getOffset(pose.pose.position.x, pose.pose.position.y);
        if (offset > 0.8 * DELTA_METERS && offset0 > 0 && offset > offset0) { // on the left of cable
            // Go back to initial point and restart
            compute_waypoint(-dx, -dy, -dz, 0.0, 0.0, -5.0 * M_PI / 180.0); // decrease yaw by 5 degrees
            //send setpoints for 10 seconds
            for(int i = 0; ros::ok() && i < DELTA_SECONDS * ROS_RATE; ++i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            continue;
        }

        if (offset < -0.8 * DELTA_METERS && offset0 < 0 && offset < offset0) { // on the right of cable
            // Go back to initial point and restart
            compute_waypoint(-dx, -dy, -dz, 0.0, 0.0, 5.0 * M_PI / 180.0); // increase yaw by 5 degrees
            //send setpoints for 10 seconds
            for(int i = 0; ros::ok() && i < DELTA_SECONDS * ROS_RATE; ++i) {
                local_pos_pub.publish(pose);
                ros::spinOnce();
                rate.sleep();
            }
            continue;
        }

        good_yaw = true;
    }


    double theta0 = asin((offset - offset0) / DELTA_METERS);
    ROS_INFO("offset0 = %1.1f, offset = %1.1f", offset0, offset);
    double alpha = yaw - theta0;
    ROS_INFO("yaw = %1.1f degrees, theta0 = %1.1f degrees, alpha = %1.1f degrees", yaw * 180.0 / M_PI, theta0 * 180.0 / M_PI, alpha * 180.0 / M_PI);

    ROS_INFO("offset0 = %1.1f, offset = %1.1f, yaw = %1.1f degrees", offset0, offset, yaw * 180.0 / M_PI);


    // Initialize for next step
    double alpha_avg = alpha;
    offset0 = offset;
    getRPY();
    double yaw0 = yaw;
    ROS_INFO("Cable orientation = %1.1f degrees", alpha_avg * 180.0 / M_PI);
    getRPY(); // Get yaw
    ROS_INFO("  x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);    
    ROS_INFO("  roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);    
    ROS_INFO("=====> way point 0 finished!\n");


    for (int iwp = 1; iwp < 5; iwp++) {
        ROS_INFO("going to way point %d =====>", iwp);
        ROS_INFO("Current yaw = %1.1f degrees", yaw * 180.0 / M_PI);
        yaw = alpha_avg + asin(-offset / DELTA_METERS); // Adjust yaw
        double dx = DELTA_METERS * cos(yaw);
        double dy = DELTA_METERS * sin(yaw);
        double dz = 0.0;
        double dyaw = alpha_avg - yaw0; // Change this
        ROS_INFO("cable orientation = %1.1f degrees, yaw0 = %1.1f degrees, yaw = %1.1f degrees, delta yaw = %1.1f degrees", alpha_avg * 180.0 / M_PI, yaw0 * 180.0 / M_PI, yaw * 180.0 / M_PI, dyaw * 180.0 / M_PI);
        // TODO: Added upper limit on dz and dyaw
        compute_waypoint(dx, dy, dz, 0.0, 0.0, dyaw);

        //send setpoints for 10 seconds
        for(int i = 0; ros::ok() && i < DELTA_SECONDS * ROS_RATE; ++i) {
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }

        offset = getOffset(pose.pose.position.x, pose.pose.position.y);
        double theta0 = asin((offset - offset0) / DELTA_METERS);
        ROS_INFO("offset0 = %1.1f, offset = %1.1f", offset0, offset);
        double alpha = yaw - theta0;
        ROS_INFO("yaw = %1.1f degrees, theta0 = %1.1f degrees, alpha = %1.1f degrees", yaw * 180.0 / M_PI, theta0 * 180.0 / M_PI, alpha * 180.0 / M_PI);

        // Initialize for next step
        alpha_avg = alpha * lambda + alpha_avg * (1 - lambda);
        offset0 = offset;
        getRPY();
        yaw0 = yaw;
        getRPY(); // Get yaw
        ROS_INFO("  x = %1.1f, y = %1.1f, z = %1.1f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);    
        ROS_INFO("  roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees", roll * 180.0 / M_PI, pitch * 180.0 / M_PI, yaw * 180.0 / M_PI);    
        ROS_INFO("=====> way point %d finished!\n", iwp);
    }




        // double offset = getOffset(pose.pose.position.x, pose.pose.position.y);
        // getRPY();
        // yaw0 = yaw;

        // double theta0 = asin((offset - offset0) / DELTA_METERS);
        // double alpha = yaw0 - theta0;
        // alpha_avg = alpha * lambda + alpha_avg * (1 - lambda);
        // ROS_INFO("Cable orientation = %1.1f degrees", alpha_avg * 180.0 / M_PI);



    ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
      ros::spinOnce();
      rate.sleep();
    }
    return 0;
}
