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

#include <cmath>


#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;

void state_callback(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

float travel_dist = 2;
float roll = 0;
float pitch = 0;
float yaw = 0;
geometry_msgs::Quaternion current_qtn_msg;

void imu_callback(const sensor_msgs::Imu::ConstPtr& msg) {
    current_qtn_msg = msg->orientation;
    // ROS_INFO("Hello");
    // ROS_INFO("Imu Seq: [%d]", msg->header.seq);
    // ROS_INFO("Imu Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    // ROS_INFO("Imu Angular Velocity roll: [%f], pitch: [%f], yaw: [%f]", msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    // ROS_INFO("Imu Linear Acceleration x: [%f], y: [%f], z: [%f]", msg->linear_acceleration.x,msg->linear_acceleration.y,msg->linear_acceleration.z);
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
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCT...");
    }

    tf::Quaternion current_qtn;
    // tf::Quaternion current_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    tf::Quaternion delta_qtn;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

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
    while(ros::ok() && !current_state.armed){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
          ROS_INFO(current_state.mode.c_str());
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
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

    // go to the first waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    ROS_INFO("going to the first way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("first way point finished!");


    // go to the second waypoint
    pose.pose.position.x = 0;
    pose.pose.position.y = travel_dist;
    pose.pose.position.z = FLIGHT_ALTITUDE;



    // here your desired angles
    yaw = M_PI / 6;
    delta_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    // quaternionMsgToTF(commanded_pose.pose.orientation , q_orig);  // Get the original orientation of 'commanded_pose'
    
    quaternionMsgToTF(current_qtn_msg , current_qtn);
    current_qtn = delta_qtn * current_qtn;
    current_qtn.normalize();
    pose.pose.orientation.x = current_qtn.x();
    pose.pose.orientation.y = current_qtn.y();
    pose.pose.orientation.z = current_qtn.z();
    pose.pose.orientation.w = current_qtn.w();


    // quaternionTFToMsg(q_new, commanded_pose.pose.orientation); 







    //send setpoints for 10 seconds
    ROS_INFO("going to second way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("second way point finished!");

    // go to the third waypoint
    pose.pose.position.x = travel_dist;
    pose.pose.position.y = travel_dist;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // here your desired angles
    yaw = -M_PI / 6;
    delta_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    quaternionMsgToTF(current_qtn_msg , current_qtn);
    current_qtn = delta_qtn * current_qtn;
    current_qtn.normalize();
    pose.pose.orientation.x = current_qtn.x();
    pose.pose.orientation.y = current_qtn.y();
    pose.pose.orientation.z = current_qtn.z();
    pose.pose.orientation.w = current_qtn.w();

    //send setpoints for 10 seconds
    ROS_INFO("going to third way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("third way point finished!");
    
    // go to the forth waypoint
    pose.pose.position.x = travel_dist;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // here your desired angles
    yaw = M_PI / 6;
    delta_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    quaternionMsgToTF(current_qtn_msg , current_qtn);
    current_qtn = delta_qtn * current_qtn;
    current_qtn.normalize();
    pose.pose.orientation.x = current_qtn.x();
    pose.pose.orientation.y = current_qtn.y();
    pose.pose.orientation.z = current_qtn.z();
    pose.pose.orientation.w = current_qtn.w();

    //send setpoints for 10 seconds
    ROS_INFO("going to forth way point");
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }
    ROS_INFO("forth way point finished!");
    
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = FLIGHT_ALTITUDE;

    // here your desired angles
    yaw = M_PI / 6;
    delta_qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    quaternionMsgToTF(current_qtn_msg , current_qtn);
    current_qtn = delta_qtn * current_qtn;
    current_qtn.normalize();
    pose.pose.orientation.x = current_qtn.x();
    pose.pose.orientation.y = current_qtn.y();
    pose.pose.orientation.z = current_qtn.z();
    pose.pose.orientation.w = current_qtn.w();

    ROS_INFO("going back to the first point!");
    //send setpoints for 10 seconds
    for(int i = 0; ros::ok() && i < 10*20; ++i){
      local_pos_pub.publish(pose);
      ros::spinOnce();
      rate.sleep();
    }

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
