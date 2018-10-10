/**
 * @file mission.cpp
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
#include "sensor_msgs/NavSatFix.h"
// #include <sensor_msgs/LaserScan.h>
#include "tf/tf.h"
#include <angles/angles.h>

#include <sstream>
#include <fstream>
#include <math.h>
#include <cmath>
#include <algorithm>

#define ROS_RATE 20
#define UPDATE_JUMP 4 // ROS_RATE / UPDATE_JUMP is the pos update rate

#define DELTA_SECONDS_H 5
#define DELTA_SECONDS_V 5

#define DELTA_METERS_H 1.0
#define DELTA_METERS_V 1.0 // maximum dz
#define LEDDAR_RANGE 10.0
#define SAFETY_H 5.0
#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
// geometry_msgs::Quaternion current_qtn_msg;
geometry_msgs::Pose pose_in;
geometry_msgs::PoseStamped pose_stamped;

double x = 0.0, y = 0.0, z = 0.0;
double roll = 0.0, pitch = 0.0, yaw = 0.0;
double lambda = 0.8;
double offset = 0.0;
double vertical_dist = 1000.0;
double segment_angle = angles::from_degrees(2.5); // 2.5 degrees

void state_callback(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

void local_pos_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    pose_in = msg->pose;

    // current_qtn_msg = msg->pose.orientation;

    // ROS_INFO("Local Pos Seq: [%d]", msg->header.seq);
    // ROS_INFO("Local Pos Position x: [%f], y: [%f], z: [%f]", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
    // ROS_INFO("Local Pos Orientation x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
}

// void gps_callback(const sensor_msgs::NavSatFix::ConstPtr &msg)
// {
//     ROS_INFO("GPS Seq: [%d]", msg->header.seq);
//     ROS_INFO("GPS latitude: [%f], longitude: [%f], altitude: [%f]", msg->latitude, msg->longitude, msg->altitude);
// }

std::vector<std::string> split_str(const std::string &s, char delimiter)
{
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(s);
    while (std::getline(tokenStream, token, delimiter))
    {
        tokens.push_back(token);
    }
    return tokens;
}

void getOffset(double x, double y)
{
    double a = sqrt(3.0) / 3.0;
    double b = 1.0;
    double c = -1.0;
    offset = (a * x + b * y + c) / sqrt(a * a + b * b);

    // return y;
    // return -x;
}

bool getOffset()
{
    std::stringstream ss;
    std::string line;
    std::ifstream myfile("/home/jiangchuan/catkin_ws/src/ai_drone/src/leddar_results.txt");
    if (myfile.is_open())
    {
        while (getline(myfile, line))
        {
            ss << line << '\n';
        }
        myfile.close();
    }
    else
    {
        ss << "Unable to open file";
    }

    std::vector<std::string> leddar_records = split_str(ss.str(), '|');
    double min_dist = 1000.0;
    double the_seg_num = 0.0;
    for (std::vector<std::string>::iterator it = leddar_records.begin(); it != leddar_records.end(); ++it)
    {
        std::vector<std::string> segment_records = split_str(*it, ',');

        double dist = std::stod(segment_records[1]);
        if (min_dist > dist)
        {
            min_dist = dist;
            the_seg_num = std::stod(segment_records[0]);
        }
    }

    double offset_angle = (3.5 - the_seg_num) * segment_angle;
    offset = min_dist * sin(offset_angle);
    vertical_dist = min_dist * cos(offset_angle);
    ROS_INFO("the_seg_num = %f, min_dist = %f, offset = %f, vertical_dist = %f", the_seg_num, min_dist, offset, vertical_dist);
    if (vertical_dist > LEDDAR_RANGE)
    { // Too far
        return false;
    }
    return true; // Close enough
}

bool getOffset(double yaw, double x, double y, double z, double a, double b, double c, double xm, double ym)
{
    // ROS_INFO("current position x = %f, y = %f, z = %f", x, y, z);
    double can_a = 8.0;
    double can_c = 50.0;

    double square_sum = a * a + b * b;

    // if (a < 0.0 || (a == 0.0 && b > 0.0))
    // {
    //     a = -a;
    //     b = -b;
    //     c = -c;
    // }

    offset = fabs(a * x + b * y + c) / sqrt(square_sum);

    if (yaw >= M_PI)
    {
        yaw -= 2 * M_PI;
    }

    if (b == 0)
    {
        if (yaw > M_PI / 2.0 || (yaw > -M_PI / 2.0 && yaw < 0.0))
        {
            offset = -offset;
        }
    }
    else
    {
        double line_angle = atan(-a / b);
        if (yaw < line_angle - M_PI / 2.0 || (yaw > line_angle && yaw < line_angle + M_PI / 2.0))
        {
            offset = -offset;
        }
    }

    double xproj = (b * (b * x - a * y) - a * c) / square_sum;
    double yproj = (a * (-b * x + a * y) - b * c) / square_sum;

    double u = sqrt((xproj - xm) * (xproj - xm) + (yproj - ym) * (yproj - ym));

    ROS_INFO("current position x = %f, y = %f, z = %f, xm = %f, ym = %f, u = %f", x, y, z, xm, ym, u);

    vertical_dist = can_a + can_c * (cosh(u / can_c) - 1) - z;
    ROS_INFO("offset = %f, vertical_dist = %f", offset, vertical_dist);
    if (vertical_dist > LEDDAR_RANGE)
    { // Too far
        return false;
    }
    return true; // Close enough
}

void getRPY(geometry_msgs::Quaternion qtn_msg)
{
    tf::Quaternion qtn;
    quaternionMsgToTF(qtn_msg, qtn);
    qtn.normalize();
    tf::Matrix3x3 m(qtn);
    m.getRPY(roll, pitch, yaw);
}

void set_waypoint(double x, double y, double z, double roll, double pitch, double yaw)
{
    pose_stamped.pose.position.x = x;
    pose_stamped.pose.position.y = y;
    pose_stamped.pose.position.z = z;
    tf::Quaternion qtn = tf::createQuaternionFromRPY(roll, pitch, yaw);
    qtn.normalize();
    quaternionTFToMsg(qtn, pose_stamped.pose.orientation);
}

void update_position(double dx, double dy, double dz)
{
    pose_stamped.pose.position.x += dx;
    pose_stamped.pose.position.y += dy;
    pose_stamped.pose.position.z += dz;
}

void update_orientation(geometry_msgs::Quaternion qtn_msg, double droll, double dpitch, double dyaw)
{
    tf::Quaternion delta_qtn = tf::createQuaternionFromRPY(droll, dpitch, dyaw);
    tf::Quaternion qtn;
    quaternionMsgToTF(qtn_msg, qtn);
    qtn = delta_qtn * qtn;
    qtn.normalize();
    quaternionTFToMsg(qtn, pose_stamped.pose.orientation);
}

void print_position(std::string header)
{
    ROS_INFO(header.c_str());
    ROS_INFO("  x = %1.2f, y = %1.2f, z = %1.2f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
    ROS_INFO("  roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", angles::to_degrees(roll), angles::to_degrees(pitch), angles::to_degrees(yaw));
}

bool no_position_yet()
{
    geometry_msgs::Quaternion qtn = pose_in.orientation;
    // geometry_msgs::Point pt = pose_in.position;
    // return fabs(qtn.x) + fabs(qtn.y) + fabs(qtn.z) + fabs(qtn.w) + fabs(pt.x) + fabs(pt.y) + fabs(pt.z) < 1e-6;
    return fabs(qtn.x) + fabs(qtn.y) + fabs(qtn.z) + fabs(qtn.w) < 1e-6;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mission_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    ros::Subscriber local_pos_sub = nh.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose", 10, local_pos_callback);
    // ros::Subscriber gps_sub = nh.subscribe<sensor_msgs::NavSatFix>("mavros/global_position/global", 10, gps_callback);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient land_client = nh.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate((double)ROS_RATE);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("connecting to FCU ...");
    }

    // // Wait for 5 seconds to get local position
    // for (int i = 100; ros::ok() && i > 0; --i)
    // {
    //     ros::spinOnce();
    //     rate.sleep();
    // }
    // ros::Duration(5).sleep();

    while (ros::ok() && no_position_yet())
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("getting local position ...");
    }

    // // Wire simulation starts
    // double x0 = pose_in.position.x;
    // double y0 = pose_in.position.y;
    // getRPY(pose_in.orientation);                 // Get yaw
    // double alpha0 = angles::from_degrees(-20.0); // 20 degrees
    // double theta_wire = yaw + alpha0;
    // double half_wire_length = 10.0;
    // double xm = x0 + half_wire_length * cos(yaw);
    // double ym = y0 + half_wire_length * sin(yaw);
    // // line: ax + by + c = 0
    // double line_a = sin(theta_wire);
    // double line_b = -cos(theta_wire);
    // double line_c = ym * cos(theta_wire) - xm * sin(theta_wire);
    // // Wire simulation ends

    pose_stamped.pose = pose_in;

    //send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    // change to offboard mode
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();
    while (ros::ok() && current_state.mode != "OFFBOARD")
    {
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            ROS_INFO(current_state.mode.c_str());
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        local_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    // arm
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while (ros::ok() && !current_state.armed)
    {
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if (arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }
        local_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0.0f;
    land_cmd.request.latitude = 0.0f;
    land_cmd.request.longitude = 0.0f;
    land_cmd.request.altitude = 0.0f;

    // Wire simulation starts
    int num_updates = 4 * DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
    double idx = 0.0 / (double)num_updates;
    double idy = 0.0 / (double)num_updates;
    double idz = DELTA_METERS_V / (double)num_updates;
    // double idyaw = M_PI / 2.0 / (double)num_updates;
    update_orientation(pose_in.orientation, 0.0, 0.0, 0.0 * M_PI / 2.0);
    for (int i = 0; i < num_updates; i++)
    {
        update_position(idx, idy, idz);
        for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
        {
            local_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            rate.sleep();
        }
    }
    double x0 = pose_in.position.x;
    double y0 = pose_in.position.y;
    getRPY(pose_in.orientation);                // Get yaw
    double alpha0 = angles::from_degrees(40.0); // 20 degrees
    double theta_wire = yaw + alpha0;
    ROS_INFO("theta_wire = %1.1f degrees", angles::to_degrees(theta_wire));

    double half_wire_length = 10.0;
    double xm = x0 + half_wire_length * cos(yaw);
    double ym = y0 + half_wire_length * sin(yaw);
    // line: ax + by + c = 0
    double line_a = sin(theta_wire);
    double line_b = -cos(theta_wire);
    double line_c = ym * cos(theta_wire) - xm * sin(theta_wire);
    // Wire simulation ends

    /* 
        Initial way point, adjust to good z
    */
    ROS_INFO("going to initial way point =====>");
    double dx = 0.0, dy = 0.0, dz = 0.0;
    bool good_initial = false;
    int num_trial = 0;
    while (!good_initial)
    {
        if (num_trial > 50)
        {
            ROS_INFO("tring to land");
            while (ros::ok())
            {
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    if (land_client.call(land_cmd) && land_cmd.response.success)
                    {
                        ROS_INFO("Landing");
                        break;
                    }
                    last_request = ros::Time::now();
                }
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
            return 0;
        }
        num_trial++;

        // bool in_ladar_range = getOffset();
        bool in_ladar_range = getOffset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);

        if (!in_ladar_range)
        { // Far away
            ROS_INFO("  Not in ladar range yet");
            dx = 0.0;
            dy = 0.0;
            dz = DELTA_METERS_V;
        }
        else
        {                                // Getting close vertically
            getRPY(pose_in.orientation); // Get yaw
            dx = offset * sin(yaw);
            dy = -offset * cos(yaw);

            double shrink_ratio = DELTA_METERS_H / fabs(offset);
            if (shrink_ratio < 1.0)
            { // Constrain horizontal movement by DELTA_METERS_H, getting close horizontally
                dx *= shrink_ratio;
                dy *= shrink_ratio;
                dz = 0.0;
                ROS_INFO("  In ladar range, getting close horizontally, current yaw = %1.1f degrees, dx = %1.1f, dy = %1.1f", angles::to_degrees(yaw), dx, dy);
            }
            else
            {
                ROS_INFO("  In ladar range, getting close vertically, current yaw = %1.1f degrees, dx = %1.1f, dy = %1.1f", angles::to_degrees(yaw), dx, dy);
                double target_dist = vertical_dist - SAFETY_H;
                if (target_dist <= DELTA_METERS_V)
                {
                    dz = target_dist;
                    good_initial = true;
                    ROS_INFO("Last adjust, dz = %1.1f meters", dz);
                }
                else
                {
                    dz = DELTA_METERS_V;
                }
            }
        }

        // update_position(dx, dy, dz);
        // for(int i = 0; ros::ok() && i < DELTA_SECONDS_V * ROS_RATE; ++i){
        //   local_pos_pub.publish(pose_stamped);
        //   ros::spinOnce();
        //   rate.sleep();
        // }

        int num_updates = DELTA_SECONDS_V * ROS_RATE / UPDATE_JUMP;
        double idx = dx / (double)num_updates;
        double idy = dy / (double)num_updates;
        double idz = dz / (double)num_updates;
        for (int i = 0; i < num_updates; i++)
        {
            update_position(idx, idy, idz);
            for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
            {
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
        }
    }
    getRPY(pose_in.orientation); // Get yaw
    print_position("=====> initial way point finished!");

    /* 
        Way point 0, adjust to good yaw
    */
    ROS_INFO("going to way point 0 =====>");
    // No need to adjust in the first step
    double offset0 = 0.0;
    double theta0 = 0.0, alpha = 0.0;
    bool good_yaw = false;

    num_trial = 0;
    while (!good_yaw)
    {
        if (num_trial > 50)
        {
            ROS_INFO("tring to land");
            while (ros::ok())
            {
                if (ros::Time::now() - last_request > ros::Duration(5.0))
                {
                    if (land_client.call(land_cmd) && land_cmd.response.success)
                    {
                        ROS_INFO("Landing");
                        break;
                    }
                    last_request = ros::Time::now();
                }
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
            return 0;
        }
        num_trial++;

        // Initialize
        // getOffset(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        // getOffset();
        getOffset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);

        offset0 = offset;
        getRPY(pose_in.orientation); // Get yaw
        ROS_INFO("  Current yaw = %1.1f degrees", angles::to_degrees(yaw));

        dx = DELTA_METERS_H * cos(yaw);
        dy = DELTA_METERS_H * sin(yaw);
        dz = std::min(DELTA_METERS_V, vertical_dist - SAFETY_H);

        // update_position(dx, dy, dz);
        // for(int i = 0; ros::ok() && i < DELTA_SECONDS_H * ROS_RATE; ++i) {
        //     local_pos_pub.publish(pose_stamped);
        //     ros::spinOnce();
        //     rate.sleep();
        // }

        int num_updates = DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
        double idx = dx / (double)num_updates;
        double idy = dy / (double)num_updates;
        double idz = dz / (double)num_updates;
        for (int i = 0; i < num_updates; i++)
        {
            update_position(idx, idy, idz);
            for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
            {
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
        }

        // getOffset(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        // getOffset();
        getOffset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);

        theta0 = asin((offset - offset0) / DELTA_METERS_H);
        alpha = yaw - theta0;
        ROS_INFO("  yaw = %1.1f degrees, theta0 = %1.1f degrees, alpha = %1.1f degrees", angles::to_degrees(yaw), angles::to_degrees(theta0), angles::to_degrees(alpha));
        ROS_INFO("  offset0 = %1.2f, offset = %1.2f, yaw = %1.1f degrees", offset0, offset, angles::to_degrees(yaw));

        dx = offset * sin(alpha);
        dy = -offset * cos(alpha);
        dz = std::min(DELTA_METERS_V, vertical_dist - SAFETY_H);
        double dyaw = alpha - yaw; // Change this

        if ((offset > 0.5 * DELTA_METERS_H && offset0 > 0 && offset > offset0) || (offset < -0.5 * DELTA_METERS_H && offset0 < 0 && offset < offset0))
        { // on the left of cable
            // update_position(dx, dy, dz); // Go back to cable
            // update_orientation(pose_in.orientation, 0.0, 0.0, dyaw);
            // for(int i = 0; ros::ok() && i < DELTA_SECONDS_H * ROS_RATE; ++i) {
            //     local_pos_pub.publish(pose_stamped);
            //     ros::spinOnce();
            //     rate.sleep();
            // }

            int num_updates = DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
            double idx = dx / (double)num_updates;
            double idy = dy / (double)num_updates;
            double idz = dz / (double)num_updates;
            // double idyaw = dyaw / (double)num_updates;
            update_orientation(pose_in.orientation, 0.0, 0.0, dyaw);
            for (int i = 0; i < num_updates; i++)
            {
                update_position(idx, idy, idz);
                // update_orientation(pose_in.orientation, 0.0, 0.0, idyaw);
                for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
                {
                    local_pos_pub.publish(pose_stamped);
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            continue;
        }

        good_yaw = true;
    }

    // Initialize for next step
    double alpha_avg = alpha;
    offset0 = offset;
    getRPY(pose_in.orientation); // Get yaw
    double yaw0 = yaw;
    ROS_INFO("Cable orientation = %1.1f degrees", angles::to_degrees(alpha_avg));
    print_position("=====> way point 0 finished!");

    /* 
        Common way points 
    */
    for (int iwp = 1; iwp < 20; iwp++)
    {
        ROS_INFO("going to way point %d =====>", iwp);
        ROS_INFO("  Current yaw = %1.1f degrees", angles::to_degrees(yaw));
        yaw = alpha_avg + asin(-offset / DELTA_METERS_H); // Adjust yaw
        double dx = DELTA_METERS_H * cos(yaw);
        double dy = DELTA_METERS_H * sin(yaw);
        double dz = std::min(DELTA_METERS_V, vertical_dist - SAFETY_H);
        double dyaw = alpha_avg - yaw0; // Change this
        ROS_INFO("  cable orientation = %1.1f degrees, yaw0 = %1.1f degrees, yaw = %1.1f degrees, delta yaw = %1.1f degrees", angles::to_degrees(alpha_avg), angles::to_degrees(yaw0), angles::to_degrees(yaw), angles::to_degrees(dyaw));

        // TODO: Added upper limit on dz and dyaw
        // update_position(dx, dy, dz);
        // update_orientation(pose_in.orientation, 0.0, 0.0, dyaw);
        // for(int i = 0; ros::ok() && i < DELTA_SECONDS_H * ROS_RATE; ++i) {
        //     local_pos_pub.publish(pose_stamped);
        //     ros::spinOnce();
        //     rate.sleep();
        // }

        int num_updates = DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
        double idx = dx / (double)num_updates;
        double idy = dy / (double)num_updates;
        double idz = dz / (double)num_updates;
        // double idyaw = dyaw / (double)num_updates;
        update_orientation(pose_in.orientation, 0.0, 0.0, dyaw);
        for (int i = 0; i < num_updates; i++)
        {
            update_position(idx, idy, idz);
            // update_orientation(pose_in.orientation, 0.0, 0.0, idyaw);
            for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
            {
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
        }

        // getOffset(pose_stamped.pose.position.x, pose_stamped.pose.position.y);
        // getOffset();
        getOffset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);

        double theta0 = asin((offset - offset0) / DELTA_METERS_H);
        ROS_INFO("  offset0 = %1.2f, offset = %1.2f", offset0, offset);
        double alpha = yaw - theta0;
        ROS_INFO("  yaw = %1.1f degrees, theta0 = %1.1f degrees, alpha = %1.1f degrees", angles::to_degrees(yaw), angles::to_degrees(theta0), angles::to_degrees(alpha));

        // Initialize for next step
        alpha_avg = alpha * lambda + alpha_avg * (1 - lambda);
        offset0 = offset;
        getRPY(pose_in.orientation); // Get yaw
        yaw0 = yaw;
        // ROS_INFO("  x = %1.2f, y = %1.2f, z = %1.2f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
        // ROS_INFO("  roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees", angles::to_degrees(roll), angles::to_degrees(pitch), angles::to_degrees(yaw));
        // ROS_INFO("=====> way point %d finished!\n", iwp);

        std::string pos_header = "=====> way point " + std::to_string(iwp) + " finished!";
        print_position(pos_header);
    }

    ROS_INFO("tring to land");
    while (ros::ok())
    {
        if (ros::Time::now() - last_request > ros::Duration(5.0))
        {
            if (land_client.call(land_cmd) && land_cmd.response.success)
            {
                ROS_INFO("Landing");
                break;
            }
            last_request = ros::Time::now();
        }
        local_pos_pub.publish(pose_stamped);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
