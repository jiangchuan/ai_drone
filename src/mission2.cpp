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
#include <vector>

#define ROS_RATE 20
#define UPDATE_JUMP 4 // ROS_RATE / UPDATE_JUMP is the pos update rate

#define DELTA_SECONDS_H 5
#define DELTA_SECONDS_V 5

#define LEAST_SQUARE_LEN 30

#define DELTA_METERS_H 1.0
#define DELTA_METERS_V 1.0 // maximum dz
#define LEDDAR_RANGE 10.0
#define SAFETY_H 5.0
#define FLIGHT_ALTITUDE 1.5f

mavros_msgs::State current_state;
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

bool get_offset()
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

double regularize_angle(double ang)
{
    if (ang > M_PI)
    {
        ang -= 2 * M_PI;
    }
    return ang;
}

bool get_offset(double yaw, double x, double y, double z, double a, double b, double c, double xm, double ym)
{
    double can_a = 8.0;
    double can_c = 50.0;
    double square_sum = a * a + b * b;

    offset = fabs(a * x + b * y + c) / sqrt(square_sum);
    if (b == 0.0)
    {
        if (x > -c / a)
        {
            offset = -offset;
        }
    }
    else
    {
        if (y < -a * x / b - c / b)
        {
            offset = -offset;
        }
    }

    yaw = regularize_angle(yaw);
    if (b == 0.0)
    {
        if (yaw > 0.0)
        {
            offset = -offset;
        }
        offset /= fabs(sin(yaw));
    }
    else
    {
        double line_angle = atan(-a / b);
        if (yaw > line_angle - M_PI / 2.0 && yaw < line_angle + M_PI / 2.0)
        {
            offset = -offset;
        }
        offset /= fabs(cos(line_angle - yaw));
    }

    double xproj = (b * (b * x - a * y) - a * c) / square_sum;
    double yproj = (a * (-b * x + a * y) - b * c) / square_sum;
    double u = sqrt((xproj - xm) * (xproj - xm) + (yproj - ym) * (yproj - ym));

    vertical_dist = can_a + can_c * (cosh(u / can_c) - 1) - z;
    if (vertical_dist > LEDDAR_RANGE)
    { // Too far
        return false;
    }
    return true; // Close enough
}

double get_vertical_offset(double offset, double yaw, double line_angle)
{
    return offset * fabs(cos(line_angle - yaw));
}

double get_average(double arr[], int size)
{
    double sum = 0.0;
    for (int i = 0; i < size; i++)
    {
        sum += arr[i];
    }
    return sum / size;
}

double get_square_sum(double xarr[], double xavg, int size)
{
    double sum_de = 0.0;
    for (int i = 0; i < size; i++)
    {
        double xdiff = xarr[i] - xavg;
        sum_de += xdiff * xdiff;
    }
    return sum_de;
}

double get_cross_sum(double xarr[], double xavg, double yarr[], double yavg, int size)
{
    double sum_nu = 0.0;
    for (int i = 0; i < size; i++)
    {
        sum_nu += (xarr[i] - xavg) * (yarr[i] - yavg);
    }
    return sum_nu;
}

double get_slope(double xarr[], double xavg, double yarr[], double yavg, int size)
{
    double sum_nu = 0.0;
    double sum_de = 0.0;
    for (int i = 0; i < size; i++)
    {
        double xdiff = xarr[i] - xavg;
        sum_nu += xdiff * (yarr[i] - yavg);
        sum_de += xdiff * xdiff;
    }
    return sum_nu / sum_de;
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

void print_state_change(double yaw0, double yaw, double offset0, double offset, double alpha, double theta)
{
    ROS_INFO("  >>> status change: yaw0 = %1.1f degrees, yaw = %1.1f degrees", angles::to_degrees(yaw), angles::to_degrees(yaw));
    ROS_INFO("      offset0 = %1.2f, offset = %1.2f", offset0, offset);
    ROS_INFO("      cable orientation = %1.1f degrees", angles::to_degrees(alpha));
    ROS_INFO("      theta = %1.1f degrees", angles::to_degrees(theta));
}

void print_position(std::string header)
{
    ROS_INFO(header.c_str());
    ROS_INFO("  >>> end position: x = %1.2f, y = %1.2f, z = %1.2f", pose_stamped.pose.position.x, pose_stamped.pose.position.y, pose_stamped.pose.position.z);
    ROS_INFO("      roll = %1.1f degrees, pitch = %1.1f degrees, yaw = %1.1f degrees\n", angles::to_degrees(roll), angles::to_degrees(pitch), angles::to_degrees(yaw));
}

bool no_position_yet()
{
    geometry_msgs::Quaternion qtn = pose_in.orientation;
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

    // wait for local position feed
    while (ros::ok() && no_position_yet())
    {
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("getting local position ...");
    }

    pose_stamped.pose = pose_in;
    update_position(0.0, 0.0, DELTA_METERS_V);

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

    getRPY(pose_in.orientation); // Get yaw
    ROS_INFO("  >>> INITIAL yaw = %1.1f degrees", angles::to_degrees(yaw));

    // Wire simulation starts
    int snum_updates = 4 * DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
    double sidx = 0.0 / (double)snum_updates;
    double sidy = 0.0 / (double)snum_updates;
    double sidz = DELTA_METERS_V / (double)snum_updates;
    update_orientation(pose_in.orientation, 0.0, 0.0, 2.0 * M_PI / 2.0);
    for (int i = 0; i < snum_updates; i++)
    {
        update_position(sidx, sidy, sidz);
        for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
        {
            local_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            rate.sleep();
        }
    }
    double x0 = pose_in.position.x;
    double y0 = pose_in.position.y;
    getRPY(pose_in.orientation);               // Get yaw
    double theta = angles::from_degrees(40.0); // 20 degrees
    double alpha_wire = yaw + theta;
    alpha_wire = regularize_angle(alpha_wire);
    ROS_INFO("  >>> START yaw = %1.1f degrees", angles::to_degrees(yaw));
    ROS_INFO("  >>> TRUE cable orientation = %1.1f degrees", angles::to_degrees(alpha_wire));

    double half_wire_length = 10.0;
    double xm = x0 + half_wire_length * cos(yaw);
    double ym = y0 + half_wire_length * sin(yaw);
    // line: ax + by + c = 0
    double line_a = sin(alpha_wire);
    double line_b = -cos(alpha_wire);
    double line_c = ym * cos(alpha_wire) - xm * sin(alpha_wire);
    ROS_INFO("  >>> LINE COEFFS: line_a = %1.2f, line_b = %1.2f, line_c = %1.2f", line_a, line_b, line_c);
    // Wire simulation ends

    /* 
        Initial way point, adjust to good z
    */
    ROS_INFO("=====> going to initial way point");
    double dx = 0.0, dy = 0.0, dz = 0.0;
    bool good_initial = false;
    int num_trial = 0;
    while (!good_initial)
    {
        if (num_trial > 100)
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

        bool in_ladar_range = get_offset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);
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
            dx = -offset * sin(yaw);
            dy = offset * cos(yaw);

            double shrink_ratio = fabs(offset) / DELTA_METERS_H;
            if (shrink_ratio > 1.0)
            { // Constrain horizontal movement by DELTA_METERS_H, getting close horizontally
                dx /= shrink_ratio;
                dy /= shrink_ratio;
                dz = 0.0;
                ROS_INFO("  In ladar range, getting close horizontally, yaw = %1.1f degrees, offset = %1.2f, dx = %1.2f, dy = %1.2f", angles::to_degrees(yaw), offset, dx, dy);
            }
            else
            {
                ROS_INFO("  In ladar range, getting close vertically, yaw = %1.1f degrees, offset = %1.2f, dx = %1.2f, dy = %1.2f", angles::to_degrees(yaw), offset, dx, dy);
                double target_dist = vertical_dist - SAFETY_H;
                if (target_dist <= DELTA_METERS_V)
                {
                    dz = target_dist;
                    good_initial = true;
                    ROS_INFO("  In ladar range, last adjust, dz = %1.2f meters", dz);
                }
                else
                {
                    dz = DELTA_METERS_V;
                }
            }
        }

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
    print_position("=====> initial way point finished");

    /* 
        Way point 0, adjust to good yaw
    */
    ROS_INFO("=====> going to way point 0");
    // No need to adjust in the first step
    double alpha = 0.0;

    // Initialize
    getRPY(pose_in.orientation); // Get yaw
    dx = DELTA_METERS_H * cos(yaw);
    dy = DELTA_METERS_H * sin(yaw);
    dz = std::min(DELTA_METERS_V, vertical_dist - SAFETY_H);

    int num_updates = DELTA_SECONDS_H * ROS_RATE / UPDATE_JUMP;
    double xarr[LEAST_SQUARE_LEN];
    double yarr[LEAST_SQUARE_LEN];
    double idx = dx / (double)num_updates;
    double idy = dy / (double)num_updates;
    double idz = dz / (double)num_updates;

    std::vector<double> xvec;
    std::vector<double> yvec;

    for (int i = 0; i < num_updates; i++)
    {
        double currx = pose_in.position.x;
        double curry = pose_in.position.y;
        double currz = pose_in.position.z;
        get_offset(yaw, currx, curry, currz, line_a, line_b, line_c, xm, ym);

        // xarr[i] = currx - offset * sin(yaw);
        // yarr[i] = curry + offset * cos(yaw);
        xvec.push_back(currx - offset * sin(yaw));
        yvec.push_back(curry + offset * cos(yaw));

        update_position(idx, idy, idz);
        for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
        {
            local_pos_pub.publish(pose_stamped);
            ros::spinOnce();
            rate.sleep();
        }
    }

    int vec_size = xvec.size();
    int arr_size = LEAST_SQUARE_LEN;
    if (vec_size <= LEAST_SQUARE_LEN)
    {
        arr_size = vec_size;
    }

    int cood_jump = vec_size / arr_size;
    std::cout << "vec_size = " << vec_size << ", arr_size = " << arr_size << ", cood_jump = " << cood_jump << std::endl;

    for (int i = vec_size - 1, j = 0; j < arr_size; i -= cood_jump, j++)
    {
        xarr[j] = xvec[i];
        yarr[j] = yvec[i];
        std::cout << "xarr[j] = " << xarr[j] << ", yarr[j] = " << yarr[j] << std::endl;
    }
    double xavg = get_average(xarr, arr_size);
    double yavg = get_average(yarr, arr_size);
    double sum_de = get_square_sum(xarr, xavg, arr_size);
    if (sum_de < 1e-6)
    {
        alpha = M_PI / 2.0;
    }
    else
    {
        double sum_nu = get_cross_sum(xarr, xavg, yarr, yavg, arr_size);
        alpha = atan(sum_nu / sum_de);
    }

    // Initialize for next step
    double alpha_avg = alpha;
    get_offset(yaw, pose_in.position.x, pose_in.position.y, pose_in.position.z, line_a, line_b, line_c, xm, ym);
    double offset0 = offset;
    getRPY(pose_in.orientation); // Get yaw
    double yaw0 = yaw;
    print_state_change(yaw0, yaw, offset0, offset, alpha, alpha_avg - yaw);

    print_position("=====> way point 0 finished");

    /* 
        Common way points 
    */
    for (int iwp = 1; iwp < 10; iwp++)
    {
        ROS_INFO("=====> going to way point %d", iwp);
        double vertical_offset = get_vertical_offset(offset0, yaw0, alpha_avg);
        double omega = alpha_avg + asin(vertical_offset / DELTA_METERS_H);
        double dx = DELTA_METERS_H * cos(omega);
        double dy = DELTA_METERS_H * sin(omega);
        double dz = std::min(DELTA_METERS_V, vertical_dist - SAFETY_H);
        double dyaw = alpha_avg - yaw0; // Change this

        // TODO: Added upper limit on dz and dyaw
        double idx = dx / (double)num_updates;
        double idy = dy / (double)num_updates;
        double idz = dz / (double)num_updates;
        update_orientation(pose_in.orientation, 0.0, 0.0, dyaw);
        for (int i = 0; i < num_updates; i++)
        {
            double currx = pose_in.position.x;
            double curry = pose_in.position.y;
            double currz = pose_in.position.z;
            get_offset(yaw, currx, curry, currz, line_a, line_b, line_c, xm, ym);

            // xarr[i] = currx - offset * sin(yaw);
            // yarr[i] = curry + offset * cos(yaw);
            xvec.push_back(currx - offset * sin(yaw));
            yvec.push_back(curry + offset * cos(yaw));

            update_position(idx, idy, idz);
            for (int j = 0; ros::ok() && j < UPDATE_JUMP; j++)
            {
                local_pos_pub.publish(pose_stamped);
                ros::spinOnce();
                rate.sleep();
            }
        }

        int vec_size = xvec.size();
        int arr_size = LEAST_SQUARE_LEN;
        if (vec_size <= LEAST_SQUARE_LEN)
        {
            arr_size = vec_size;
        }

        int cood_jump = vec_size / arr_size;
        std::cout << "vec_size = " << vec_size << ", arr_size = " << arr_size << ", cood_jump = " << cood_jump << std::endl;

        for (int i = vec_size - 1, j = 0; j < arr_size; i -= cood_jump, j++)
        {
            xarr[j] = xvec[i];
            yarr[j] = yvec[i];
            std::cout << "xarr[j] = " << xarr[j] << ", yarr[j] = " << yarr[j] << std::endl;
        }
        double xavg = get_average(xarr, arr_size);
        double yavg = get_average(yarr, arr_size);
        double sum_de = get_square_sum(xarr, xavg, arr_size);
        if (sum_de < 1e-6)
        {
            alpha = M_PI / 2.0;
        }
        else
        {
            double sum_nu = get_cross_sum(xarr, xavg, yarr, yavg, arr_size);
            alpha = atan(sum_nu / sum_de);
        }

        print_state_change(yaw0, yaw, offset0, offset, alpha, alpha - yaw);

        // Initialize for next step
        alpha_avg = lambda * alpha + (1 - lambda) * alpha_avg;
        offset0 = offset;
        getRPY(pose_in.orientation); // Get yaw
        yaw0 = yaw;
        std::string pos_header = "=====> way point " + std::to_string(iwp) + " finished";
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
