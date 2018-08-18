#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>
#include <fstream>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>

#define BEAM_COUNT 8

double field_of_view = 20.0; // degrees
double max_range = 20.0; // meters

std::vector<std::string> split_str(const std::string& s, char delimiter) {
   std::vector<std::string> tokens;
   std::string token;
   std::istringstream tokenStream(s);
   while (std::getline(tokenStream, token, delimiter))
   {
      tokens.push_back(token);
   }
   return tokens;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "leddar_pub_node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  // ros::Publisher leddar_pub = n.advertise<std_msgs::String>("leddar_publisher", 1000);
  ros::Publisher leddar_pub = n.advertise<sensor_msgs::LaserScan>("leddar_publisher", 100);


  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  while (ros::ok()) {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    // std_msgs::String msg;
    std::stringstream ss;
    std::string line;
    std::ifstream myfile ("/home/jiangchuan/catkin_ws/src/ai_drone/src/leddar_results.txt");
    if (myfile.is_open()) {
      while ( getline (myfile,line) ) {
        ss << line << '\n';
      }
      myfile.close();
    } else {
        ss << "Unable to open file";
    } 


    // Construct LaserScan message.
    sensor_msgs::LaserScan msg;
    msg.header.frame_id = "leddar";
    msg.header.stamp = ros::Time::now();

    // Set up field of view.
    msg.angle_min = angles::from_degrees(-field_of_view / 2.0);
    msg.angle_max = angles::from_degrees(field_of_view / 2.0);
    msg.angle_increment = angles::from_degrees(field_of_view / BEAM_COUNT);
    msg.range_min = 0.0;
    msg.range_max = max_range;

    
    std::vector<std::string> leddar_records = split_str(ss.str(), '|');
    for (std::vector<std::string>::iterator it = leddar_records.begin() ; it != leddar_records.end(); ++it) {
      std::cout << ' ' << *it;
    }




    // Push detections into message.
    for (int i = 0; i < BEAM_COUNT; i++) {
      // msg.ranges.push_back(detections[i].mDistance);
      msg.ranges.push_back(i);
    }


    // msg.data = ss.str();
    // ROS_INFO("%s", msg.data.c_str());








    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    leddar_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
