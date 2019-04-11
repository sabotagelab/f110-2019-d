
// %Tag(FULLTEXT)%
#include<iostream>
#include<sstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  { 
    //Topic you want to publish
    pub_ = n_.advertise<geometry_msgs::Point>("/gap_center", 10);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/scan", 10, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::LaserScan& input)
  {
    


    geometry_msgs::Point output;
    output.z = 0;
    pub_.publish(output);

    // ROS_INFO("I heard: [%f]", output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  double window[10];
  int iteration;
};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "pemdas_gap_finding");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
// %Tag(CALLBACK)%
// void chatterCallback(const geometry_msgs::Twist& msg){
//   // int data = msg.linear.x;
//   ROS_INFO("I heard: [%f]", msg.linear.x);


// }
// // %EndTag(CALLBACK)%

// int main(int argc, char **argv)
// {
//   *
//    * The ros::init() function needs to see argc and argv so that it can perform
//    * any ROS arguments and name remapping that were provided at the command line.
//    * For programmatic remappings you can use a different version of init() which takes
//    * remappings directly, but for most command-line programs, passing argc and argv is
//    * the easiest way to do it.  The third argument to init() is the name of the node.
//    *
//    * You must call one of the versions of ros::init() before using any other
//    * part of the ROS system.
   
//   ros::init(argc, argv, "average_vel_sub");
//   ros::init(argc, argv, "average_vel_pub");

//   ros::NodeHandle n;

// // %Tag(SUBSCRIBER)%
//   ros::Subscriber sub = n.subscribe("/turtle1/cmd_vel", 1000, chatterCallback);

//   ros::Publisher vel_pub = n.advertise<std_msgs::float32>("/average_velocity", 1000);
  

// // %EndTag(SUBSCRIBER)%

// // %Tag(SPIN)%
//   ros::spin();
// // %EndTag(SPIN)%

//   return 0;
// }
// %EndTag(FULLTEXT)%