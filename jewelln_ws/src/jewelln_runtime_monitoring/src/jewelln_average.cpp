#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "std_msgs/Float32.h"
#include <queue>
#include <string>


std::queue<float> historicalVelocity;
float avg = 0;

void nextAverage(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    if (historicalVelocity.size() < 10) {
        ROS_INFO("Not enough historical data for 10-sample average...");
    } else {
	float old = historicalVelocity.front();
        historicalVelocity.pop();
	avg -= old;
    }

    float newVelAvg = (float)cmd_vel->linear.x / 10.0f;
    historicalVelocity.push(newVelAvg);
    avg += newVelAvg; 
    
    ROS_INFO("New average velocity calculated: Avg=%f", avg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "jewelln_average");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float32>("average_velocity", 1000);
    ros::Rate loop_rate(5);
   

    ros::Subscriber sub = n.subscribe("turtle1/cmd_vel", 1000, nextAverage);

    while (ros::ok())
    {
        if (historicalVelocity.size() >= 10) {
	    std_msgs::Float32 msg;
	    msg.data = avg;
            pub.publish(msg);
        }

        loop_rate.sleep();
	    ros::spinOnce();
    }

    return 0;
}



