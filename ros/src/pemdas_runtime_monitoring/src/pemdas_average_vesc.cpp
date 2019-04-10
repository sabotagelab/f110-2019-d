#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <vesc_msgs/VescStateStamped.h>
#include <queue>
#include <string>


std::queue<float> historicalSpeed;
float avg = 0;

void nextAverage(const vesc_msgs::VescStateStamped::ConstPtr& vesc)
{
    if (historicalSpeed.size() < 10) {
        ROS_INFO("Not enough historical data for 10-sample average...");
    } else {
	float old = historicalSpeed.front();
        historicalSpeed.pop();
	avg -= old;
    }

    float newSpdAvg = (float)vesc->state.speed / 10.0f;
    historicalSpeed.push(newSpdAvg);
    avg += newSpdAvg; 
    
    ROS_INFO("New average speed calculated: Avg=%f", avg);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pemdas_vesc_average");
    ros::NodeHandle n;

    ros::Publisher pub = n.advertise<std_msgs::Float64>("average_velocity", 100);
    ros::Rate loop_rate(5);
   

    ros::Subscriber sub = n.subscribe("sensors/core", 10, nextAverage);

    while (ros::ok())
    {
        if (historicalSpeed.size() >= 10) {
	    std_msgs::Float64 msg;
	    msg.data = avg;
            pub.publish(msg);
        }

        loop_rate.sleep();
	    ros::spinOnce();
    }

    return 0;
}



