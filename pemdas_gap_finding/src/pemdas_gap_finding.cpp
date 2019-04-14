
// %Tag(FULLTEXT)%
#include<iostream>
#include<sstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point.h"
#include <array>
#include <vector>
#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  { 
    //Topic you want to publish
    pub_ = n_.advertise<std_msgs::Int32>("/features", 100);
    //Topic you want to subscribe
    sub_ = n_.subscribe("/scan", 100, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::LaserScan& scan)
	{
  
	this->scansize = scan.ranges.size();
	this->thres = 15000;
	// float input[scan.ranges.size()] = {scan.ranges};
	float input[scan.ranges.size()];
	for (int i = 0;i<this->scansize;i++) input[i] = scan.ranges[i];
	double dscan[this->scansize];
	double value;
	int pre_k = 1;
	std_msgs::Int32 k;
	std::vector<int> feature;	

	// checks if any scan ranges are infinite or NaN
	for(int i = 0; i < scansize; i++){
		if (std::isnan(input[i]) || std::isinf(input[i])){
			input[i] = scan.range_max + 1;
		}
	}

	// Take first gradient of laser range data (rate of change of information)
	dscan[0] =(input[0] - input[1])/scan.angle_increment;
	dscan[scansize-1] = (input[scansize-1] - input[scansize-2])/scan.angle_increment;
	for(int i = 1; i < scansize-1; i++){
		dscan[i] = (.5*(input[i-1] - input[i+1]))/scan.angle_increment;
	}

	// Take second gradient of laser range data (spikes indicate features)
	if (fabs(dscan[0] - dscan[1])/scan.angle_increment > this->thres) feature.push_back(0);
	for(int i = 1; i < scansize-1; i++){
		value = fabs(.5*dscan[i-1] - dscan[i+1])/scan.angle_increment;
		if (value > this->thres) feature.push_back(i);
		// ROS_INFO("I heard: [%f]", value);
	}
	if (fabs(dscan[0] - dscan[1])/scan.angle_increment > this->thres) feature.push_back(scansize-1);
	ROS_INFO("Features at:");
	for (int i = 0; i < feature.size(); i++){
		ROS_INFO("[%i]",feature[i]);
		if (feature[i+1]-feature[i] > 1) pre_k++;
	}
	k.data = pre_k;

    pub_.publish(k);

  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;
  int scansize;
  double thres; 
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

