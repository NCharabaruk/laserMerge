//LaserMerge.cpp
//This node is used to merge the scans from two laser range finders

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include <vector>

using namespace std;

class mergeScans
{
public:
	mergeScans()
	{
		pub = n.advertise<sensor_msgs::LaserScan>("scan", 100);
		laser1 = n.subscribe("scan1", 10, &mergeScans::callback1, this);
		laser2 = n.subscribe("scan2", 10, &mergeScans::callback2, this);
	}

	void callback1(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		sensor_msgs::LaserScan newScan;

		int numPoints = (msg->angle_max - msg->angle_min) / msg->angle_increment;
		vector<float> newDist; //new distances
		vector<float> dist; //old distances
		vector<float> theta_p; //new angles
		vector<float> theta; //old angles

		//populate arrays to hold the original angles and their assiciated distances
		for (int iii = 0; iii < numPoints; iii++)
		{
			theta.push_back(((msg->angle_max - msg->angle_max) / msg->angle_increment) * iii + msg->angle_min);
			dist.push_back(msg->ranges[iii]);
		}

		//calculate new angles and distances
		for (int jjj = 0; jjj < numPoints; jjj++)
		{
			if (theta[jjj] < -M_PI / 2)
			{
				float theta_n = M_PI + theta[jjj];
				float x = dist[jjj] * sin(theta_n);
				float y = dist[jjj] * cos(theta_n);
				float dy = y - 0.333;

				if (dy < 0)
				{
					dy = dy * -1;
					theta_p.push_back(atan(x / dy));
				}
				else
				{
					theta_p.push_back(atan(x / dy) - M_PI);
				}

				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
			else if (theta[jjj] > M_PI / 2)
			{
				float theta_n = M_PI - theta[jjj];
				float x = dist[jjj] * sin(theta_n);
				float y = dist[jjj] * cos(theta_n);
				float dy = y - 0.333;
	
				if (dy < 0)
				{
					dy = dy * -1;
					theta_p.push_back(atan(x / dy));
				}
				else
				{
					theta_p.push_back(M_PI - atan(x / dy));
				}

				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
			else
			{
				float x = dist[jjj] * sin(theta[jjj]);
				float y = dist[jjj] * cos(theta[jjj]);
				float dy = y + 0.333;
				theta_p.push_back(atan(x / dy));
				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
		}

		float newAngleIncr = (theta_p.end() - theta_p.begin()) / theta_p.size();

		newScan.header.stamp = msg->header.stamp;
		newScan.header.frame_id = "virtual_laser";
		newScan.angle_min = theta_p[0];
		newScan.angle_max = theta_p[numPoints-1];
		newScan.angle_increment = newAngleIncr;
		newScan.time_increment = msg->time_increment;
		newScan.scan_time = msg->scan_time;
		newScan.range_min = msg->range_min;
		newScan.range_max = msg->range_max;
		newScan.intensities = msg->intensities;

		for (int kkk = 0; kkk < newDist.size(); kkk++)
		{
			newScan.ranges[kkk] = newDist[kkk];
		}
	
		pub.publish(newScan);
	}

	void callback2(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		sensor_msgs::LaserScan newScan;

		int numPoints = sizeof(msg->ranges) / sizeof(msg->ranges[0]); //number of elements in the ranges array
		vector<float> newDist; //new distances
		vector<float> dist; //old distances
		vector<float> theta_p; //new angles
		vector<float> theta; //old angles

		//populate arrays to hold the original angles and their assiciated distances
		for (int iii = 0; iii < numPoints; iii++)
		{
			theta.push_back(((msg->angle_max - msg->angle_max) / msg->angle_increment) * iii + msg->angle_min);
			dist.push_back(msg->ranges[iii]);
		}

		//calculate new angles and distances
		for (int jjj = 0; jjj < numPoints; jjj++)
		{
			if (theta[jjj] < -M_PI / 2)
			{
				float theta_n = M_PI + theta[jjj];
				float x = dist[jjj] * sin(theta_n);
				float y = dist[jjj] * cos(theta_n);
				float dy = y - 0.333;

				if (dy < 0)
				{
					dy = dy * -1;
					theta_p.push_back(atan(x / dy));
				}
				else
				{
					theta_p.push_back(atan(x / dy) - M_PI);
				}

				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
			else if (theta[jjj] > M_PI / 2)
			{
				float theta_n = M_PI - theta[jjj];
				float x = dist[jjj] * sin(theta_n);
				float y = dist[jjj] * cos(theta_n);
				float dy = y - 0.333;
	
				if (dy < 0)
				{
					dy = dy * -1;
					theta_p.push_back(atan(x / dy));
				}
				else
				{
					theta_p.push_back(M_PI - atan(x / dy));
				}

				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
			else
			{
				float x = dist[jjj] * sin(theta[jjj]);
				float y = dist[jjj] * cos(theta[jjj]);
				float dy = y + 0.333;
				theta_p.push_back(atan(x / dy));
				newDist.push_back(sqrt((x * x) + (dy * dy)));
			}
		}

		float newAngleIncr = (theta_p.end() - theta_p.begin()) / theta_p.size();

		newScan.header.stamp = msg->header.stamp;
		newScan.header.frame_id = "virtual_laser";
		newScan.angle_min = theta_p[0];
		newScan.angle_max = theta_p[numPoints-1];
		newScan.angle_increment = newAngleIncr;
		newScan.time_increment = msg->time_increment;
		newScan.scan_time = msg->scan_time;
		newScan.range_min = msg->range_min;
		newScan.range_max = msg->range_max;
		newScan.intensities = msg->intensities;

		for (int kkk = 0; kkk < numPoints; kkk++)
		{
			newScan.ranges[kkk] = newDist[kkk];
		}
	
		pub.publish(newScan);
	}
		
private:
	ros::NodeHandle n;
	ros::Publisher pub;
	ros::Subscriber laser1;
	ros::Subscriber laser2;
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "laserMerge");
	mergeScans MGObject;
	ros::spin();
	return 0;
}


