//LaserMerge.cpp
//This node is used to merge the scans from two laser range finders

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#define _USE_MATH_DEFINES
#include "math.h"

class mergeScans
{
public:
	mergeScans()
	{
		pub = n.advertise<sensor_msgs::LaserScan>("scan", 100);
		laser1 = n.subscribe("scan1", 10, &mergeScans::callback, this);
		laser2 = n.subscribe("scan2", 10, &mergeScans::callback, this);
	}

	void callback(const sensor_msgs::LaserScan::ConstPtr& msg)
	{
		sensor_msgs::LaserScan newScan;

		int numPoints = (msg->angle_max - msg->angle_min) / msg->angle_increment;
		float newDist[540] = { 0 }; //540 = (max_angle(2.356 rad) - min_angle(-2.356 rad))/angle_increment(8.726e-3 rad), new distances
		float dist[540] = { 0 }; //old distances
		double theta_p[540] = { 0 }; //new angles
		double theta[540] = { 0 }; //old angles
		bool dir = 1;

		if (msg->header.frame_id == "front_laser")
		{
			dir = 1;
		}
		else if (msg->header.frame_id == "rear_laser")
		{
			dir = 0;
		}
		else
		{
			ROS_WARN("Unknown frame_id, assuming laser is forward facing");
		}

		//populate arrays to hold the original angles and their assiciated distances
		for (int iii = 0; iii < numPoints; iii++)
		{
			theta[iii] = ((msg->angle_max - msg->angle_max) / msg->angle_increment) * iii + msg->angle_min;
			dist[iii] = msg->ranges[iii];
		}

		//calculate new angles and distances
		for (int jjj = 0; jjj < numPoints; jjj++)
		{
			if (theta[jjj] < -M_PI / 2)
			{
				double theta_n = M_PI + theta[jjj];
				double x = dist[jjj] * sin(theta_n);
				double y = dist[jjj] * cos(theta_n);
				double dy = y - 0.333;

				if (dy < 0)
				{
					dy = dy * -1;
					theta_p[jjj] = atan(x / dy);
				}
				else
				{
					theta_p[jjj] = atan(x / dy) - M_PI;
				}

				newDist[jjj] = sqrt((x * x) + (dy * dy));
			}
			else if (theta[jjj] > M_PI / 2)
			{
				double theta_n = M_PI - theta[jjj];
				double x = dist[jjj] * sin(theta_n);
				double y = dist[jjj] * cos(theta_n);
				double dy = y - 0.333;
	
				if (dy < 0)
				{
					dy = dy * -1;
					theta_p[jjj] = atan(x / dy);
				}
				else
				{
					theta_p[jjj] = M_PI - atan(x / dy);
				}

				newDist[jjj] = sqrt((x * x) + (dy * dy));
			}
			else
			{
				double x = dist[jjj] * sin(theta[jjj]);
				double y = dist[jjj] * cos(theta[jjj]);
				double dy = y + 0.333;
				theta_p[jjj] = atan(x / dy);
				newDist[jjj] = sqrt((x * x) + (dy * dy));
			}

			if (!dir)
			{
				if (theta_p[jjj] <= 0)
				{
					theta_p[jjj] += M_PI;
				}
				else
				{
					theta_p[jjj] -= M_PI;
				}
			}
		}

		double newAngleIncr = (theta[numPoints - 1] - theta[0]) / numPoints;

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


