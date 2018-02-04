
#include <ros/ros.h>
#include "velodyne_calibration/tf_map2vicon.h"

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_tf_map2vicon");

	cout << "Note : you must be in directory that include .pcd file to be load." << endl;
	TFmap2vicon mv;

	ros::spin();

	return 0;
}

