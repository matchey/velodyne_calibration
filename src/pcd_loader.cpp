//pcd_loader.cpp

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#define MAX_NUM 100

unsigned int mode;

using namespace std;	


int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_loader");
  	ros::NodeHandle n;
	ros::Rate roop(1);

	string str;

	cout << "file name -> ";
	cin >> str;
	str = str + ".pcd";

	ros::Publisher pub  = n.advertise<sensor_msgs::PointCloud2>("/PointCloud_Surfel", 1);
	ros::Publisher pub2 = n.advertise<sensor_msgs::PointCloud2>("/PointCloud_RGB", 1);

	//pcl::PointCloud<pcl::PointXYZ> cloud;// (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZINormal> cloud_IN;// (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointNormal> cloud_IN;// (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointSurfel> cloud_IN;// (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_IN2;// (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB> cloud_rgb;// (new pcl::PointCloud<pcl::PointXYZ>);
	//pcl::PointCloud<pcl::PointXYZRGB> cloud_c;// (new pcl::PointCloud<pcl::PointXYZ>);

			
	//if( pcl::io::loadPCDFile<pcl::PointXYZINormal>(str, cloud_IN) == -1 ){
	if( pcl::io::loadPCDFile<pcl::PointSurfel>(str, cloud_IN) == -1 ){
	//if( pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(str, cloud_IN) == -1 ){
	//if( pcl::io::loadPCDFile<pcl::PointNormal>(str, cloud_IN) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}
	if( pcl::io::loadPCDFile<pcl::PointXYZRGBNormal>(str, cloud_IN2) == -1 ){
		cout << "load error !!\n";
		exit(1);
	}
	sensor_msgs::PointCloud2 pc, pc2;
	pcl::toROSMsg(cloud_IN, pc);
	pcl::toROSMsg(cloud_IN2, pc2);

	pc.header.frame_id  = "map";
	pc2.header.frame_id = "map";
	
	while(ros::ok()){
		
		pc.header.stamp  = ros::Time::now();
		pc2.header.stamp = ros::Time::now();
		pub.publish(pc);
		pub2.publish(pc2);
		
		roop.sleep();
		ros::spinOnce();
	}
	return (0);
}

