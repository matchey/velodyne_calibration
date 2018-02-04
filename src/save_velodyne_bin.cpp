//
// src : save_velodyne_bin.cpp
//
// last update: '17.04.11
// author: matchey
//
// memo:
// 	pub_velodyne_moment.cppからタイミングを受け取り
// 	velodyneの点群を一周分保存する
// 	(velodyne取り付け位置のキャリブレーションに利用)
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sstream>
#include <string>

using namespace std;

class PointSaver
{
	ros::NodeHandle n;
	ros::Subscriber sub_flag;
	ros::Subscriber sub_pc;
		
	int counter;
	bool is_save;
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
	// pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud;

	public:
	PointSaver()
		: counter(0), is_save(false), cloud(new pcl::PointCloud<pcl::PointNormal>)// cloud(new pcl::PointCloud<pcl::PointXYZINormal>)
	{
		sub_flag = n.subscribe<std_msgs::Bool>("/flag/moment", 1, &PointSaver::isCallback, this);
		sub_pc = n.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1,
				&PointSaver::velCallback, this);
		// sub_pc = n.subscribe<sensor_msgs::PointCloud2>("/perfect_velodyne/normal", 1,
		// 		&PointSaver::velCallback, this);
	}

	void isCallback(const std_msgs::BoolConstPtr& msg){
		is_save = msg->data;
	}

	void velCallback(const sensor_msgs::PointCloud2ConstPtr& input) {
		if(is_save){
			pcl::fromROSMsg (*input, *cloud);

			string pcd_filename = "cloud_0.pcd";
			ostringstream ostr;
			ostr.str("");
			ostr << "cloud_" << counter << ".pcd";
			pcd_filename = ostr.str();

			pcl::io::savePCDFileBinary(pcd_filename, *cloud);

			cout<<"saved cloud_"<<counter<<".pcd"<<endl;

			counter++;
		}
		is_save = false;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "save_velodyne_bin");
	PointSaver ps;
	ros::spin();
	return 0;
}

