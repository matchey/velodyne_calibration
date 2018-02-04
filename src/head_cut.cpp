/*
 * src: pass_through_filter.cpp
 * 
 * memo:
 *
*/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>//追加..rosのPointCloud, pclのPointCloud変換
#include <pcl_ros/point_cloud.h>			//　〃
#include <pcl/io/pcd_io.h>

const float z_min = -5.0;
const float z_max = 40.0;


using namespace std;


void passThroughFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &filtered_cloud)
{
	//-------------------------
	//--- PassThrogh Filter -------
	//-------------------------
	//pcl::PassThrough<pcl::PointXYZI> pass_x;
	//pass_x.setInputCloud(cloud);
	//pass_x.setFilterFieldName("x");
	//pass_x.setFilterLimits(-20.0, 20.0);
	//pass_x.setFilterLimitsNegative;　// xmin~xmaxを除く
	//pass_x.filter(*velo_input_passed_x);

	//pcl::PassThrough<pcl::PointXYZI> pass_y;
	//pass_y.setInputCloud(cloud);
	//pass_y.setFilterFieldName("y");
	//pass_y.setFilterLimits(3.0, 20.0);
	//pass_y.setFilterLimitsNegative;　// xmin~xmaxを除く
	//pass_y.filter(*velo_filtered);
	
	pcl::PassThrough<pcl::PointXYZ> pass_z;
	pass_z.setInputCloud(cloud);
	pass_z.setFilterFieldName("z");
	pass_z.setFilterLimits(z_min, z_max);
	//pass_z.setFilterLimitsNegative;　// xmin~xmaxを除く
	pass_z.filter(*filtered_cloud);
}


int main(int argc, char **argv)
{
	string fname_in, fname_out;
	cout << "Input file name(.pcd)  ==> ";
	cin  >> fname_in;
	fname_in  += ".pcd";
	cout << "Output file name(.pcd) ==> ";
	cin  >> fname_out;
	fname_out += ".pcd";


	ros::init(argc, argv, "pass_through_filter");
	ros::NodeHandle n;

	//ros::Subscriber velo_pnt_sub = n.subscribe("/velodyne_points", 1, veloPointsCallback);
	//ros::Rate r(20);
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud 		   (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	if(pcl::io::loadPCDFile<pcl::PointXYZ>(fname_in, *cloud) == -1){
		cout << "load error !!\n";
		exit(1);
	}
	
	passThroughFilter(cloud, filtered_cloud);

	pcl::io::savePCDFileBinary(fname_out, *filtered_cloud);
	cout << "Finish!!\n";

	return 0;
}
