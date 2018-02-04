/*
 * src: pcd_mover_sample.cpp
 * 
 * lastupdate:'16.08.01
 * memo:
 *	pcdファイルを読み込み任意の値に並進回転移動させるsrc.
 *
*/

#include <stdio.h>
#include <iostream>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/SVD>


using namespace Eigen;
using namespace std;	


void cnv(pcl::PointCloud<pcl::PointNormal>::Ptr org, pcl::PointCloud<pcl::PointNormal>::Ptr rsl, 
		 double dx, double dy, double dz, double angle_x, double angle_y, double angle_z)
{
	cout << "normal conv" << endl;
	pcl::PointNormal p;

	Vector3d point_in, point_out, axis_x, axis_y ,axis_z;
	Quaterniond quat_X, quat_Y, quat_Z;
	axis_x << 1.0, 0.0, 0.0;
	axis_y << 0.0, 1.0, 0.0;
	axis_z << 0.0, 0.0, 1.0;

	quat_X = AngleAxisd(angle_x/180.0*M_PI, axis_x); // x軸を中心に回転するquat作成
	quat_Y = AngleAxisd(angle_y/180.0*M_PI, axis_y); // y軸
	quat_Z = AngleAxisd(angle_z/180.0*M_PI, axis_z); // z軸
	Quaterniond total_quat;
	total_quat = quat_Z * quat_Y * quat_X;
	
	size_t SIZE = org->points.size();
	rsl->points.resize(SIZE);

	for(size_t i=0; i<SIZE; i++){	
		point_in << org->points[i].x, org->points[i].y, org->points[i].z;
        
		//回転
		point_out = total_quat * point_in;
        
		//並進
		rsl->points[i].x = point_out(0) + dx;
		rsl->points[i].y = point_out(1) + dy;
		rsl->points[i].z = point_out(2) + dz;
		
		rsl->points[i].curvature = org->points[i].curvature;
	}
	cout<<"moved!!\n";
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "pcd_mover_sample");
  	ros::NodeHandle n;
	ros::Rate roop(1);

	
	double dx=0.0, dy=0.0, dz=0.0;
	cout << "dx[m] = "; cin >> dx;
	cout << "dy[m] = "; cin >> dy;
    cout << "dz[m] = "; cin >> dz;

	double angle_x=0.0, angle_y=0.0, angle_z=0.0;
	cout << "rotate_Z軸[deg] = "; cin >> angle_z;//[deg]
	cout << "rotate_Y軸[deg] = "; cin >> angle_y;//[deg]
    cout << "rotate_X軸[deg] = "; cin >> angle_x;//[deg]

	string pcd_filename;
	cout << "file name(.pcd) -> ";
	// cin >> pcd_filename;
	// pcd_filename = pcd_filename + ".pcd";
	pcd_filename = "cloud_0.pcd";
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_n (new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud_aft (new pcl::PointCloud<pcl::PointNormal>);

	unsigned short int type;
	
	type = 0;
	try{
		pcl::io::loadPCDFile<pcl::PointNormal>(pcd_filename, *cloud_n);
		cout << "loaded [PointNormal] file!!\n";
	}catch(std::exception e1){
		try{
			pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_filename, *cloud_rgb);
			type = 2;
			cout << "loaded [PointXYZRGB] file!!\n";
		}catch(std::exception e2){
			pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_filename, *cloud);
			type = 1;
			cout << "loaded [PointXYZ] file!!\n";
		}
	}

	cout << "type = " << type << endl;
	if(type == 0)
		cnv(cloud_n, cloud_aft, dx, dy, dz, angle_x, angle_y, angle_z);
	/*else if(type[i] == 1)
		cnv(cloud_, cloud_c[i]);
	else if(type[i] == 2)
		cnv(cloud_rgb, cloud_rgb[i]);
	*/else {
		return 1;
	}

	if(pcl::io::savePCDFileBinary("aft.pcd", *cloud_aft) == -1){
		PCL_ERROR("Cloudn't save pcd file!!\n");
		return(-1);
	}
	cout<<"save moved pcd file!!\n";

	return (0);
}

