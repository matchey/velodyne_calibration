//
// src : get_min_z.cpp
//
// last update: '17.04.11
// author: matchey
//
// memo:
// 	入力された(x, y)を中心とする半径x[m]の円内で最もz(高さ)が低いpointのzを返す
//
#include <ros/ros.h>
// #include <sensor_msgs/PointCloud2.h>
// #include <std_msgs/Bool.h>
// #include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
// #include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
// #include <sstream>
#include <string>
#include <vector>

using namespace std;

class GndGetter
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
	double x;
	double y;
	double z;
	vector<int> pointIdxRadiusSearch;

	public:
	GndGetter()
		: cloud(new pcl::PointCloud<pcl::PointXYZ>)
	{
		set_cloud();
	}

	void set_cloud(string filename = "cloud_0.pcd" )
	{
		if (pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *cloud) == -1){
			cout << "pcd file not found." << endl;
		}
	}

	void input_xy()
	{
		double _x = 0.0;
		double _y = 0.0;
		cout<<"set (x, y) : ";
		scanf("%lf,%lf",&_x,&_y);

		set_xy(_x, _y);

		cout<<"("<<x<<", "<<y<<")"<<endl;
	}

	void set_xy(double _x, double _y){x = _x; y = _y;}

	bool get_min_z()
	{
		if(pointIdxRadiusSearch.size()){
			double min_z = cloud->points[ pointIdxRadiusSearch[0] ].z;
			for (size_t i = 1; i < pointIdxRadiusSearch.size (); ++i){
				if(cloud->points[ pointIdxRadiusSearch[i] ].z < min_z){
					min_z = cloud->points[ pointIdxRadiusSearch[i] ].z;
				}
			}
			z = min_z;
		}else{
			z = 0.0;
			cout<<"========no points========"<<endl;
			return false;
		}
		pointIdxRadiusSearch.clear();
		return true;
	}

	void radius_neighbors(double radius = 3.0)
	{
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

		kdtree.setInputCloud (cloud);

		pcl::PointXYZ searchPoint;

		searchPoint.x = x;
		searchPoint.y = y;
		searchPoint.z = -1.3;

		vector<float> pointRadiusSquaredDistance;

		if ( kdtree.radiusSearch (searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
		{
			// for (size_t i = 0; i < pointIdxRadiusSearch.size (); ++i)
			// 	std::cout << "    "  <<   cloud->points[ pointIdxRadiusSearch[i] ].x 
			// 		<< " " << cloud->points[ pointIdxRadiusSearch[i] ].y 
			// 		<< " " << cloud->points[ pointIdxRadiusSearch[i] ].z 
			// 		<< " (squared distance: " << pointRadiusSquaredDistance[i] << ")" << std::endl;
		}
	}

	void spin()
	{
		bool flag = true;
		while(flag){
			input_xy();
			if(x == 99999){
				break;
			}else{
				radius_neighbors(0.3);
				get_min_z();
				cout<<"min z : "<<z<<endl;
			}
		}
	}

	void plot4csv(double radius = 2.0)
	{
		ofstream ofs("ground1.csv");
		double x1 = 0.0; //start
		double y1 = 0.0;
		double x2 = 0.0; //goal
		double y2 = 0.0;
		double _x = 0.0; //plot
		double _y = 0.0;

		double slope = 0.0;
		double dx = 1.0;

		cout<<"set start (x,y): ";
		scanf("%lf,%lf",&x1,&y1);
		
		cout<<"set  end  (x,y): ";
		scanf("%lf,%lf",&x2,&y2);

		if(x1 > x2){
			double tmp_x = x1;
			double tmp_y = y1;
			x1 = x2;
			y1 = y2;
			x2 = tmp_x;
			y2 = tmp_y;
		}
		
		if(fabs(x1-x2)<10e-6){
			x2 = x1 + 10e-5;
		}
		
		dx = radius * cos( atan2(y2-y1, x2-x1) );
		slope = (y2-y1)/(x2-x1);

		_x = x1;
		_y = y1;

		while( !(_x > x2) ){
			set_xy(_x, _y);
			radius_neighbors(radius);
			if(get_min_z()){
				ofs << x << "," << z << endl;
			}
			_x += dx;
			_y = slope * (_x - x2) + y2;
		}

		ofs.close();
	}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "get_min_z");
	GndGetter gg;
	
	// gg.spin();
	gg.plot4csv();

	return 0;
}

