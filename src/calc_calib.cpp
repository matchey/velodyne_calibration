/*
 * src: icp_sample.cpp
 *
 * last_update: '16.08.02
 * memo:
 *		registrationを試すコード
 *
 *	[icp]と[gicp]はsrcとtgtが逆. 本来ならば，srcをtgtに合わせる.
 */

#include <iostream>
#include <stdio.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>
//#include "fast_pcl/resistration/ndt.h"
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/common/transforms.h> 
#include <boost/thread/thread.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point32.h>
#include <Eigen/Core>
#include <tf/transform_broadcaster.h>
#include <pcl/io/ply_io.h>
#include <time.h>
// #include <chrono>


// #define N 313350

using namespace std;
using namespace Eigen;
//using pcl::visualization

const double prm_1 = 1.0; //0.5//0.3
// const double prm_1 = 1.0; //0.5//0.3
const double prm_2 = 20;
const double prm_3 = 1e-8;//1e-5

// static std::chrono::time_point<std::chrono::system_clock> matching_start, matching_end;

inline double SIGN(double x) {return (x >= 0.0) ? +1.0 : -1.0;}
inline double NORM(double a, double b, double c, double d) {return sqrt(a * a + b * b + c * c + d * d);}
Eigen::Vector4f mat2quat(Eigen::Matrix3f mat){/*{{{*/
	Eigen::Vector4f q;
	q(0) = ( mat(0,0) + mat(1,1) + mat(2,2) + 1.0) / 4.0;
	q(1) = ( mat(0,0) - mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(2) = (-mat(0,0) + mat(1,1) - mat(2,2) + 1.0) / 4.0;
	q(3) = (-mat(0,0) - mat(1,1) + mat(2,2) + 1.0) / 4.0;
	if(q(0) < 0.0) q(0) = 0.0;
	if(q(1) < 0.0) q(1) = 0.0;
	if(q(2) < 0.0) q(2) = 0.0;
	if(q(3) < 0.0) q(3) = 0.0;
	q(0) = sqrt(q(0));
	q(1) = sqrt(q(1));
	q(2) = sqrt(q(2));
	q(3) = sqrt(q(3));
	if(q(0) >= q(1) && q(0) >= q(2) && q(0) >= q(3)) {
		q(0) *= +1.0;
		q(1) *= SIGN(mat(2,1) - mat(1,2));
		q(2) *= SIGN(mat(0,2) - mat(2,0));
		q(3) *= SIGN(mat(1,0) - mat(0,1));
	} else if(q(1) >= q(0) && q(1) >= q(2) && q(1) >= q(3)) {
		q(0) *= SIGN(mat(2,1) - mat(1,2));
		q(1) *= +1.0;
		q(2) *= SIGN(mat(1,0) + mat(0,1));
		q(3) *= SIGN(mat(0,2) + mat(2,0));
	} else if(q(2) >= q(0) && q(2) >= q(1) && q(2) >= q(3)) {
		q(0) *= SIGN(mat(0,2) - mat(2,0));
		q(1) *= SIGN(mat(1,0) + mat(0,1));
		q(2) *= +1.0;
		q(3) *= SIGN(mat(2,1) + mat(1,2));
	} else if(q(3) >= q(0) && q(3) >= q(1) && q(3) >= q(2)) {
		q(0) *= SIGN(mat(1,0) - mat(0,1));
		q(1) *= SIGN(mat(2,0) + mat(0,2));
		q(2) *= SIGN(mat(2,1) + mat(1,2));
		q(3) *= +1.0;
	} else {
		printf("coding error\n");
	}
	double r = NORM(q(0), q(1), q(2), q(3));
	q(0) /= r;
	q(1) /= r;
	q(2) /= r;
	q(3) /= r;

	return q;
}/*}}}*/

int main (int argc, char** argv)
{
	string file_src;
	// char file_src[100];
	string file_tgt;

	cout << "Note : you must be in directory that include .pcd file to be load." << endl;

	//
	// cout << "Input TargetPointCloud -> ";
	// cin >> file_tgt;
	// cout << "Input SourcePointCloud -> ";
	// cin >> file_src;
    //
	// file_src += ".pcd";
	// file_tgt += ".pcd";
    //
	int mode = 2;
	// cout << "[ICP]:1 [GICP]:2 [NDT]:3 --> ";
	// cin  >> mode;
    //
	// file_tgt = "cloud_0.pcd";
	file_tgt = "cloud_1.pcd";
	file_src = "aft.pcd";

	/*
	   ros::init(argc,argv,"REGISTRATION_SAMPLE");
	   ros::NodeHandle n;
	   ros::Rate roop(1);
	   */
	int cnt = 0;
	double tmp_x = 0.0;
	double tmp_y = 0.0;
	double tmp_z = 0.0;
	double roll = 0.0;
	double pitch = 0.0;
	double yaw = 0.0;
	double tmp_score = 0.0;
	double tmp_time = 0.0;

	double icp_score = 0.0;
	double last_icp_score = 0.0;

	while(cnt < 1){
		if(mode==1){
		}else if(mode == 2){
			cout << "GICP registration!!" << endl;
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_src (new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_tgt (new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr cloud_final (new pcl::PointCloud<pcl::PointNormal>);
			pcl::PointCloud<pcl::PointNormal>::Ptr filtered_cloud (new pcl::PointCloud<pcl::PointNormal>);

			pcl::io::loadPCDFile<pcl::PointNormal> ( file_src, *cloud_src); // GICPはtgtをsrcに合わせる,(本来はsrcをtgt)
			pcl::io::loadPCDFile<pcl::PointNormal> ( file_tgt, *cloud_tgt);


			//	matching_start = std::chrono::system_clock::now();
			/*		
					pcl::ApproximateVoxelGrid<pcl::PointNormal> approximate_voxel_filter;
					approximate_voxel_filter.setLeafSize (0.2, 0.2, 0.2); //0.2
					approximate_voxel_filter.setInputCloud (cloud_src);
					approximate_voxel_filter.filter (*filtered_cloud);
					std::cout << "Filtered cloud contains " << filtered_cloud->size ()
					<< " data points from room_scan2.pcd" << std::endl;
					*/
			pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp;

			//GICP (Set Parameter)
			gicp.setMaximumIterations(prm_2);		  // GICPの最大繰り返し回数
			gicp.setTransformationEpsilon (prm_3);    // RANSAC除去距離 
			gicp.setEuclideanFitnessEpsilon(prm_3);   // 変換パラメータ値
			gicp.setMaxCorrespondenceDistance (prm_1);// 対応距離の最大値

			gicp.setInputSource(cloud_src);
			//gicp.setInputSource(filtered_cloud);	
			gicp.setInputTarget(cloud_tgt);

			// matching_start = std::chrono::system_clock::now();
			pcl::PointCloud<pcl::PointNormal> Final;
			gicp.align(Final);


			// double exe_time = 0.0;
			// matching_end = std::chrono::system_clock::now();
			// exe_time = std::chrono::duration_cast<std::chrono::milliseconds>(matching_end - matching_start).count() / 1000.0;
			// cout << "処理に" << exe_time << "[sec]かかりました" << endl;	
			// tmp_time += exe_time;

			Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
			transform = gicp.getFinalTransformation();
			cout << "Transformation\n" << transform << endl;
			pcl::transformPointCloud ( *cloud_src, *cloud_final, transform);

			tmp_x += transform(0,3);
			tmp_y += transform(1,3);
			tmp_z += transform(2,3);
			// roll = 
			Eigen::Matrix3f mat;
			Eigen::Vector4f quat;
			// mat << sum(0,0), sum(0,1), sum(0,2), sum(1,0), sum(1,1), sum(1,2), sum(2,0), sum(2,1), sum(2,2);
			mat << transform(0,0), transform(0,1), transform(0,2), transform(1,0), transform(1,1), transform(1,2), transform(2,0), transform(2,1), transform(2,2);
			quat = mat2quat(mat);

			tf::Quaternion q(quat(1), quat(2), quat(3), quat(0));
			tf::Matrix3x3 m(q);
			m.getRPY(roll, pitch, yaw);

			double score = gicp.getFitnessScore();
			cout << "GICP_SCORE: " << score << endl;
			tmp_score += score;
			//cloud_final = (Final + *cloud_src).makeShared();
			//Eigen::Matrix4f rsl = gicp.getFinalTransformation();

			//pcl::io::savePCDFileBinary("final.pcd", Final);
			// pcl::io::savePCDFileBinary("gicp_final.pcd", *cloud_final);

		}else if(mode==3){
		}else{
			cout<< "mode setting error!!" << endl;
		}
		cnt++;
		// if( icp_score < 10e-05 || abs(icp_score - last_icp_score) < 10e-07 ){
		// 	break;
		// }
	}
	cout <<"\nFinish!!!!!!!!!!!!\n"<<endl;
	// cout << "x_value = " << tmp_x/10.0 << endl << "y_value = " << tmp_y/10.0 << endl
	// 	<< "z_value = " << tmp_z/10.0 << endl << "score = " << tmp_score/10.0 << endl
	// 	<< "ave_time = " << tmp_time/10.0 << endl;
	cout << "[m]\n" << "x_value = " << tmp_x << endl << "y_value = " << tmp_y << endl
		<< "z_value = " << tmp_z << endl << endl << "[deg]" <<endl;;
	cout << "roll = " << roll*180/M_PI << endl << "pitch = " << pitch*180/M_PI << endl
		<< "yaw = " << yaw*180/M_PI << endl;

	return (0);
}

