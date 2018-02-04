
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/gicp.h>
#include <Eigen/Geometry>
#include "mmath/mmath.h"
#include "mmath/binarion.h"
#include "velodyne_calibration/tf_map2vicon.h"

using namespace std;

TFmap2vicon::TFmap2vicon()
	: map(new pcl::PointCloud<pcl::PointNormal>), pc(new pcl::PointCloud<pcl::PointNormal>),
	  odom_correction(Eigen::Affine3d::Identity()), yaw_correction(0.0)
{
	n.getParam("/init/pose", init_pose);
	n.param<string>("/topic_name/odom_complement", topic_odom, "/odom/complement");
	n.param<string>("/topic_name/odom_vicon", topic_vicon, "/odom/vicon");
	n.param<string>("/topic_name/pointclouds", topic_pc, "/velodyne_points");
	// n.param<string>("/topic_name/pointclouds", topic_flag, "/flag/moment");
	n.param<string>("/map/load", map_name, "map_0.pcd");

	cout << "subscribe topic name odom : " << topic_odom << endl;
	// cout << "init pose : " << init_pose << endl;

	init.x = init_pose["x"];
	init.y = init_pose["y"];
	init.theta = init_pose["yaw"];

	cout << "init pose (x, y, yaw) : ("
		 << init.x << ", " << init.y << ", " << init.theta << ")" << endl;

	pcl::io::loadPCDFile<pcl::PointNormal>(map_name, *map);

	sub_odom = n.subscribe<nav_msgs::Odometry>(topic_odom, 1, &TFmap2vicon::odomCallback, this);
	sub_vicon = n.subscribe<geometry_msgs::Pose>(topic_vicon, 1, &TFmap2vicon::viconCallback, this);
	sub_pc = n.subscribe<sensor_msgs::PointCloud2>(topic_pc, 1, &TFmap2vicon::pcCallback, this);
	// sub_flag = n.subscribe<std_msgs::Bool>(topic_flag, 1, &TFmap2vicon::isCallback, this);
	pub_pc = n.advertise<sensor_msgs::PointCloud2>("/velodyne/onMap", 1);
}

// void TFmap2vicon::isCallback(const std_msgs::BoolConstPtr& msg)
// {
// 	is_save = msg->data;
// }

void TFmap2vicon::viconCallback(const geometry_msgs::Pose::ConstPtr& msg)
{
	getPoseICP();

	pose_vicon.push_back(*msg);
}

void TFmap2vicon::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
	double x = msg->pose.pose.position.x;
	double y = msg->pose.pose.position.y;

	Eigen::Quaterniond q_pose(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
					 	 	  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
	Eigen::Quaterniond q_init(Eigen::AngleAxisd(init.theta, Eigen::Vector3d::UnitZ()));

	q_pose = q_init * odom_correction.rotation() * q_pose;

	Eigen::Vector3d v(1, 1, 1);
	v(0) = x*cos(init.theta) - y*sin(init.theta) + init.x;
	v(1) = x*sin(init.theta) + y*cos(init.theta) + init.y;

	v = odom_correction * v;

	pose.position.x = v(0);
	pose.position.y = v(1);
	pose.orientation.x = q_pose.x();
	pose.orientation.y = q_pose.y();
	pose.orientation.z = q_pose.z();
	pose.orientation.w = q_pose.w();
	
	pub_tf();

	if(pose_map.size() < pose_vicon.size()){
		pose_map.push_back(pose);

		if(pose_vicon.size() == 2){
			saveYaml();
		}
	}
}

void TFmap2vicon::pcCallback(const sensor_msgs::PointCloud2ConstPtr& input)
{
	pcl::fromROSMsg(*input, *pc);
}

////////////////////// private //////////////////////

void TFmap2vicon::getPoseICP()
{
	pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
	
	Eigen::Translation<double, 3> trans = Eigen::Translation<double, 3>(pose.position.x,
										   							    pose.position.y,
										   								pose.position.z);

	Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
					 	 pose.orientation.y, pose.orientation.z);

	Eigen::Affine3d transform = trans * q;

	pcl::transformPointCloud(*pc, *cloud, transform);

	pcl::GeneralizedIterativeClosestPoint<pcl::PointNormal, pcl::PointNormal> gicp;
	//GICP (Set Parameter)
	gicp.setMaximumIterations(100);		  // GICPの最大繰り返し回数
	gicp.setTransformationEpsilon(1e-8);    // RANSAC除去距離 
	gicp.setEuclideanFitnessEpsilon(1e-8);   // 変換パラメータ値
	gicp.setMaxCorrespondenceDistance(2.0);// 対応距離の最大値
	// maximum allowable difference between two consecutive rotations
	gicp.setRotationEpsilon(0.4);
	// the number of neighbors used when selecting a point neighbourhood to compute covariances
	// setCorrespondenceRandomness (int k);
	// set maximum number of iterations at the optimization step
	// setMaximumOptimizerIterations(int max);

	gicp.setInputSource(cloud);
	gicp.setInputTarget(map);

	cout << "calculating..." << endl;
	pcl::PointCloud<pcl::PointNormal> Final;
	gicp.align(Final);
	// gicp.align(); // 計算だけする関数ないの?

	Eigen::Matrix4d mtrans = gicp.getFinalTransformation().cast<double>(); //return float
	Eigen::Affine3d transformation;
	transformation.matrix() = mtrans;

	cout << "--" << endl;
	cout << "gicp :\n" << transformation.matrix() << endl;
	cout << "--" << endl;
	
	odom_correction = transformation * odom_correction;
	// odom_correction = odom_correction * transformation;
	
    //
	// // Eigen::Vector3d euler = transformation.rotation().eulerAngles(1, 2, 0);
	// Eigen::Vector3d euler = odom_correction.rotation().eulerAngles(0,1,2);
	//
	// cout << "yaw_correction : " << yaw_correction*180/M_PI << endl;
	// cout << "euler :\n" << euler << endl;
	// // yaw_correction += euler(0);
	// // yaw_correction = euler(0);
	// yaw_correction = euler(2);
    //
	// cout << "yaw_correction : " << yaw_correction*180/M_PI << endl;
}

void TFmap2vicon::saveYaml()
{
	// cout << "vicon[0] : (" << pose_vicon[0].position.x << ", " << pose_vicon[0].position.y << ")" << endl;
	// cout << "map[0] : (" << pose_map[0].position.x << ", " << pose_map[0].position.y << ")" << endl;
	// cout << "vicon[1] : (" << pose_vicon[1].position.x << ", " << pose_vicon[1].position.y << ")" << endl;
	// cout << "map[1] : (" << pose_map[1].position.x << ", " << pose_map[1].position.y << ")" << endl;

	// double diff_x = pose_vicon[0].position.x - pose_map[0].position.x;
	// double diff_y = pose_vicon[0].position.y - pose_map[0].position.y;
	double diff_x = pose_map[0].position.x - pose_vicon[0].position.x;
	double diff_y = pose_map[0].position.y - pose_vicon[0].position.y;
	double theta_map = atan2(pose_map[1].position.y - pose_map[0].position.y,
	                         pose_map[1].position.x - pose_map[0].position.x);
	double theta_vicon = atan2(pose_vicon[1].position.y - pose_vicon[0].position.y,
	                           pose_vicon[1].position.x - pose_vicon[0].position.x);
	double diff_yaw = Binarion::deviation(theta_vicon, theta_map);

	std::ofstream ofs("/home/amsl/TFmap2vicon.yaml"); //launchだと~/.rosがカレントパス
	if(ofs){
		ofs << "TFmap2vicon:" << endl;
		ofs << "    parent_frame: /map" << endl;
		ofs << "    child_frame: /world" << endl;
		ofs << "    6DoF: [";
		ofs << fixed;
		ofs << diff_x << ", ";
		ofs << diff_y << ", 0.0, 0.0, 0.0, ";
		ofs << diff_yaw << "] # x, y, z, roll, pitch, yaw";
	}else{
		cerr << "ofs ERROR" << endl;
	}
	ofs.close();

	double yaw = tf::getYaw(pose.orientation);

	std::ofstream ofstr("/home/amsl/lastpose.txt"); //launchだと~/.rosがカレントパス
	if(ofstr){
		ofstr << "= - (";
		ofstr << fixed;
		ofstr << yaw << " - ";
		ofstr << diff_yaw << ")";
	}else{
		cerr << "ofstr ERROR" << endl;
	}
	ofstr.close();

	// static_transform_publisher x y z qx qy qz qw frame_id child_frame_id  period_in_ms
	// static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period_in_ms
	// std::ofstream ofs_args("/home/amsl/TFmap2vicon.args"); //launchだと~/.rosがカレントパス
	// if(ofs_args){
	// 	ofs_args << "args=\"";
	// 	ofs_args << fixed;
	// 	ofs_args << diff_x << " ";
	// 	ofs_args << diff_y << " 0.0 ";
	// 	ofs_args << diff_yaw << " 0.0 0.0 /world /map 100\" />";
	// }else{
	// 	cerr << "ofs_args ERROR" << endl;
	// }

	// ofs_args.close();
}

void TFmap2vicon::pub_tf()
{
    // //first, we'll publish the transform over tf
    // geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = ros::Time::now();
    // odom_trans.header.frame_id = "/map";
    // odom_trans.child_frame_id = "/matching_base_link";
    //
	// mmath::pointSubstitution(odom_trans.transform.translation, pose.position);
	// mmath::quatSubstitution(odom_trans.transform.rotation, pose.orientation);
    // // odom_trans.transform.translation.x = x;
    // // odom_trans.transform.translation.y = y;
    // // odom_trans.transform.translation.z = 0.0;
    // // odom_trans.transform.rotation = odom_quat;
    //
    // //send the transform
    // odom_broadcaster.sendTransform(odom_trans);

	pcl::PointCloud<pcl::PointNormal>::Ptr cloud (new pcl::PointCloud<pcl::PointNormal>);
	Eigen::Translation<double, 3> trans = Eigen::Translation<double, 3>(pose.position.x,
										   							    pose.position.y,
										   								pose.position.z);
	Eigen::Quaterniond q(pose.orientation.w, pose.orientation.x,
					 	 pose.orientation.y, pose.orientation.z);
	Eigen::Affine3d transform = trans * q;
	pcl::transformPointCloud(*pc, *cloud, transform);
	sensor_msgs::PointCloud2 pc2;
	// cloud->header.frame_id = "/map";
	// cloud->header.stamp = ros::Time::now();
	pcl::toROSMsg(*cloud, pc2);
	pc2.header.frame_id = "/map";
	pc2.header.stamp = ros::Time::now();
	pub_pc.publish(pc2);
}

