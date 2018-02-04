
#ifndef TF_MAP2VICON_H
#define TF_MAP2VICON_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>

class TFmap2vicon
{
	ros::NodeHandle n;
	ros::Subscriber sub_odom;
	ros::Subscriber sub_vicon;
	ros::Subscriber sub_pc;
	// ros::Subscriber sub_flag;
	ros::Publisher pub_pc;

	tf::TransformBroadcaster odom_broadcaster;

	XmlRpc::XmlRpcValue init_pose;
	std::string topic_odom;
	std::string topic_vicon;
	std::string topic_pc;
	// std::string topic_flag;
	std::string map_name;
	XmlRpc::XmlRpcValue params_gisp;

	geometry_msgs::Pose2D init;
	pcl::PointCloud<pcl::PointNormal>::Ptr map;
	geometry_msgs::Pose pose;
	std::vector<geometry_msgs::Pose> pose_map;
	std::vector<geometry_msgs::Pose> pose_vicon;
	pcl::PointCloud<pcl::PointNormal>::Ptr pc;
	// bool is_save;
	Eigen::Affine3d odom_correction;
	double yaw_correction;

	void getPoseICP();
	void saveYaml();
	void pub_tf();

	public:
	TFmap2vicon();
	void odomCallback(const nav_msgs::Odometry::ConstPtr&);
	void viconCallback(const geometry_msgs::Pose::ConstPtr&);
	void pcCallback(const sensor_msgs::PointCloud2ConstPtr&);
	// void isCallback(const std_msgs::BoolConstPtr&);
};

#endif

