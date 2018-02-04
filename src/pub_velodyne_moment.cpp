//
// src : pub_velodyne_moment.cpp
//
// last update: '17.04.11
// author: matchey
//
// memo:
// 	save_velodyne_bin.cppに
// 	velodyneの点群を保存するタイミングをpublishする
// 	(velodyne取り付け位置のキャリブレーションに利用)
//

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sstream>
#include <string>

using namespace std;

class MomentPublisher
{
	ros::NodeHandle n;
	ros::Rate r;
	ros::Publisher pub;

	std_msgs::Bool pub_flag;

	public:
	MomentPublisher()
		: r(1)
	{
		pub = n.advertise<std_msgs::Bool>("/flag/moment", 1);
	}

	bool pub_point_cloud(){
		string ans = "n";
		cout<<"save velodyne point cloud [y/n] : "<<endl;
		cin >> ans;
		if(ans!="n"){
			pub_flag.data = true;
			pub.publish(pub_flag);
		}else{
			pub_flag.data = false;
			return false;
		}
		return true;
	}

	bool spin()
	{
		ros::Rate loop_rate(r);

		while(ros::ok()){
			if(!pub_point_cloud()) break;
			ros::spinOnce();
			loop_rate.sleep();
		}
		return true;
	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_velodyne_moment");
	MomentPublisher mp;
	mp.spin();
	return 0;
}

