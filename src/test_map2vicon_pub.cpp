
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_broadcaster.h>

using namespace std;

class ViconPosePublisher
{
	ros::NodeHandle n;
	ros::Publisher pub;

	std::string topic_vicon;
	geometry_msgs::Pose pose;

	public:
	ViconPosePublisher();
	void setPose(const geometry_msgs::Pose&);
	void publish();
};

ViconPosePublisher::ViconPosePublisher()
{
	n.param<string>("/topic_name/odom_vicon", topic_vicon, "/odom/vicon");
	pub = n.advertise<geometry_msgs::Pose>(topic_vicon, 1);
}

void ViconPosePublisher::setPose(const geometry_msgs::Pose& pose_in)
{
	pose = pose_in;
}

void ViconPosePublisher::publish()
{
	// cout << "published\n" << pose << endl;
	pub.publish(pose);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "pub_vicon_pose");

	geometry_msgs::Pose pose;
	double x = 0.0;
	double y = 0.0;
	// double yaw = 0.0;
	string input;

	ViconPosePublisher vp;

	for(int i=0; i<2; ++i){
		cout << "== set vicon's position " << i+1 << " (x, y) ==" << endl;
		cout << "x : "; cin >> input;
		x = stod(input);
		cout << "y : "; cin >> input;
		y = stod(input);
		// cout << "yaw : "; cin >> yaw;

		cin.ignore();
		cout << "ok? [Y/n] : ";
		getline(cin, input);
		if(input[0] == 'n' || input[0] == 'N'){
			--i;
		}else{
			pose.position.x = x;
			pose.position.y = y;
			pose.position.z = 0.0;
			// pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			pose.orientation.w = 1.0;

			vp.setPose(pose);
			vp.publish();
		}
	}

	cout << "finish" << endl;

	return 0;
}

