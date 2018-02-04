/*******************************
src: Map用クォータニオン計算
    (q1, q2, q3, ω)の値算出

last_update: '15.08.31
*******************************/
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/RotationMethods.h>
//#include <Matrix3x3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using namespace std;

int main(void)
{
    int mode=0;
    cout << "[1]:RPY-->Quat / [2]:Quat-->RPY  ==> ";
    cin  >> mode;     

    if(mode==1){
        double x_deg=0.0, y_deg=0.0, z_deg=0.0;
        double q1=0.0, q2=0.0, q3=0.0, w=0.0;

        cout << "Z軸中心回転角: z_deg = ";
        cin >> z_deg;
        cout << "Y軸中心回転角: y_deg = ";
        cin >> y_deg;
        cout << "X軸中心回転角: x_deg = ";
        cin >> x_deg;

        z_deg = z_deg / 180.0 * M_PI;//[rad]
        y_deg = y_deg / 180.0 * M_PI;// 〃
        x_deg = x_deg / 180.0 * M_PI;// 〃

		// tf::Quaternion quaternion = tf::createQuaternionFromRPY(x_deg, y_deg, z_deg);

        q1 = cos(z_deg*0.5)*cos(y_deg*0.5)*sin(x_deg*0.5) - sin(z_deg*0.5)*sin(y_deg*0.5)*cos(x_deg*0.5); 
        q2 = sin(z_deg*0.5)*cos(y_deg*0.5)*sin(x_deg*0.5) + cos(z_deg*0.5)*sin(y_deg*0.5)*cos(x_deg*0.5); 
        q3 = -1*cos(z_deg*0.5)*sin(y_deg*0.5)*sin(x_deg*0.5) + sin(z_deg*0.5)*cos(y_deg*0.5)*cos(x_deg*0.5); 
        w  = cos(z_deg*0.5)*cos(y_deg*0.5)*cos(x_deg*0.5) + sin(z_deg*0.5)*sin(y_deg*0.5)*sin(x_deg*0.5); 

        cout <<"q1 = "<< q1 << endl;    
        cout <<"q2 = "<< q2 << endl;    
        cout <<"q3 = "<< q3 << endl;    
        cout <<"ω  = " << w  << endl;
    }
    else if(mode==2){
        double qx=0, qy=0, qz=0, qw=0;
        double roll=0, pitch=0, yaw=0;

        //geometry_msgs::Quaternion Quat;

        cout<<"qx: "; cin >> qx;
        cout<<"qy: "; cin >> qy;
        cout<<"qz: "; cin >> qz;
        cout<<"qw: "; cin >> qw;
/* 
        Quat.x = qx;
        Quat.y = qy;
        Quat.z = qz;
        Quat.w = w;
*/    
//        btQuaternion btq(Quat.x, Quat.y, Quat.z, Quat.w);
  //      tf::Matrix3x3(btq).getRPY(roll, pitch, yaw);

        tf::Quaternion q(qx, qy, qz, qw);
        tf::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);      

        cout << " roll = " << roll/M_PI*180.0 << endl;
        cout << "pitch = " << pitch/M_PI*180.0<< endl;
        cout << "  yaw = " << yaw  /M_PI*180.0<< endl;
    }

    return 0;
}
