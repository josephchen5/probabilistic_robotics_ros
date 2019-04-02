#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <iostream>
#include <math.h>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <tf/transform_broadcaster.h>


#define MATRIX_SIZE 50
using namespace std;

double interval=0, sigma,roll,pitch,yaw;
bool start = false;
sensor_msgs::Imu imuMsg;
geometry_msgs::Point p;


// Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>

Eigen::Matrix3d I = Eigen::Matrix3d::Zero(); //初始化为零
Eigen::Matrix3d C = Eigen::Matrix3d::Zero(); //初始化为零
Eigen::Vector3d ea = Eigen::Vector3d::Zero(); //初始化为零
Eigen::Vector3d sg = Eigen::Vector3d::Zero(); //初始化为零


void imuCallback(sensor_msgs::Imu msg)
{
  if(start){
    interval=2;

    interval = (msg.header.stamp.nsec-imuMsg.header.stamp.nsec)*0.000000001;
    cout<<"T = "<<interval<<endl;
//    ROS_INFO("I heard: ");

    //variation of angles
    double wx,wy,wz;
    wx = msg.angular_velocity.x * interval;
    wy = msg.angular_velocity.y * interval;
    wz = msg.angular_velocity.z * interval;

    Eigen::Matrix3d B = Eigen::Matrix3d::Zero(); //初始化为零
    B <<   0, -wz,  wy,
          wz,   0, -wx,
         -wy,  wx,   0;

   // cout<<"B = "<<endl<<B<<endl;

    sigma = pow(pow(wx,2)+pow(wy,2)+pow(wz,2),0.5);
 //   cout << "sigma = "<<sigma<<endl;

    //compute rotation matrix C
    if(sigma!=0)
            C = C * (I + (sin(sigma)/sigma)*B + ((1-cos(sigma))/sigma/sigma)*(B*B));
    cout<<"C = "<<endl<<C<<endl;;

    //COMPUTE RPY
    ea = C.eulerAngles(0, 1, 2);
    roll = ea(0);
    pitch = ea(1);
    yaw = ea(2);
    cout<<"RPY = "<<ea[0]<<" "<<ea[1]<<" "<<ea[2]<<" "<<endl;

 //   cout<<"position_global = "<<sg[0]<<" "<<sg[1]<<" "<<sg[2]<<" "<<endl<<"-----------------------------------"<<endl;

    p.x = sg[0];
    p.y = sg[1];
    p.z = sg[2];

        //publish tf
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    //transform.setOrigin( tf::Vector3(0, 0, 0) );
    transform.setOrigin( tf::Vector3(sg[0], sg[1], sg[2]) );
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "body"));




  }

  imuMsg = msg;
  start = true;


}





int main(int argc, char **argv)
{

  // Matrix3d 实质上是 Eigen::Matrix<double, 3, 3>

  Eigen::Matrix3d D = Eigen::Matrix3d::Zero(); //初始化为零

  I << 1, 0, 0, 0, 1, 0,0, 0, 1;
//  cout << I << endl;
  C = I;
// cout << C << endl;

  Eigen::Vector3d ab = Eigen::Vector3d::Zero(); //初始化为零
  Eigen::Vector3d ag = Eigen::Vector3d::Zero(); //初始化为零
  Eigen::Vector3d vg = Eigen::Vector3d::Zero(); //初始化为零
  Eigen::Vector3d gg = Eigen::Vector3d::Zero(); //初始化为零


  vg << 0,0,0;
  sg << 0,0,0;
  gg << 0,0,9.80665;

 // cout << vg << endl;
 // cout << sg << endl;
 // cout << gg << endl;




  ros::init(argc, argv, "imu_path");
  ros::NodeHandle nh;

  ros::Subscriber imu_sub = nh.subscribe("/imu", 1000, imuCallback);




  ros::spin();

  return 0;
}
