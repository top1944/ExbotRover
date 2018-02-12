#ifndef STATUSPUBLISHER_H
#define STATUSPUBLISHER_H

#include "ros/ros.h"
#include <boost/thread.hpp>
#include <boost/assign/list_of.hpp>
#include <algorithm>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Vector3.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Float64.h"
#include "tf/transform_broadcaster.h"
#include "AsyncSerial.h"
#include <memory.h>
#include <math.h>
#include <stdlib.h>

#define DISABLE 0
#define ENABLE 1
#define PI 3.14159265
#define FRAME_INFO_SIZE 7
// receive data demo
// #define data[8] {0xff, 0x01, 0x02, 0x00, 0x13, 0x0001, 0x76, 0xcd}

namespace serial_server
{
  typedef struct{
    int encoder_r; // right wheel encoder data  0x4
    int encoder_l; // left wheel encoder data  0x8
    float longitude; // the longitude of GPS data  0xC
    float latitude; // the latitude of GPS data  0x10
    float power; // the power supply voltage  0x14
    float temperature; //   0x18
    float theta; //   0x1C
    float IMU[9]; // mpu9250 3 axis data  0x40
  }UPLOAD_STATUS;


  class StatusPublisher
  {
  public:
    StatusPublisher();
    StatusPublisher(double separation, double radius);
    void Refresh();
    void Update(const char data[], unsigned int len);
    // respectively get the value of parameters
    double get_wheel_separation();
    double get_wheel_radius();
    geometry_msgs::Pose2D get_robot_pose2d();
    geometry_msgs::Twist get_robot_twist();
    std_msgs::Float64 get_power();
    nav_msgs::Odometry get_odom();
    void get_wheel_speed(double speed[2]);
    int Crc16(unsigned char *data, unsigned int len);
    UPLOAD_STATUS robot_status;

  private:
    // wheel separation: the midpoint of the wheel width: meters
    double wheel_separation;
    // wheel radius
    double wheel_radius;
    int encoder_ppr;
    geometry_msgs::Pose2D robotPose2D;
    geometry_msgs::Twist robotTwist;
    std_msgs::Float64 robotPower;
    nav_msgs::Odometry robotOdom;
    ros::NodeHandle nh;
    ros::Publisher mPose2DPub;
    ros::Publisher mTwistPub;
    ros::Publisher mPowerPub;
    ros::Publisher mOdomPub;

    bool mUpdated;

    boost::mutex mMutex;
  };
}

#endif // STATUSPUBLISHER_H
