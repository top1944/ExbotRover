#include "DiffDriverController.h"
#include <time.h>

namespace serial_server
{
  DiffDriverController::DiffDriverController()
  {
    max_wheel_speed = 2.0;
    cmd_topic = "cmd_vel";
    status = new StatusPublisher();
    cmd_serial = NULL;
    MoveFlag = true;
  }
  DiffDriverController::DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *status_, CallbackAsyncSerial *cmd_serial_)
  {
    MoveFlag = true;
    max_wheel_speed = max_speed_;
    cmd_topic = cmd_topic_;
    status = status_;
    cmd_serial = cmd_serial_;
  }
  void DiffDriverController::run()
  {
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub = nodeHandle.subscribe(cmd_topic, 1, &DiffDriverController::sendcmd, this);
    ros::spin();
  }

  void DiffDriverController::sendcmd(const geometry_msgs::Twist &command)
  {
    static time_t t1 = time(NULL), t2;
    int i = 0, wheel_l = 1;
    // separation is wheelbase, radius is wheel's radius, speed_lin is rounds, speed_ang
    double separation = 0, radius = 0, speed_lin = 0, speed_ang=0, speed_temp[2];
    int speed[2] = {0,0}; //right 0 and left 1
    char cmd_str[11] = {(char)0xFF,(char)0x01,(char)0x04,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x00,(char)0x5A,(char)0x88};

//    if(xq_status->get_status()==0) return; //  robot is still initialization
    separation = status->get_wheel_separation();
    radius = status->get_wheel_radius();
    // wheel_l=status->get_wheel_ppr();

    // translate speed unit, from meter to revolution
    speed_lin = command.linear.x / (2.0 * PI * radius);
    speed_ang = command.angular.z * separation / (2.0 * PI * radius);
    float scale=std::max(std::abs(speed_lin + speed_ang / 2.0), std::abs(speed_lin - speed_ang/2.0)) / max_wheel_speed;
    if(scale>1.0)
    {    	
        scale=1.0/scale;
    }
    else
    {    	
        scale=1.0;
    }
    // limit max speed
    speed_temp[0] = scale * (speed_lin + speed_ang / 2) / max_wheel_speed * 100.0;
    speed_temp[0] = std::min(speed_temp[0], 100.0);
    speed_temp[0] = std::max(-100.0, speed_temp[0]);

    speed_temp[1] = scale * (speed_lin - speed_ang / 2) / max_wheel_speed * 100.0;
    speed_temp[1] = std::min(speed_temp[1], 100.0);
    speed_temp[1] = std::max(-100.0, speed_temp[1]);

    //  command.linear.x/
    // now, we don't care about left and right, just only depend on right(0)
    for(i = 0;i< 2;i++)
    {    	
        speed[i] = floor(speed_temp[i]);
     if(speed[i] < 0)
     {
        cmd_str[5 + i] = (char)0x01; // back
        cmd_str[7 + i] = -speed[i];
     }
     else if(speed[i]>0)
     {     	
        cmd_str[5 + i] = (char)0x00; // forward
        cmd_str[7 + i] = speed[i];
     }
     else
     {     	
        cmd_str[5 + i] = (char)0x02; // stop
        cmd_str[7 + i] = (char)0x00;
     }
    }

    boost::mutex::scoped_lock lock(mMutex);
    if(!MoveFlag)
    {    	
        cmd_str[5 + i] = (char)0x02; // stop
        cmd_str[7 + i] = (char)0x00;
    }
    if(NULL!=cmd_serial)
	{
		if(cmd_str[5] != cmd_str[6])
		{
			cmd_str[7] = 80 + cmd_str[7];
			cmd_str[8] = 80 + cmd_str[8];
		}
		cmd_serial->write(cmd_str, 11);
    }

   // command.linear.x
  }

  void DiffDriverController::imuCalibration(const std_msgs::Bool &calFlag)
  {

  }

  void DiffDriverController::updateMoveFlag(const std_msgs::Bool &moveFlag)
  {  	
    boost::mutex::scoped_lock lock(mMutex);
    MoveFlag = moveFlag.data;
  }

}
