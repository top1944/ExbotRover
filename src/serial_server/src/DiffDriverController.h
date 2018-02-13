#ifndef DIFFDRIVERCONTROLLER_H
#define DIFFDRIVERCONTROLLER_H

#include <ros/ros.h>
#include "StatusPublisher.h"
#include "AsyncSerial.h"
#include <std_msgs/Bool.h>

namespace serial_server
{
  class DiffDriverController
  {
  public:
    DiffDriverController();
    DiffDriverController(double max_speed_, std::string cmd_topic_, StatusPublisher *status_, CallbackAsyncSerial *cmd_serial_);
    void run();
    void sendcmd(const geometry_msgs::Twist &command);
    void imuCalibration(const std_msgs::Bool &calFlag);
    void updateMoveFlag(const std_msgs::Bool &moveFlag);

  private:
    double max_wheel_speed; // unit is r/s
    std::string cmd_topic;
    StatusPublisher *status;
    CallbackAsyncSerial *cmd_serial;
    boost::mutex mMutex;
    bool MoveFlag;

  };

}
#endif
