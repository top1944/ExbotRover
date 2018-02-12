/*
 * File: main.cpp
 * Author: Zhu chaozheng
 * Created on January 5, 2018, 21:50pm
 * this is v1.0
 *
 */
#include "AsyncSerial.h"
#include "DiffDriverController.h"
#include "StatusPublisher.h"
#include <iostream>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "ros/package.h"

using namespace std;

int main(int argc, char **argv)
{
  cout << "this is a serial project writen by blackant team!" << endl;
  ros::init(argc, argv, "serial_server");
  ros::start();

  // get serial parameters
  std::string port;
  ros::param::param<std::string>("~port", port, "/dev/ttyUSB0");
  int baud;
  ros::param::param<int>("~baud", baud, 115200);
  cout<<"port:"<<port<<" baud:"<<baud<<endl;
  // get robot mechanical parameters
  double separation = 0, radius = 0;
  bool DebugFlag = false;
  ros::param::param<double>("~wheel_separation", separation, 0.25);
  ros::param::param<double>("~wheel_radius", radius, 0.075);
  ros::param::param<bool>("~debug_flag", DebugFlag, false);
  serial_server::StatusPublisher serial_status(separation, radius);

  // get robot control parameters
  double max_speed;
  string cmd_topic;
  ros::param::param<double>("~max_speed", max_speed, 2.0);
  ros::param::param<std::string>("~cmd_topic", cmd_topic, "cmd_vel");

  try {

    CallbackAsyncSerial serial(port,baud);
    // when data is coming at serial, this function will be executed, and execute the method Update()
    serial.setCallback(boost::bind(&serial_server::StatusPublisher::Update, &serial_status, _1, _2));
    // integrate different hardware by topic cmd_vel
    serial_server::DiffDriverController diffdriver(max_speed, cmd_topic, &serial_status, &serial);
    // create a new thread monitor data
    boost::thread cmd2serialThread(&serial_server::DiffDriverController::run, &diffdriver);
    // // send test flag
    //     char debugFlagCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'T'};
    //     if(DebugFlag){
    //       std::cout << "Debug mode Enabled" << std::endl;
    //       serial.write(debugFlagCmd, 5);
    //     }
    //     // send reset cmd
    //     char resetCmd[] = {(char)0xcd, (char)0xeb, (char)0xd7, (char)0x01, 'I'};
    //     serial.write(resetCmd, 5);
    //
    ros::Rate r(50);//发布周期为50hz
    while (ros::ok())
    {
      if(serial.errorStatus() || serial.isOpen() == false)
      {
        cerr << "Error: serial port closed unexpectedly" << endl;
        break;
      }
      serial_status.Refresh();//定时发布状态
      r.sleep();
    }
    quit:
    serial.close();

  } catch (std::exception& e)
  {
    cerr << "Exception: " << e.what() << endl;
  }
  ros::shutdown();
  return 0;
}
