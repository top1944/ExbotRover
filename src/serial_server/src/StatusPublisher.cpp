#include "StatusPublisher.h"

namespace serial_server
{
  StatusPublisher::StatusPublisher()
  {
    mUpdated = false;
    wheel_separation = 0.25;
    wheel_radius = 0.075;
    // the rotation of the wheel, the numeric value of the encoder
    encoder_ppr = 130;

    robotPose2D.x = 0.0;
    robotPose2D.y = 0.0;
    robotPose2D.theta = 0.0;

    robotTwist.linear.x = 0.0;
    robotTwist.linear.y = 0.0;
    robotTwist.linear.z = 0.0;
    robotTwist.angular.x = 0.0;
    robotTwist.angular.y = 0.0;
    robotTwist.angular.z = 0.0;

    robotPower.data = 0.0;

    // init struct robot_status, set struct is 0
    int i = 0;
    int *status;
    status = (int *)&robot_status;
    memset(status, 0, sizeof(status));

    mPose2DPub = nh.advertise<geometry_msgs::Pose2D> ("serial_server/Pose2D", 1, true);
    mTwistPub = nh.advertise<geometry_msgs::Twist> ("serial_server/Twist", 1, true);
    mPowerPub = nh.advertise<std_msgs::Float64> ("serial_server/Power", 1, true);
    mOdomPub = nh.advertise<nav_msgs::Odometry> ("serial_server/Odom", 1, true);

  }
  StatusPublisher::StatusPublisher(double separation, double radius)
  {
    new (this)StatusPublisher();
    wheel_separation = separation;
    wheel_radius = radius;
  }

  // according to the method Update(), get struct UPLOAD_STATUS newest value
  void StatusPublisher::Refresh()
  {
    boost::mutex::scoped_lock lock(mMutex);
    static double theta_last=0.0;
    static unsigned int ii=0;
        ii++;
    if(mUpdated)
    {
      // Time
      ros::Time current_time = ros::Time::now();
      //pose
      double delta_robot, delta_x, delta_y, delta_theta, var_len, var_angle, theta;

      // square of unit length and angle
      var_len = (50.0f / encoder_ppr * 2.0f * PI * wheel_radius) * (50.0f / encoder_ppr * 2.0f * PI * wheel_radius);
      var_angle = (0.01f / 180.0f * PI) * (0.01f / 180.0f * PI);

     
      // actual distance of center(right and left wheels) in small periods of time
      delta_robot = (robot_status.encoder_r + robot_status.encoder_l - 60000) / 2.0f * 1.0f / encoder_ppr * 2.0f * PI * wheel_radius;
	std::cout << robot_status.encoder_r <<" " << robot_status.encoder_l << " " << delta_robot << std::endl;
      // if actual distance is too long
	/*	
      if(delta_robot > 0.05||delta_robot < -0.05)
      {
        delta_robot = 0;
      }
	*/
      // x,y in small periods of time
      delta_x = delta_robot * cos(robotPose2D.theta * PI / 180.0f);
      delta_y = delta_robot * sin(robotPose2D.theta * PI / 180.0f);
	std::cout<< " " << delta_x << " " << delta_y << std::endl;		
      theta = robot_status.theta;
	std::cout << theta << std::endl;
      delta_theta = theta - theta_last;
      theta_last = theta;

      if(delta_theta > 270 ) delta_theta -= 360;
      if(delta_theta < -270 ) delta_theta += 360;

      if(delta_theta > 20||delta_theta < -20)
      {
        delta_theta = 0;
      }
      robotPose2D.x += delta_x;
      robotPose2D.y += delta_y;
      robotPose2D.theta += delta_theta;
	std::cout << "pose2D" << robotPose2D.x << " " << robotPose2D.y << " " << robotPose2D.theta << std::endl;
      if(robotPose2D.theta > 360.0) robotPose2D.theta -= 360;
      if(robotPose2D.theta < 0.0) robotPose2D.theta += 360;
      mPose2DPub.publish(robotPose2D);
      //Twist
      double angle_speed;
      robotTwist.linear.x = delta_robot * 50.0f;
      angle_speed = -robot_status.IMU[5];
      robotTwist.angular.z = angle_speed * PI / 180.0f;
      mTwistPub.publish(robotTwist);

      robotPower.data = robot_status.power;
      mPowerPub.publish(robotPower);
      robotOdom.header.stamp = current_time;
      robotOdom.header.frame_id = "odom";
      robotOdom.pose.pose.position.x = robotPose2D.x;
      robotOdom.pose.pose.position.y = robotPose2D.y;
      robotOdom.pose.pose.position.z = 0.0f;
      geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(robotPose2D.theta / 180.0f*PI);
      robotOdom.pose.pose.orientation = odom_quat;
	  /*
      robotOdom.pose.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (var_angle) ;
	  */
      robotOdom.child_frame_id = "base_link";
//      robotOdom.child_frame_id = "base_footprint";
      robotOdom.twist.twist.linear.x = robotTwist.linear.x; // * cos(robotPose2D.theta* PI / 180.0f);
      robotOdom.twist.twist.linear.y = robotTwist.linear.y; // * sin(robotPose2D.theta* PI / 180.0f);
      robotOdom.twist.twist.angular.z = robotTwist.angular.z;
	  /*
      robotOdom.twist.covariance =  boost::assign::list_of(var_len) (0) (0)  (0)  (0)  (0)
                                                              (0) (var_len)  (0)  (0)  (0)  (0)
                                                              (0)   (0)  (999) (0)  (0)  (0)
                                                              (0)   (0)   (0) (999) (0)  (0)
                                                              (0)   (0)   (0)  (0) (999) (0)
                                                              (0)   (0)   (0)  (0)  (0)  (var_angle) ;
                                                              */
      mOdomPub.publish(robotOdom);
      // pub transform

      static tf::TransformBroadcaster br;
      tf::Quaternion q;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(robotPose2D.x, robotPose2D.y, 0.0));
      q.setRPY(0, 0, robotPose2D.theta/180*PI);
      transform.setRotation(q);
//      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_footprint"));
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

      ros::spinOnce();
      mUpdated = false;
    }
  }

  void StatusPublisher::Update(const char data[], unsigned int len)
  {
    boost::mutex::scoped_lock lock(mMutex);

    int i = 0, j = 0;
    int *receive_byte;
    int rx_step = 0;
    int crc_code = 0;
    static unsigned char last_str[2] = {0x00, 0x00};
    static unsigned char new_packet_ctr = DISABLE; // enable mean a new packet, disable mean unhandle packet
    static int valid_packet_len = 0; // packet correct length
    static int total_packet_len = 0;
    static unsigned char cmd_string_buf[512];
    unsigned char current_str = 0x00;
    const int cmd_string_max_size = 512;
    receive_byte = (int *)&robot_status;
    /*
     *
     * use swith to verify per frame
     * 0xFF is verified frame head
     * 0x02 is also verified frame head
     * 0x40 ....
     *
     *
     */
    for(i = 0; i < len; i ++)
    {
    	
    		
	    current_str = data[i];
     	if(last_str[0] == 255 && last_str[1] == 2 && current_str == 64)
      	{
	 		new_packet_ctr = ENABLE;
		 	valid_packet_len = 0;
		 	total_packet_len = valid_packet_len;
        	last_str[0] = last_str[1];
	        last_str[1] = current_str;
    	}
	    last_str[0] = last_str[1];
    	last_str[1] = current_str;
		if(new_packet_ctr == ENABLE)
		{
			valid_packet_len = current_str;
			new_packet_ctr = DISABLE;
		}
		else
		{
			total_packet_len ++;
			cmd_string_buf[total_packet_len - 1] = current_str;
			if(valid_packet_len == total_packet_len && valid_packet_len > 0)
			{
				
				memcpy(&receive_byte[0], &cmd_string_buf[2], 64);
		   		//crc_code = Crc16(&cmd_string_buf[0], total_packet_len - 2);
			    mUpdated = true;
				valid_packet_len = 0;
				total_packet_len = 0;
			}
		}
	}
    return;
  }
  // respectively get the value of parameters
  double StatusPublisher::get_wheel_separation()
  {
    return wheel_separation;
  }
  double StatusPublisher::get_wheel_radius()
  {
    return wheel_radius;
  }
  geometry_msgs::Pose2D StatusPublisher::get_robot_pose2d()
  {
    return robotPose2D;
  }
  geometry_msgs::Twist StatusPublisher::get_robot_twist()
  {
    return robotTwist;
  }
  std_msgs::Float64 StatusPublisher::get_power()
  {
    return robotPower;
  }
  nav_msgs::Odometry StatusPublisher::get_odom()
  {
    return robotOdom;
  }
  void StatusPublisher::get_wheel_speed(double speed[2])
  {
    // right 1, left 0
    //speed[0] = robot_status.encoder_l * 2.0 * PI * wheel_radius;
    //speed[1] = robot_status.encoder_r * 2.0 * PI * wheel_radius;
  }
  int StatusPublisher::Crc16(unsigned char *data, unsigned int len)
  {
    int crc, i;
	  crc = 0;
	  while(--len >= 0)
    {
		  crc = (crc ^ (((int)*data++) << 8));
		  for(i = 0; i < 8; ++i)
		    if(crc & 0x8000)
		      crc = ((crc << 1) ^ 0x1021);
		    else
		      crc = crc << 1;
	   }
	   return (crc & 0xFFFF);
  }



}
