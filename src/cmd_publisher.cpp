#include "cyberpod_sim_ros/common.hpp"
#include "ros/ros.h"
#include "cyberpod_sim_ros/input.h"
#include "cyberpod_sim_ros/state.h"
#include "cyberpod_sim_ros/cmd.h"
#include "cyberpod_sim_ros/common.hpp"
#include "cyberpod_sim_ros/keyboard.hpp"

#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <signal.h>
#include <stdlib.h>

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Publisher pub_cmd_;

cyberpod_sim_ros::cmd cmd_;

double cmdVec_[2] = {0.};

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"cmd_pub");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Initialize some variables
	cmd_.cmd.resize(2,0.0);

	// Init pubs, subs and srvs
	pub_cmd_ = nh_->advertise<cyberpod_sim_ros::cmd>("/cyberpod_sim_ros/cmd", 1);

	// Display node info
	ROS_INFO("cmd pub node successfuly started:");

	// Initialize rate
	ros::Rate rate(500);

	// Initialize time
  ros::Time begin = ros::Time::now();

	// Take it for a spin
	while (ros::ok()){
		// Compute command
		cmd_.header.stamp = ros::Time::now();

    cmdVec_[0] = 0.0;
    cmdVec_[1] = 0.0;

		if (ros::Time::now()- begin > ros::Duration(2)) {
      cmdVec_[0] = 1.5;
      cmdVec_[1] = 0.0;
    }

    if (ros::Time::now()- begin > ros::Duration(5)) {
      cmdVec_[0] = -1.5;
      cmdVec_[1] = 0.0;
    }

    if (ros::Time::now()- begin > ros::Duration(8)) {
      cmdVec_[0] = 1.0;
      cmdVec_[1] = 0.0;
    }

    if (ros::Time::now()- begin > ros::Duration(10)) {
      cmdVec_[0] = 0.0;
      cmdVec_[1] = 0.0;
			ROS_INFO("PUBLISHING ZEROS");
    }

		cmd_.cmd[0] = cmdVec_[0];
		cmd_.cmd[1] = cmdVec_[1];

		// Publish cmd message
		pub_cmd_.publish(cmd_);

		//Wait for tick
		rate.sleep();
	}

	ros::shutdown();

	return 0;
}
