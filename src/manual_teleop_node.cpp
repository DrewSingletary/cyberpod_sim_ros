#include "cyberpod_sim_ros/manual_teleop_node.hpp" 

using namespace Eigen;

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;
ros::Publisher pub_cmd_;

cyberpod_sim_ros::cmd cmd_;

uint32_t iter_;
double hz_, vxy_, vz_, vyaw_, boost_;
std::string input_dev_;
bool run_;
double smoothing_dt_;
Vector4d cmdVec_(0.0,0.0,0.0,0.0);

void my_handler(int sig)
{
	ROS_INFO("Received SIGINT signal");
	run_ = false;
}

int main (int argc, char *argv[])
{
	// Init ros
	ros::init(argc,argv,"planner");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Initialize some variables
	iter_ = 0;
	run_ = true;
	cmd_.vDes.fill(0.0);
	int ch;
	struct termios oldt;
	struct termios newt;

	// Remove terminal echo
	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~ECHO;
	tcsetattr(fileno(stdin), TCSANOW, &newt);

	// Init pubs, subs and srvs
	pub_cmd_ = nh_->advertise<cyberpod_sim_ros::cmd>("uav_cmd_des", 1);

	// Retreive params
	nhParams_->param<std::string>("input_dev",input_dev_,"/dev/input/event0");
	nhParams_->param<double>("hz",hz_,100.0);
	nhParams_->param<double>("vxy",vxy_,1.0);
	nhParams_->param<double>("vz",vz_,1.0);
	nhParams_->param<double>("vyaw",vyaw_,1.0);
	nhParams_->param<double>("boost",boost_,2.0);
	nhParams_->param<double>("smoothing_dt",smoothing_dt_,0.0);

	//Initialize keyboard observer
	cKeyboard kb(input_dev_.c_str());
	if(!kb.active)
	{
		ROS_INFO("Could not find keyboard at: %s",input_dev_.c_str());
	}

	// Display node info
	ROS_INFO("Manual teleop node successfuly started with:");
	ROS_INFO("___input_dev=%s",input_dev_.c_str());	
	ROS_INFO("___hz=%.0f",hz_);
	ROS_INFO("___vxy=%.2f",vxy_);
	ROS_INFO("___vz=%.2f",vz_);
	ROS_INFO("___vyaw=%.2f",vyaw_);
	ROS_INFO("___boost=%.2f",boost_);
	ROS_INFO("___smoothing_dt=%.3f",smoothing_dt_);

	// Initialize rate
	ros::Rate rate(hz_);

	// Initialize SIGINT handler
	signal(SIGINT,my_handler);

	// Take it for a spin
	while(run_)
	{
		//Get latest parameters
		nhParams_->param<double>("vxy",vxy_,1.0);
		nhParams_->param<double>("vz",vz_,1.0);
		nhParams_->param<double>("vyaw",vyaw_,1.0);

		// Compute command
		Vector4d cmdVec(0.0,0.0,0.0,0.0);

		double boost = 1.0;
		if(kb.getKeyState(KEY_LEFTSHIFT))
			boost = boost_;

		if(kb.getKeyState(KEY_W))
			cmdVec+=boost*vxy_*moveBindings['w'];

		if(kb.getKeyState(KEY_S))
			cmdVec+=boost*vxy_*moveBindings['s'];

		if(kb.getKeyState(KEY_A))
			cmdVec+=boost*vxy_*moveBindings['a'];

		if(kb.getKeyState(KEY_D))
			cmdVec+=boost*vxy_*moveBindings['d'];

		if(kb.getKeyState(KEY_Q))
			cmdVec+=boost*vyaw_*moveBindings['q'];

		if(kb.getKeyState(KEY_E))
			cmdVec+=boost*vyaw_*moveBindings['e'];

		if(kb.getKeyState(KEY_SPACE))
			cmdVec+=boost*vz_*moveBindings[' '];

		if(kb.getKeyState(KEY_LEFTCTRL))
			cmdVec+=boost*vz_*moveBindings['c'];

		cmd_.header.seq = iter_;
		cmd_.header.stamp = ros::Time::now();
		cmd_.status = static_cast<uint8_t>(STATUS::RUNNING);

		if(smoothing_dt_>=0.001)
		{
			cmdVec_ += (1/hz_)*(cmdVec-cmdVec_)/(smoothing_dt_/3.0);
		}
		else
			cmdVec_ = cmdVec;

		cmd_.vDes[0] = cmdVec_(0);
		cmd_.vDes[1] = cmdVec_(1);
		cmd_.vDes[2] = cmdVec_(2);
		cmd_.vDes[3] = cmdVec_(3);
		iter_++;

		// Publish cmd message
		pub_cmd_.publish(cmd_);

		//Wait for tick
		rate.sleep();
	}

	// Send last command as 0
	cmd_.vDes[0] = 0.0;
	cmd_.vDes[1] = 0.0;
	cmd_.vDes[2] = 0.0;
	cmd_.vDes[3] = 0.0;

	pub_cmd_.publish(cmd_);	

	// Reapply terminal settings
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

	ros::shutdown();

	return 0;
}