#include "cyberpod_sim_ros/safety_filter_node.hpp"

ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_inputDes_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_info_;

cyberpod_sim_ros::input inputDes_;
cyberpod_sim_ros::input inputAct_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::filterInfo filter_info_;

int32_t passThrough_;
uint32_t iter_;

CyberTimer<1000> filterTimer;

#include "cyberpod_sim_ros/asif_filter_truck.hpp"

using namespace Eigen;

void filterInput(void)
{
	double xNow[nx] = {stateCurrent_.x,stateCurrent_.y,stateCurrent_.v,stateCurrent_.theta,stateCurrent_.thetaDot};
	double uDesNow[nu] = {inputDes_.inputVec[0],inputDes_.inputVec[1]};
	double uActNow[nu] = {0.0,0.0};
	double relax;

	double h[0];
	double Dh[5];
	safetySet(xNow,h,Dh);
	ROS_INFO("%f",h[0]);

	filterTimer.tic();
	int32_t rc = asif->filter(xNow,uDesNow,uActNow,relax);
	filterTimer.toc();

	if (rc < 0) {
		ROS_INFO("QP FAILED");
	}

	filter_info_.filterTimerUs = filterTimer.getAverage()*1.0e6;

	if(passThrough_>0 /*|| rc==-1 || rc==2*/)
	{
		std::copy(inputDes_.inputVec.begin(),inputDes_.inputVec.end(),inputAct_.inputVec.begin());
	}
	else
	{
		inputAct_.inputVec[0] = uActNow[0];
		inputAct_.inputVec[1] = uActNow[1];
		ROS_INFO("u: %f %f",uActNow[0],uActNow[1]);
	}

	iter_++;
	inputAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
	inputAct_.header.seq = iter_;
	inputAct_.header.stamp = ros::Time::now();
	inputAct_.header.frame_id = std::string("stateSeq=") + std::to_string(stateCurrent_.header.seq) + std::string(", inputDesSeq=") + std::to_string(inputDes_.header.seq);
	filter_info_.header.seq = iter_;
	filter_info_.header.stamp = inputAct_.header.stamp;
}

void inputCallback(const cyberpod_sim_ros::input::ConstPtr msg)
{
	inputDes_ = *msg;
}

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{

	stateCurrent_ = *msg;

	filterInput();
	pub_inputAct_.publish(inputAct_);
	pub_info_.publish(filter_info_);
}


int main(int argc, char *argv[])
{

	// Init ros
	ros::init(argc,argv,"safety_filter");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("state_true", 1, stateCallback);
	sub_inputDes_ = nh_->subscribe<cyberpod_sim_ros::input>("inputDes", 1, inputCallback);

	pub_inputAct_ = nh_->advertise<cyberpod_sim_ros::input>("input", 1);
	pub_info_ = nh_->advertise<cyberpod_sim_ros::filterInfo>("safety_filter_info", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passThrough_,0);

	if(passThrough_!=0 && passThrough_!=1)
	{
		passThrough_ = 1;
		ROS_WARN("passTrough must be 0 or 1. Will be set to %i",passThrough_);
	}
	// Initialize variables
	iter_ = 0;
	inputDes_.inputVec.fill(0.0);

	ASIF::ASIF::Options opts;
	opts.relaxLb = 0.1;

	asif = new ASIF::ASIF(nx,nu,npSS,safetySet,dynamics);
	asif->initialize(lb,ub,opts);

	// Display node info
	ROS_INFO("Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passThrough_);
	// Take it for a spin
	ros::spin();

	return 0;
}
