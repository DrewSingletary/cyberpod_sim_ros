#include "cyberpod_sim_ros/safety_filter_node.hpp"



ros::NodeHandle *nh_;
ros::NodeHandle *nhParams_;

ros::Subscriber sub_state_;
ros::Subscriber sub_inputDes_;

ros::Publisher pub_inputAct_;
ros::Publisher pub_info_;
ros::Publisher pub_backupTraj_;

cyberpod_sim_ros::input inputDes_;
cyberpod_sim_ros::input inputAct_;
cyberpod_sim_ros::state stateCurrent_;
cyberpod_sim_ros::filterInfo filter_info_;
nav_msgs::Path backTrajMsg_;

int32_t passThrough_;
uint32_t iter_;
double integration_dt_;
double backup_Tmax_;

#include "cyberpod_sim_ros/asif_filter.hpp"

using namespace Eigen;

void filterInput(void)
{
	iter_++;
	inputAct_.header.seq = iter_;
	inputAct_.header.stamp = ros::Time::now();
	inputAct_.header.frame_id = std::string("stateSeq=") + std::to_string(stateCurrent_.header.seq) + std::string(", inputDesSeq=") + std::to_string(inputDes_.header.seq);

	double xNow[nx] = {stateCurrent_.stateVec[0],stateCurrent_.stateVec[3],stateCurrent_.stateVec[5],stateCurrent_.stateVec[6]};
	double uDesNow[nu] = {inputDes_.inputVec[0]};
	double uActNow[nu] = {0.0};
	double tNow = 0.0;
	double relax;

	filter_info_.asifReturnCode = asif->filter(xNow,uDesNow,uActNow,relax);

	if(passThrough_==1)
	{
		std::copy(inputDes_.inputVec.begin(),inputDes_.inputVec.end(),inputAct_.inputVec.begin());
	}
	else
	{
		inputDes_.inputVec[0] = uActNow[0];
		inputDes_.inputVec[1] = uActNow[0];
	}

	inputAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
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

	Quaterniond cyberpod_q;
	Vector3d cyberpod_eul;
	cyberpod_eul(0) = 0.0;
	cyberpod_eul(2) = 0.0;
	uint32_t i = 0;
	for(auto & pose : backTrajMsg_.poses)
	{
		cyberpod_eul(1) = (*asif).backTraj_[i].second[2];
		eul2quatZYX(cyberpod_eul,cyberpod_q);

		pose.pose.position.x = (*asif).backTraj_[i].second[0];
		pose.pose.orientation.w = cyberpod_q.w();
		pose.pose.orientation.x = cyberpod_q.x();
		pose.pose.orientation.y = cyberpod_q.y();
		pose.pose.orientation.z = cyberpod_q.z();

		i++;
	}


	pub_backupTraj_.publish(backTrajMsg_);
}


int main(int argc, char *argv[])
{

	// Init ros
	ros::init(argc,argv,"safety_filter");

	// Instanciate NodeHandles
	nhParams_ = new ros::NodeHandle("~");
	nh_ = new ros::NodeHandle();

	// Init pubs, subs and srvs
	sub_state_ = nh_->subscribe<cyberpod_sim_ros::state>("state", 1, stateCallback);
	sub_inputDes_ = nh_->subscribe<cyberpod_sim_ros::input>("inputDes", 1, inputCallback);

	pub_inputAct_ = nh_->advertise<cyberpod_sim_ros::input>("input", 1);
	pub_info_ = nh_->advertise<cyberpod_sim_ros::filterInfo>("safety_filter_info", 1);
	pub_backupTraj_ = nh_->advertise<nav_msgs::Path>("backup_traj", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passThrough_,0);
	nhParams_->param<double>("integration_dt",integration_dt_,0.01);
	nhParams_->param<double>("backup_Tmax",backup_Tmax_,1.0);

	if(passThrough_!=0 && passThrough_!=1)
	{
		passThrough_ = 1;
		ROS_WARN("passTrough must be 0 or 1. Will be set to %i",passThrough_);
	}

	// Initialize variables
	iter_ = 0;
	backTrajMsg_.header.frame_id = "/world";
	inputDes_.inputVec.fill(0.0);


	// Initialize asif
	ASIF::ASIFimplicitTB::Options opts;
	opts.backTrajHorizon = 3.0;
	opts.backTrajDt = 0.01;
	opts.relaxCost = 10;
	opts.relaxSafeLb = 2.0;
	opts.relaxTTS = 30.0;
	opts.relaxMinOrtho = 60.0;
	opts.backTrajMinOrtho = 0.001;

	asif = new ASIF::ASIFimplicitTB(nx,nu,npSS,npBTSS,
	                                safetySet,backupSet,dynamics,dynamicsGradients,backupController);

	asif->initialize(lb,ub,opts);
	
	geometry_msgs::PoseStamped poseTmp;
	poseTmp.header.frame_id = "/world";
	poseTmp.pose.position.z = 0.195;
	poseTmp.pose.position.y = 0.0;
	backTrajMsg_.poses.resize((*asif).backTraj_.size(),poseTmp);

	// Display node info
	ROS_INFO("Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passThrough_);
	ROS_INFO("___integration_dt=%.3f",integration_dt_);
	ROS_INFO("___backup_Tmax=%.3f",backup_Tmax_);

	// Take it for a spin
	ros::spin();

	return 0;
}

