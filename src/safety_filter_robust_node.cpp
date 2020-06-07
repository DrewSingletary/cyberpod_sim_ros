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
double backup_dt_;

int time_delay_;

std::vector<double> inputBuffer_;

bool learning = false;

CyberTimer<1000> filterTimer;

#include "cyberpod_sim_ros/asif_filter_robust.hpp"
#include "learning/weights.h"

using namespace Eigen;

void filterInput(void)
{
	double xNow[nx];
	if (time_delay_ > 0) {
		double f[nx]; double g[nx*nu]; double xDot[nx];
		double x_0[nx] = {stateCurrent_.stateVec[0],stateCurrent_.stateVec[3],stateCurrent_.stateVec[5],stateCurrent_.stateVec[6]};
		memcpy(xNow,x_0,sizeof(double)*nx);
		for (int i = 0; i < time_delay_; i++) {
			dynamics(xNow,f,g);
			for (int j = 0; j < nx; j++) {
				xDot[j] = f[j];
				for (int k = 0; k < nu; k++) {
						xDot[j] +=g[j+k*nx]*inputBuffer_[i];
				}
				xNow[j] += xDot[j]*integration_dt_;
			}
		}
		// ROS_INFO("x0: (%f,%f,%f,%f) input: (%f) xnow: (%f,%f,%f,%f)",x_0[0],x_0[1],x_0[2],x_0[3],inputBuffer_[0],
		// 																								 xNow[0],xNow[1],xNow[2],xNow[3]);
	}
	else {
		xNow[0] = stateCurrent_.stateVec[0];
		xNow[1] = stateCurrent_.stateVec[3];
		xNow[2] = stateCurrent_.stateVec[5];
		xNow[3] = stateCurrent_.stateVec[6];
	}
	double uDesNow[nu] = {inputDes_.inputVec[0]};
	double uActNow[nu] = {0.0};
	double tNow = 0.0;
	double relax[2];

	filterTimer.tic();
	int32_t rc = asif->filter(xNow,uDesNow,uActNow,relax);
	filterTimer.toc();

	if (time_delay_ > 0){
		inputBuffer_.push_back(uActNow[0]);
		inputBuffer_.erase(inputBuffer_.begin());
	}

	if (rc < 0) {
		ROS_INFO("QP FAILED");
	}

	if (learning) {
		filter_info_.Lfh_diff[0] = asif->learning_data_.Lfh_diff[0];
		for (int i = 0; i < (int) nu; i ++) {
			filter_info_.Lgh_diff[i] = asif->learning_data_.Lgh_diff[i];
		}
	}
	filter_info_.hSafetyNow = asif->hSafetyNow_;
	filter_info_.indexDebug = asif->index_debug_;
	for (int i = 0; i < npSS; i++) {
		filter_info_.hDebug[i] = asif->h_index_[i];
	}

	filter_info_.hBackupEnd = asif->hBackupEnd_;
	filter_info_.filterTimerUs = filterTimer.getAverage()*1.0e6;

	// filter_info_.BTorthoBS = asif->BTorthoBS_;
	// filter_info_.TTS = asif->TTS_;
	for (int i = 0; i < nx*npSS; i++)
		filter_info_.DhDebug[i] = asif->Dh_index_[i];
	filter_info_.asifStatus = ASIF::ASIFimplicitRB::filterErrorMsgString(rc);
	filter_info_.relax1 = relax[0];
	filter_info_.relax2 = relax[1];

	std::copy((*asif).backTraj_.back().second.begin(),(*asif).backTraj_.back().second.begin()+4,filter_info_.xBackupEnd.begin());

	if(passThrough_>0 /*|| rc==-1 || rc==2*/)
	{
		std::copy(inputDes_.inputVec.begin(),inputDes_.inputVec.end(),inputAct_.inputVec.begin());
	}
	else
	{
		inputAct_.inputVec[0] = uActNow[0];
		inputAct_.inputVec[1] = uActNow[0];
	}

	iter_++;
	inputAct_.status = static_cast<uint8_t>(STATUS::RUNNING);
	inputAct_.header.seq = iter_;
	inputAct_.header.stamp = ros::Time::now();
	inputAct_.header.frame_id = std::string("stateSeq=") + std::to_string(stateCurrent_.header.seq) + std::string(", inputDesSeq=") + std::to_string(inputDes_.header.seq);
	backTrajMsg_.header.seq = iter_;
	backTrajMsg_.header.stamp = inputAct_.header.stamp;
	filter_info_.header.seq = iter_;
	filter_info_.header.stamp = inputAct_.header.stamp;
}

void inputCallback(const cyberpod_sim_ros::input::ConstPtr msg)
{
	inputDes_ = *msg;
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

void stateCallback(const cyberpod_sim_ros::state::ConstPtr msg)
{

	stateCurrent_ = *msg;
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
	pub_backupTraj_ = nh_->advertise<nav_msgs::Path>("backup_traj", 1);

	// Retreive params
	nhParams_->param<int32_t>("pass_through",passThrough_,0);
	nhParams_->param<double>("integration_dt",integration_dt_,0.01);
	nhParams_->param<double>("backup_Tmax",backup_Tmax_,1.0);
	nhParams_->param<double>("backup_dt",backup_dt_,0.01);
	nhParams_->param<int>("time_delay",time_delay_,0);

	if(passThrough_!=0 && passThrough_!=1)
	{
		passThrough_ = 1;
		ROS_WARN("passTrough must be 0 or 1. Will be set to %i",passThrough_);
	}

	// Initialize variables
	iter_ = 0;
	backTrajMsg_.header.frame_id = "/world";
	inputDes_.inputVec.fill(0.0);

	if (time_delay_ > 0) {
		for (int i = 0; i < time_delay_; i++)
			inputBuffer_.push_back(0.0);
	}

	// Initialize asif
	// ASIF::ASIFimplicitTB::Options opts;
	// opts.backTrajHorizon = backup_Tmax_;
	// opts.backTrajDt = integration_dt_;
	// opts.relaxCost = 10;
	// opts.relaxSafeLb = 2.0;
	// opts.relaxTTS = 30.0;
	// opts.relaxMinOrtho = 60.0;
	// opts.backTrajMinOrtho = 0.001;

	ROS_INFO("Setting up robust asif 1");

	asif = new ASIF::ASIFimplicitRB(nx,nu,npSS,npBS,npBTSS,
	                              safetySet,safetySet_int,backupSet,backupSet_int,
																dynamics,dynamics_int,
																dynamicsGradients,dynamicsGradients_int,
																backupController);
	ROS_INFO("Setting up robust asif 2");
	ASIF::ASIFimplicitRB::Options opts;
	opts.backTrajHorizon = backup_Tmax_;
	opts.backTrajDt = integration_dt_;
	opts.backContDt = backup_dt_;
	opts.relaxReachLb = 5.;
	opts.relaxSafeLb = 1.;
	opts.n_debug = -1;
	opts.use_learning = learning;
	opts.x_unc = new double[nx];
	for (uint32_t i = 0; i < nx; i++)
		opts.x_unc[i] = x_unc_[i];
	if (learning){
	ASIF::LearningData learning_data_;
	learning_data_.d_drift_in = d_drift_in;
	learning_data_.d_act_in = d_act_in;
	learning_data_.d_drift_hidden = d_drift_hidden;
	learning_data_.d_act_hidden = d_act_hidden;
	learning_data_.d_drift_hidden_2 = d_drift_hidden_2;
	learning_data_.d_act_hidden_2 = d_act_hidden_2;
	learning_data_.d_drift_out = d_drift_out;
	learning_data_.d_act_out = d_act_out;
	learning_data_.w_1_drift = w_1_drift;
	learning_data_.w_2_drift = w_2_drift;
	learning_data_.w_3_drift = w_3_drift;
	learning_data_.b_1_drift = b_1_drift;
	learning_data_.b_2_drift = b_2_drift;
	learning_data_.b_3_drift = b_3_drift;
	learning_data_.w_1_act = w_1_act;
	learning_data_.w_2_act = w_2_act;
	learning_data_.w_3_act = w_3_act;
	learning_data_.b_1_act = b_1_act;
	learning_data_.b_2_act = b_2_act;
	learning_data_.b_3_act = b_3_act;
	asif->learning_data_ = learning_data_;
	}
	asif->initialize(lb,ub,opts);
	for (int i = 0; i < nx*npSS; i++)
		filter_info_.DhDebug.push_back(0.0);

	for (int i = 0; i < npSS; i++)
		filter_info_.hDebug.push_back(0.0);

	for (int i = 0; i < nu; i++)
		filter_info_.Lgh_diff.push_back(0.0);

	filter_info_.Lfh_diff.push_back(0.0);

	geometry_msgs::PoseStamped poseTmp;
	poseTmp.header.frame_id = "/world";
	poseTmp.pose.position.z = 0.195;
	poseTmp.pose.position.y = 0.0;
	backTrajMsg_.poses.resize((*asif).backTraj_.size(),poseTmp);

	// Display node info
	ROS_INFO("Robust Safety Filter node successfuly started with:");
	ROS_INFO("___pass_through=%u",passThrough_);
	ROS_INFO("___integration_dt=%.3f",integration_dt_);
	ROS_INFO("___backup_Tmax=%.3f",backup_Tmax_);

	// Take it for a spin
	ros::spin();

	return 0;
}
