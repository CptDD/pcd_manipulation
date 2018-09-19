#include <despot/evaluator.h>
#include <despot/core/information.h>
#include <iostream>
#include <ros/ros.h>

#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>

//#include <bulb_processor/classify.h>
#include <bulb_processor/classify_multiple.h>
#include <bulb_scanner/position.h>
#include <conveyor_advance/advance.h>

#define POSITION_SERVICE "bulb_move"
#define CLASSIFICATION_SERVICE "bulb_classification_service"
#define CONVEYOR_ADVANCE_SERVICE "conveyor_advance"


using namespace std;

namespace despot {

/* =============================================================================
 * EvalLog class
 * =============================================================================*/

time_t EvalLog::start_time = 0;
double EvalLog::curr_inst_start_time = 0;
double EvalLog::curr_inst_target_time = 0;
double EvalLog::curr_inst_budget = 0;
double EvalLog::curr_inst_remaining_budget = 0;
int EvalLog::curr_inst_steps = 0;
int EvalLog::curr_inst_remaining_steps = 0;
double EvalLog::allocated_time = 1.0;
double EvalLog::plan_time_ratio = 1.0;

EvalLog::EvalLog(string log_file) :
	log_file_(log_file) {
	ifstream fin(log_file_.c_str(), ifstream::in);
	if (!fin.good() || fin.peek() == ifstream::traits_type::eof()) {
		time(&start_time);
	} else {
		fin >> start_time;

		int num_instances;
		fin >> num_instances;
		for (int i = 0; i < num_instances; i++) {
			string name;
			int num_runs;
			fin >> name >> num_runs;
			runned_instances.push_back(name);
			num_of_completed_runs.push_back(num_runs);
		}
	}
	fin.close();
}

void EvalLog::Save() {
	ofstream fout(log_file_.c_str(), ofstream::out);
	fout << start_time << endl;
	fout << runned_instances.size() << endl;
	for (int i = 0; i < runned_instances.size(); i++)
		fout << runned_instances[i] << " " << num_of_completed_runs[i] << endl;
	fout.close();
}

void EvalLog::IncNumOfCompletedRuns(string problem) {
	bool seen = false;
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == problem) {
			num_of_completed_runs[i]++;
			seen = true;
		}
	}

	if (!seen) {
		runned_instances.push_back(problem);
		num_of_completed_runs.push_back(1);
	}
}

int EvalLog::GetNumCompletedRuns() const {
	int num = 0;
	for (int i = 0; i < num_of_completed_runs.size(); i++)
		num += num_of_completed_runs[i];
	return num;
}

int EvalLog::GetNumRemainingRuns() const {
	return 80 * 30 - GetNumCompletedRuns();
}

int EvalLog::GetNumCompletedRuns(string instance) const {
	for (int i = 0; i < runned_instances.size(); i++) {
		if (runned_instances[i] == instance)
			return num_of_completed_runs[i];
	}
	return 0;
}

int EvalLog::GetNumRemainingRuns(string instance) const {
	return 30 - GetNumCompletedRuns(instance);
}

double EvalLog::GetUsedTimeInSeconds() const {
	time_t curr;
	time(&curr);
	return (double) (curr - start_time);
}

double EvalLog::GetRemainingTimeInSeconds() const {
	return 24 * 3600 - GetUsedTimeInSeconds();
}

// Pre-condition: curr_inst_start_time is initialized
void EvalLog::SetInitialBudget(string instance) {
	curr_inst_budget = 0;
	if (GetNumRemainingRuns() != 0 && GetNumRemainingRuns(instance) != 0) {
		cout << "Num of remaining runs: curr / total = "
			<< GetNumRemainingRuns(instance) << " / " << GetNumRemainingRuns()
			<< endl;
		curr_inst_budget = (24 * 3600 - (curr_inst_start_time - start_time))
			/ GetNumRemainingRuns() * GetNumRemainingRuns(instance);
		if (curr_inst_budget < 0)
			curr_inst_budget = 0;
		if (curr_inst_budget > 18 * 60)
			curr_inst_budget = 18 * 60;
	}
}

double EvalLog::GetRemainingBudget(string instance) const {
	return curr_inst_budget
		- (get_time_second() - EvalLog::curr_inst_start_time);
}

/* =============================================================================
 * Evaluator class
 * =============================================================================*/

Evaluator::Evaluator(DSPOMDP* model, string belief_type, Solver* solver,
	clock_t start_clockt, ostream* out) :
	model_(model),
	belief_type_(belief_type),
	solver_(solver),
	start_clockt_(start_clockt),
	target_finish_time_(-1),
	out_(out) {
}

Evaluator::~Evaluator() {
}


bool Evaluator::RunStep(int step, int round,string &ss) {
	if (target_finish_time_ != -1 && get_time_second() > target_finish_time_) {
		if (!Globals::config.silence && out_)
			*out_ << "Exit. (Total time "
				<< (get_time_second() - EvalLog::curr_inst_start_time)
				<< "s exceeded time limit of "
				<< (target_finish_time_ - EvalLog::curr_inst_start_time) << "s)"
				<< endl
				<< "Total time: Real / CPU = "
				<< (get_time_second() - EvalLog::curr_inst_start_time) << " / "
				<< (double(clock() - start_clockt_) / CLOCKS_PER_SEC) << "s"
				<< endl;
		exit(1);
	}


	stringstream st;


	double step_start_t = get_time_second();

	double start_t = get_time_second();

	int action = solver_->Search().action;

	double end_t = get_time_second();
	logi << "[RunStep] Time spent in " << typeid(*solver_).name()
		<< "::Search(): " << (end_t - start_t) << endl;

	double reward;
	OBS_TYPE obs;
	OBS_TYPE obs2;

	double secondary_probability;

	start_t = get_time_second();

	bool terminal = ExecuteAction(action, reward, obs);


	ros::NodeHandle nh;
	stringstream action_stream;
	stringstream state_stream;



	end_t = get_time_second();
	logi << "[RunStep] Time spent in ExecuteAction(): " << (end_t - start_t)
		<< endl;

	start_t = get_time_second();
	*out_ << "-----------------------------------Round " << round
				<< " Step " << step << "-----------------------------------"
				<< endl;
	st << "-----------------------------------Round " << round
				<< " Step " << step << "-----------------------------------"
				<< endl;


	if (!Globals::config.silence && out_) {
		*out_ << "- Action = ";
		model_->PrintAction(action, *out_);

		st << "- Action = ";
		model_->PrintAction(action, st);

		model_->PrintAction(action,action_stream);
	}

	 /*************************************
     *
     *			Moving	the  arm
     *
     ************************************/


	if (state_ != NULL) {
		if (!Globals::config.silence && out_) {
			*out_ << "- State:";
			model_->PrintState(*state_, *out_);

			st << "- State:";
			model_->PrintState(*state_, st);
			model_->PrintState(*state_,state_stream);
		}
	}

	vector<string> action_tokens,state_tokens,temp_tokens;
	string action_temp=action_stream.str();
	string state_temp=state_stream.str();

	if(boost::contains(action_temp,"d_")==1)
	{
		cout<<"Decision action is present:"<<action_temp<<" moving the conveyor belt!"<<endl;

		ros::ServiceClient advanceClient=nh.serviceClient<conveyor_advance::advance>(CONVEYOR_ADVANCE_SERVICE);
		conveyor_advance::advance advanceSrv;

		/*if(advanceClient.call(advanceSrv))
		{
			cout<<"Conveyor advanced!"<<endl;
		}else
		{
			cout<<"Error in advancing the conveyor!"<<endl;
		}*/


		/*cout<<"Decision making!"<<endl;

		map<string,double> pdf=solver_->belief()->get_pdf();

		map<string,double>::iterator it;*/

		/*cout<<"Before!"<<endl;
		for(it=pdf.begin();it!=pdf.end();++it)
		{
			cout<<it->first<<" "<<it->second<<endl;
		}

		Belief *new_belief=solver_->belief();
		new_belief->propagate();

		solver_->belief(new_belief);

		pdf=solver_->belief()->get_pdf();

		/*cout<<"After!"<<endl;
		for(it=pdf.begin();it!=pdf.end();++it)
		{
			cout<<it->first<<" "<<it->second<<endl;
		}*/


		/*if(advanceClient.call(advanceSrv))
		{
			cout<<"Conveyor advanced!"<<endl;
		}else
		{
			cout<<"Error in advancing the conveyor!"<<endl;
		}*/
	}else
	{
		boost::split(action_tokens,action_temp,boost::is_any_of(":"));
		boost::split(state_tokens,state_temp,boost::is_any_of(","));
		boost::split(temp_tokens,state_tokens[0],boost::is_any_of(":"));

		string proc_position=temp_tokens[temp_tokens.size()-1];
		boost::replace_all(proc_position,"vp", "");

		ros::ServiceClient positionClient=nh.serviceClient<bulb_scanner::position>(POSITION_SERVICE);

		bulb_scanner::position position_srv;
		stringstream act;
		act<<action_tokens[0];
		int action;
		act>>action;
		position_srv.request.action.data=action;
		position_srv.request.current_position_label.data=proc_position;

		if(positionClient.call(position_srv))
		{
			cout<<"Positioning the arm!"<<endl;
		}else
		{
			cout<<"An error has occured while positioning the arm!"<<endl;
		}

		ros::ServiceClient classificationClient=nh.serviceClient<bulb_processor::classify_multiple>(CLASSIFICATION_SERVICE);
		bulb_processor::classify_multiple classification_srv;

		classification_srv.request.pose.data=proc_position;

		if(classificationClient.call(classification_srv))
		{
			//cout<<"Classification :"<<classification_srv.response.type.data<<endl;
			//obs=classification_srv.response.type.data;

			cout<<classification_srv.response.first_type.data<<endl;

			obs=classification_srv.response.first_type.data;
			obs2=classification_srv.response.second_type.data;
		  secondary_probability=classification_srv.response.second_prob.data;


		}else
		{
			cout<<"An error has occured while classification!"<<endl;
		}

	}






	if (!Globals::config.silence && out_) {
		*out_ << "- Observation = ";
		model_->PrintObs(*state_, obs, *out_);

		st << "- Observation = ";
		model_->PrintObs(*state_, obs, st);
	}

	if (state_ != NULL) {
		if (!Globals::config.silence && out_)
			*out_ << "- ObsProb = " << model_->ObsProb(obs, *state_, action)
				<< endl;

			st << "- ObsProb = " << model_->ObsProb(obs, *state_, action)
				<< endl;
	}


	/*********************Belief propagation***************************************/
	//cout<<"Chosen action is :"<<action<<endl;

	if(boost::contains(action_temp,"d_")==1)
	{
		cout<<"Decision making!"<<endl;

		map<string,double> pdf=solver_->belief()->get_pdf();
		map<string,double>::iterator it;

		/*cout<<"Before!"<<endl;
		for(it=pdf.begin();it!=pdf.end();++it)
		{
			cout<<it->first<<" "<<it->second<<endl;
		}*/
		
		Belief *new_belief=solver_->belief();
		//new_belief->propagate();
		new_belief->propagate_values();

		solver_->belief(new_belief);
		pdf=solver_->belief()->get_pdf();

		cout<<"After!"<<endl;
		for(it=pdf.begin();it!=pdf.end();++it)
		{
			cout<<it->first<<" "<<it->second<<endl;
		}
	}

	/******************************************************************************/


	st<<ReportStepReward();


	end_t = get_time_second();

	double step_end_t;
	if (terminal) {
		step_end_t = get_time_second();
		logi << "[RunStep] Time for step: actual / allocated = "
			<< (step_end_t - step_start_t) << " / " << EvalLog::allocated_time
			<< endl;
		if (!Globals::config.silence && out_)
			*out_ << endl;
		step_++;
		return true;
	}

	*out_<<endl;

	start_t = get_time_second();

	cout<<"Pre"<<endl;
	cout<<solver_->belief()->text()<<endl;

	st<<"Pre"<<endl;
	st<<solver_->belief()->text()<<endl;

	solver_->Update(action, obs);
	cout<<"Post"<<endl;
	cout<<solver_->belief()->text()<<endl;

	st<<"Post"<<endl;
	st<<solver_->belief()->text()<<endl;

	cout<<"Pre secondary!"<<endl;
	cout<<solver_->secondary_belief()->text()<<endl;
	solver_->Update_secondary(action,obs2,secondary_probability);

	cout<<"Post secondary!"<<endl;
	//cout<<solver_->secondary_belief()->text()<<endl;
	cout<<solver_->secondary_belief()->text()<<endl;

	end_t = get_time_second();
	logi << "[RunStep] Time spent in Update(): " << (end_t - start_t) << endl;

	step_++;

	ss=st.str();

	return false;
}

double Evaluator::AverageUndiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
	}
	return undiscounted_round_rewards_.size() > 0 ? (sum / undiscounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrUndiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < undiscounted_round_rewards_.size(); i++) {
		double reward = undiscounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = undiscounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}


double Evaluator::AverageDiscountedRoundReward() const {
	double sum = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
	}
	return discounted_round_rewards_.size() > 0 ? (sum / discounted_round_rewards_.size()) : 0.0;
}

double Evaluator::StderrDiscountedRoundReward() const {
	double sum = 0, sum2 = 0;
	for (int i = 0; i < discounted_round_rewards_.size(); i++) {
		double reward = discounted_round_rewards_[i];
		sum += reward;
		sum2 += reward * reward;
	}
	int n = discounted_round_rewards_.size();
	return n > 0 ? sqrt(sum2 / n / n - sum * sum / n / n / n) : 0.0;
}

string Evaluator::ReportStepReward() {

	if (!Globals::config.silence && out_)
		*out_ << "- Reward = " << reward_ << endl
			<< "- Current rewards:" << endl
			<< "  discounted / undiscounted = " << total_discounted_reward_
			<< " / " << total_undiscounted_reward_ << endl;

	stringstream ss;

	ss << "- Reward = " << reward_ << endl
			<< "- Current rewards:" << endl
			<< "  discounted / undiscounted = " << total_discounted_reward_
			<< " / " << total_undiscounted_reward_ << endl;

	return ss.str();

}

/* =============================================================================
 * POMDPEvaluator class
 * =============================================================================*/

POMDPEvaluator::POMDPEvaluator(DSPOMDP* model, string belief_type,
	Solver* solver, clock_t start_clockt, ostream* out,
	double target_finish_time, int num_steps) :
	Evaluator(model, belief_type, solver, start_clockt, out),
	random_((unsigned) 0) {
	target_finish_time_ = target_finish_time;

	if (target_finish_time_ != -1) {
		EvalLog::allocated_time = (target_finish_time_ - get_time_second())
			/ num_steps;
		Globals::config.time_per_move = EvalLog::allocated_time;
		EvalLog::curr_inst_remaining_steps = num_steps;
	}
}

POMDPEvaluator::~POMDPEvaluator() {
}

int POMDPEvaluator::Handshake(string instance) {
	return -1; // Not to be used
}

void POMDPEvaluator::InitRound() {
	step_ = 0;

	double start_t, end_t;
	// Initial state
	state_ = model_->CreateStartState();
	logi << "[POMDPEvaluator::InitRound] Created start state." << endl;
	if (!Globals::config.silence && out_) {
		*out_ << "Initial state: ";
		model_->PrintState(*state_, *out_);
		*out_ << endl;
	}

	ros::NodeHandle nh;
	ros::ServiceClient positionClient=nh.serviceClient<bulb_scanner::position>(POSITION_SERVICE);

	bulb_scanner::position position_srv;
	position_srv.request.action.data=-1;
	position_srv.request.current_position_label.data="15";

	if(positionClient.call(position_srv))
	{
		cout<<"Positioning the arm!"<<endl;
	}else
	{
		cout<<"An error has occured while positioning the arm!"<<endl;
	}

	// Initial belief
	start_t = get_time_second();
	delete solver_->belief();
	//delete solver_->secondary_belief();

	end_t = get_time_second();
	logi << "[POMDPEvaluator::InitRound] Deleted old belief in "
		<< (end_t - start_t) << "s" << endl;

	start_t = get_time_second();
	Belief* belief = model_->InitialBelief(state_, belief_type_);
	Belief* secondary=model_->InitialBelief(state_,belief_type_);

	end_t = get_time_second();
	logi << "[POMDPEvaluator::InitRound] Created intial belief "
		<< typeid(*belief).name() << " in " << (end_t - start_t) << "s" << endl;

	solver_->belief(belief);
	solver_->belief_secondary(secondary);

	total_discounted_reward_ = 0;
	total_undiscounted_reward_ = 0;
}

double POMDPEvaluator::EndRound() {

	if (!Globals::config.silence && out_) {
		*out_ << "Total discounted reward = " << total_discounted_reward_ << endl
			<< "Total undiscounted reward = " << total_undiscounted_reward_ << endl;
	}

	discounted_round_rewards_.push_back(total_discounted_reward_);
	undiscounted_round_rewards_.push_back(total_undiscounted_reward_);

	return total_undiscounted_reward_;
}

bool POMDPEvaluator::ExecuteAction(int action, double& reward, OBS_TYPE& obs) {
	double random_num = random_.NextDouble();

	bool terminal = model_->Step(*state_, random_num, action, reward, obs);

	reward_ = reward;
	total_discounted_reward_ += Globals::Discount(step_) * reward;
	total_undiscounted_reward_ += reward;

	return terminal;
}

double POMDPEvaluator::End() {
	return 0; // Not to be used
}

void POMDPEvaluator::UpdateTimePerMove(double step_time) {
	if (target_finish_time_ != -1) {
		if (step_time < 0.99 * EvalLog::allocated_time) {
			if (EvalLog::plan_time_ratio < 1.0)
				EvalLog::plan_time_ratio += 0.01;
			if (EvalLog::plan_time_ratio > 1.0)
				EvalLog::plan_time_ratio = 1.0;
		} else if (step_time > EvalLog::allocated_time) {
			double delta = (step_time - EvalLog::allocated_time)
				/ (EvalLog::allocated_time + 1E-6);
			if (delta < 0.02)
				delta = 0.02; // Minimum reduction per step
			if (delta > 0.05)
				delta = 0.05; // Maximum reduction per step
			EvalLog::plan_time_ratio -= delta;
			// if (EvalLog::plan_time_ratio < 0)
			// EvalLog::plan_time_ratio = 0;
		}

		EvalLog::curr_inst_remaining_budget = target_finish_time_
			- get_time_second();
		EvalLog::curr_inst_remaining_steps--;

		if (EvalLog::curr_inst_remaining_steps <= 0) {
			EvalLog::allocated_time = 0;
		} else {
			EvalLog::allocated_time =
				(EvalLog::curr_inst_remaining_budget - 2.0)
					/ EvalLog::curr_inst_remaining_steps;

			if (EvalLog::allocated_time > 5.0)
				EvalLog::allocated_time = 5.0;
		}

		Globals::config.time_per_move = EvalLog::plan_time_ratio
			* EvalLog::allocated_time;
	}
}

} // namespace despot
