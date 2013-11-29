#include "rl_algorithm.h"
#include "rl_utility.h"
#include <fstream>
#include <iostream>

RLAlgorithm::RLAlgorithm(int states_number, int actions_number) {
	Init(states_number, actions_number);
}

void RLAlgorithm::Init(int states_number, int actions_number) {
#ifdef _TRACE_LOG
	std::cout << "Init q table: " << states_number << " * " << actions_number << std::endl;
#endif

	state_number_ =  states_number;
	action_number_ = actions_number;

	// q table stores state - action values
	q_ = new double*[state_number_];
	for (int i = 0; i < state_number_; ++i)
	{
		q_[i] = new double[actions_number];
		for (int j = 0; j < action_number_; ++j)
		{
			q_[i][j] = 0.0;
		}
	}

//	policy_ = new double[action_number_];
}

RLAlgorithm::~RLAlgorithm() {
}

void RLAlgorithm::Explore(const int state, int& action, 
						  const double exploration_rate, const std::vector<int>& action_index) {
	egreedy(state, action, exploration_rate, action_index);
}

//void RLAlgorithm::MaxExplore(const int state, int& action, const std::vector<int>& action_index) {
//	GetMaxActionRandom(state, action, action_index);
//}

void RLAlgorithm::egreedy(const int state, int& action, const double epsilon, const std::vector<int>& action_index) {
	if (RL::RLUtility::RandUnit() < epsilon) 
	{
		GetMaxAction(state, action, action_index);
	} 
	else 
	{
		GetRandomAction(state, action, action_index) ;
	}
}
void RLAlgorithm::GetMaxAction(const int state, int& action, const std::vector<int>& action_index) {
	GetMaxActionRandom(state, action, action_index);
}

void RLAlgorithm::GetMaxActionRandom(const int state, int& action, const std::vector<int>& action_index) {
	double* q_values = q_[state];
	std::vector<int> max_all = RL::RLUtility::ArgMaxAll(q_values, action_index);
	action = max_all[RL::RLUtility::Rand0A(max_all.size())];
}

void RLAlgorithm::GetMaxActionFirst(const int state, int& action, const std::vector<int>& action_index) {
	double* q_values = q_[state];
	action = RL::RLUtility::ArgMax(q_values, action_index);
}

void RLAlgorithm::GetRandomAction(const int state, int& action, const std::vector<int>& action_index) {
	int index = RL::RLUtility::Rand0A(action_index.size());
	action = action_index[index];
}

void RLAlgorithm::Update(const int state, const int action, const double reward, 
						 const int next_state, const bool end_of_episode, 
						 const double learning_rate, const double gamma ) {
	if (end_of_episode) 
	{
		q_[state][action] += learning_rate * (reward - q_[state][action]);
	}
	else 
	{
		double max_q_value = RL::RLUtility::Max(q_[next_state], action_number_) ;
		q_[state][action] += learning_rate * (reward + gamma * max_q_value - q_[state][action]);
	}
}

void RLAlgorithm::SaveQTable() {
	std::ofstream ofs(qtable_path.c_str(), std::ofstream::out) ;

	for ( int i = 0; i < state_number_; ++i)
	{
		for ( int j = 0; j < action_number_; ++j)
		{
			ofs << q_[i][j] << " ";
		}
		ofs << std::endl;
	}

	ofs.close();
}

void RLAlgorithm::LoadQTable() 
{
	std::ifstream ifs(qtable_path.c_str(), std::ifstream::in);

	double tmp;
	for ( int i = 0; i < state_number_; ++i)
	{
		for ( int j = 0; j < action_number_; ++j)
		{
			ifs >> tmp;
			q_[i][j] = tmp;
		}
	}
	ifs.close();
}


void RLAlgorithm::PrintQTable() {

}