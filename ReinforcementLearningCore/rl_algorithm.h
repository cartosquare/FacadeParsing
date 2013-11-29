#ifndef  RL_ALGORITHM_H
#define  RL_ALGORITHM_H

#include <string>
#include <math.h>
#include <vector>

//! This is the reinforcement learning algorithm
/*! 
This class is used to learn a q table.
*/
class RLAlgorithm {
public:
	//! Constructor 
	RLAlgorithm() {}

	//! Constructor 
	/*! 
	Initialize the algorithm with the number of states and actions.
	*/
	RLAlgorithm(int states_number, int actions_number);

	//! Constructor 
	/*! 
	Initialize the algorithm with the number of states and actions.
	*/
	void Init(int states_number, int actions_number);

	~RLAlgorithm();

	//! Given a state, explore an action with a exploration rate.
	/*! 
	\param state current state.
	\param action explored action.
	\param exploration_rate exploration rate.
	\param action_index suitable actions for current state.
	*/
	void Explore(const int state, int& action, const double exploration_rate, const std::vector<int>& action_index);

	//void MaxExplore(const int state, int& action, const std::vector<int>& action_index);

	//! Update the state-action value
	/*!
	\param state current state.
	\param action chosen action.
	\param reward reward executing chosen action under current state.
	\param next_state next state.
	\param end_of_episode whether the episode is end.
	\param learning_rate the learning rate.
	\param gamma learning parameter.
	*/
	void Update(const int state, const int action, const double reward, const int next_state, 
		const bool end_of_episode, const double learning_rate, const double gamma);

	//! Save the q table to a file.
	void SaveQTable();

	//! Load the q table from a file.
	void LoadQTable();

	//! Get the state-action value.
	inline double GetQValue(int state, int action) {  return q_[state][action]; }

	//! Set the qtable file
	inline void set_qtable_path(std::string path) { qtable_path = path; }

	//! Print q table.
	static void PrintQTable();

protected:
	//! Epsilon greedy algorithm used to explore an action under a state with a exploration rate.
	/*! 
	\param state current state.
	\param action explored action.
	\param epsilon exploration rate.
	\param action_index suitable actions for current state.
	*/
	void egreedy(const int state, int& action, const double epsilon, const std::vector<int>& action_index);


	void GetMaxAction(const int state, int& action, const std::vector<int>& action_index);
	void GetMaxActionFirst(const int state, int& action, const std::vector<int>& action_index);
	void GetMaxActionRandom(const int state, int& action, const std::vector<int>& action_index);
	void GetRandomAction(const int state, int& action, const std::vector<int>& action_index);

	double** q_;		/*!< Q table. */
	std::string qtable_path;					/*!< q table path. */
	//double* policy_ ;
	//double q_target_;	
	int state_number_, action_number_; /*< Number of state and action. */
};


#endif // ! RL_ALGORITHM_H