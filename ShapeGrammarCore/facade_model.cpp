#include "facade_model.h"
#include "scale_model.h"
#include <fstream>
#include <ctime>

//#define _TRACE_LOG
//#define _PRINT_QTABLE
#define  _PRINT_RESULT

FacadeModel::FacadeModel ( int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar): 
	FacadeGeography(widthX, widthY, facade_grammar)
{
	InitModel();
}

FacadeModel::FacadeModel (std::shared_ptr<FacadeGrammar> facade_grammar) : FacadeGeography (facade_grammar)
{
}

void FacadeModel::InitModel()
{
	// Initial state, there are two kinds of states:
//	state_ = 0 ; 	// @state_ is for learning algorithm, since learning algorithm only need a number

	// @current_state_ contains very detailed state information
	current_state_ = std::shared_ptr<ShapeSymbol>(new ShapeSymbol(facade_grammar_));
	current_state_->set_position_x (0);
	current_state_->set_position_y (0);
	current_state_->set_width (width_);
	current_state_->set_height (height_);

	current_state_->set_symbol_name (facade_grammar_->get_atom_symbol());

	// Init state and actions discrete
	states_number_  = InitStateMap();
	//actions_number_ = InitActionMap();
	actions_number_ = InitActionMapFromFile();
	InitSymbolSuitableActions();

	RLAlgorithm::Init(states_number_, actions_number_);

	epsilon_ = 0.9;
	learning_rate_ = 1.0;
	gamma_ = 0.1;
	episodes_ = 50000;
}

FacadeModel::~FacadeModel()
{

}

int FacadeModel::InitActionMapFromFile() {
	std::ifstream fin(action_parameter_file_.c_str());
	std::string dummy;
	int size;
	fin >> dummy >> size;
	for (int i = 0; i < size; ++i)
	{
		int parameter;
		fin >> parameter;
		vertical_wall_parameters.push_back(parameter);
	}
	fin >> dummy >> size;
	for (int i = 0; i < size; ++i)
	{
		int parameter;
		fin >> parameter;
		vertical_window_parameters.push_back(parameter);
	}
	fin >> dummy >> size;
	for (int i = 0; i < size; ++i)
	{
		int parameter;
		fin >> parameter;
		horizontal_wall_parameters.push_back(parameter);
	}
	fin >> dummy >> size;
	for (int i = 0; i < size; ++i)
	{
		int parameter;
		fin >> parameter;
		horizontal_window_parameters.push_back(parameter);
	}

	int count = 0;
	for (int i = 0; i < facade_grammar_->get_grammar_number(); ++i) 
	{
		ShapeGrammar sg = facade_grammar_->get_grammar(i);

		if (sg.apply_direction_ == 0) // vertical action
		{
			if (sg.right_hand_symbol1_ == "Wall")
			{
				for (int k = 0; k < vertical_wall_parameters.size(); ++k)
				{
					SplitAction sa (sg.grammar_name_, vertical_wall_parameters[k], i);

					std::pair<SplitAction, int> temp_pair;
					temp_pair.first = sa;
					temp_pair.second = count;
					action_to_index_.insert(temp_pair);

					std::pair<int, SplitAction> temp_pair_opp;
					temp_pair_opp.first = count;
					temp_pair_opp.second = sa;
					index_to_action_.insert(temp_pair_opp);

					count++;
				}
				
			}
			else // FloorWall(the height is equal to window)
			{
				for (int k = 0; k < vertical_window_parameters.size(); ++k)
				{
					SplitAction sa (sg.grammar_name_, vertical_window_parameters[k], i);

					std::pair<SplitAction, int> temp_pair;
					temp_pair.first = sa;
					temp_pair.second = count;
					action_to_index_.insert(temp_pair);

					std::pair<int, SplitAction> temp_pair_opp;
					temp_pair_opp.first = count;
					temp_pair_opp.second = sa;
					index_to_action_.insert(temp_pair_opp);

					count++;
				}
			}
		}
		else // horizontal action
		{
			if (sg.right_hand_symbol1_ == "Wall")
			{
				for (int k = 0; k < horizontal_wall_parameters.size(); ++k)
				{
					SplitAction sa (sg.grammar_name_, horizontal_wall_parameters[k], i);

					std::pair<SplitAction, int> temp_pair;
					temp_pair.first = sa;
					temp_pair.second = count;
					action_to_index_.insert(temp_pair);

					std::pair<int, SplitAction> temp_pair_opp;
					temp_pair_opp.first = count;
					temp_pair_opp.second = sa;
					index_to_action_.insert(temp_pair_opp);

					count++;
				}
			}
			else
			{
				for (int k = 0; k < horizontal_window_parameters.size(); ++k)
				{
					SplitAction sa (sg.grammar_name_, horizontal_window_parameters[k], i);

					std::pair<SplitAction, int> temp_pair;
					temp_pair.first = sa;
					temp_pair.second = count;
					action_to_index_.insert(temp_pair);

					std::pair<int, SplitAction> temp_pair_opp;
					temp_pair_opp.first = count;
					temp_pair_opp.second = sa;
					index_to_action_.insert(temp_pair_opp);

					count++;
				}
			}
		}		
	}	

	return count;
}

int FacadeModel::InitActionMap()
{
	int count = 0;
	for (int i = 0; i < facade_grammar_->get_grammar_number(); ++i)
	{
		ShapeGrammar sg = facade_grammar_->get_grammar(i);

#if 0
		// And the "empty" action
		SplitAction sa(sg.grammar_name_, 0, i);

		std::pair<SplitAction, int> temp_pair;
		temp_pair.first = sa;
		temp_pair.second = count;
		action_to_index_.insert (temp_pair);

		std::pair<int, SplitAction> temp_pair_opp;
		temp_pair_opp.first = count;
		temp_pair_opp.second = sa;
		index_to_action_.insert (temp_pair_opp);
		count++;
#endif

		for ( int j = static_cast<int>(sg.parameter_min); j <= static_cast<int>(sg.parameter_max); ++j )
		{
			SplitAction sa (sg.grammar_name_, j, i);

			std::pair<SplitAction, int> temp_pair;
			temp_pair.first = sa;
			temp_pair.second = count;
			action_to_index_.insert ( temp_pair );

			std::pair<int, SplitAction> temp_pair_opp;
			temp_pair_opp.first = count;
			temp_pair_opp.second = sa;
			index_to_action_.insert ( temp_pair_opp );

			count++;
		}
	}
#ifdef _TRACE_LOG
	std::ofstream ofs ( "..\Data\action_index.txt" );
	for ( int i = 0; i < count; ++i )
	{
		SplitAction sa = index_to_action_.at ( i );
		ofs << i << ": " << sa.get_grammar_name() << "\t" <<
			sa.get_parameter() << "\n";
	}
	ofs.close();

#endif
	return count;
}

int FacadeModel::InitStateMap()
{
	int dim = width_ > height_ ? width_ : height_;
	int count = 0;

	// atom state
	Facadestate fs (facade_grammar_->get_atom_symbol(), 0);
	std::pair<Facadestate, int> temp_pair;
	temp_pair.first = fs;
	temp_pair.second = count;
	state_to_index_.insert (temp_pair);

	std::pair<int, Facadestate> temp_pair_opp;
	temp_pair_opp.first = count;
	temp_pair_opp.second = fs;
	index_to_state_.insert (temp_pair_opp);

	count++;
	for (int pos = 0; pos < dim; ++pos)		// position along the split direction
	{
		// unterminal_symbol_
		for (int shape_id = 0; shape_id < facade_grammar_->get_unterminal_symbol_size(); ++shape_id)
		{
			Facadestate fs (facade_grammar_->get_unterminal_symbol(shape_id), pos);
			std::pair<Facadestate, int> temp_pair;
			temp_pair.first = fs;
			temp_pair.second = count;
			state_to_index_.insert (temp_pair);

			std::pair<int, Facadestate> temp_pair_opp;
			temp_pair_opp.first = count;
			temp_pair_opp.second = fs;
			index_to_state_.insert (temp_pair_opp);

			count++;
		}

		// terminal_symbol_
		for (int shape_id = 0; shape_id < facade_grammar_->get_terminal_symbol_size(); ++shape_id)
		{
			Facadestate fs (facade_grammar_->get_terminal_symbol(shape_id), pos);
			std::pair<Facadestate, int> temp_pair;
			temp_pair.first = fs;
			temp_pair.second = count;
			state_to_index_.insert (temp_pair);

			std::pair<int, Facadestate> temp_pair_opp;
			temp_pair_opp.first = count;
			temp_pair_opp.second = fs;
			index_to_state_.insert (temp_pair_opp);

			count++;
		}
	}
#ifdef _TRACE_LOG
	std::ofstream ofs ( "..\Data\state_index.txt" );
	for ( int i = 0; i < count; ++i )
	{
		Facadestate sa = index_to_state_.at ( i );
		ofs << i << ": " << sa.get_grammar_name() << "\t" <<
			sa.get_parameter() << "\n";
	}
	ofs.close();

#endif
	return count;
}

void FacadeModel::InitSymbolSuitableActions()
{
	for (int i = 0; i < this->states_number_; ++i)
	{
		std::vector<int> actions;
		// get the state
		Facadestate fs = index_to_state_.at (i);
		for (int j = 0; j < this->actions_number_; ++j)
		{
			// get the action
			SplitAction sa = index_to_action_.at (j);
			// get the corresponding shape grammar
			for ( int k = 0; k < facade_grammar_->get_grammar_number(); ++k )
			{
				if (sa.get_grammar_name() == facade_grammar_->get_grammar(k).grammar_name_)
				{
					sa.set_shape_grammar(k);
					break;
				}
			}
			// If the state symbol is the left hand symbol of the action,
			// then the action is suitable
			ShapeGrammar shape_grammar = facade_grammar_->get_grammar(sa.get_shape_grammar_index());
			if ( fs.get_grammar_name() == shape_grammar.left_hand_symbol_)
			{
#if 0
				if ((shape_grammar.apply_direction_ == 0) && (fs.get_parameter() + sa.get_parameter() < height_))
				{
					actions.push_back ( j );
				}
				else if ((shape_grammar.apply_direction_ == 1) && (fs.get_parameter() + sa.get_parameter() < width_))
				{
					actions.push_back ( j );
				}
#endif // 0
				actions.push_back ( j );	
			}
		}
		facade_grammar_->push_back_suitable_actions(actions);
	}

#ifdef _TRACE_LOG
	std::ofstream ofs ( "..\Data\suitable_action_index.txt" );
	for ( int i = 0; i < states_number_; ++i)
	{
		std::vector<int> actions = facade_grammar_->get_suitable_action(i);
		ofs << i << ": ";
		for (size_t j = 0; j < actions.size(); ++j)
		{
			ofs << actions[j] << "\t";
		}
		ofs << std::endl;
	}
	ofs.close();

#endif
}

void FacadeModel::RunParsingAlgorithm(int episodes) {
	episodes_ = episodes;
	unsigned int rand_seed = static_cast<unsigned int>(std::time(0));
	std::srand(rand_seed);
	learning_ = true;
	record_ = false;
	int total_pixel_number;

	// Learning @episodes_ times ... 
	for (int i = 0; i < episodes_; ++i)
	{
		// Calculate the parameters
		double seed = (double)i / (double)episodes_;
		epsilon_ = ScaleModel::ExpTrans(seed, 0.01, 0.9);
		gamma_ = 1 - epsilon_;

		double score = 0.0;
		// Reset the environment(recover the parsing original)
		Reset();
		total_pixel_number = current_state_->width() * current_state_->height();

		// Parsing from root state
		score = RunTask(current_state_, 0);

		if (i % 100 == 0)
		{
			std::cout << "\nEpisode " << i << " Score: " << score / total_pixel_number << 
				" epsilon: " << epsilon_ << " gamma: " << gamma_ /*<< " Seed: " << rand_seed*/ << std::endl;

			unsigned int rand_seed = static_cast<unsigned int>(std::time(0));
			std::srand(rand_seed);
		}
	}

#ifdef _TRACE_LOG
	SaveEasyReadQTable();
#endif
	
	// Run some episodes with big greedy ...
	epsilon_ = 0.9;
	gamma_ = 0.1;
	episodes_ = 10000;
	for (int i = 0; i < episodes_; ++i)
	{

		double score = 0.0;
		// Reset the environment(recover the parsing original)
		Reset();
		total_pixel_number = current_state_->width() * current_state_->height();

		// Parsing from root state
		score = RunTask(current_state_, 0);

		if (i % 100 == 0)
		{
			std::cout << "\nEpisode " << i << " Score: " << score / total_pixel_number << 
				" epsilon: " << epsilon_ << " gamma: " << gamma_ /*<< " Seed: " << rand_seed*/ << std::endl;

			unsigned int rand_seed = static_cast<unsigned int>(std::time(0));
			std::srand(rand_seed);
		}
	}
	
	// Get the greedy solution
	Reset();
	epsilon_ = 1.0;
	learning_ = false;
	record_ = true;
	double score = RunTask(current_state_, 0);
	std::cout << "\nGreedy Score: " << score / total_pixel_number << std::endl;

	// Save the learning result.
	this->SaveQTable();
}

double FacadeModel::RunTask(std::shared_ptr<ShapeSymbol>& state, int state_index) {
	double episode_score = 0.0;
	int action, next_state;

	while (!state->is_terminal())
	{	
		double score = 0.0;
		bool end_of_task = false;

		// Explore an action for current state
		Explore(state_index, action, epsilon_, get_suitable_actions(state_index)); 

		// Get the very detailed action
		SplitAction split_action = index_to_action_.at(action);

#ifdef _TRACE_LOG
		std::cout << "\n\tState: " << state->symbol_name() << "\t[" << state->position_x() << 
			", " << state->position_y() << ", " << state->width() << ", " << state->height() << "]\n";
		std::cout << "\tAction: " << split_action.get_grammar_name() << ", " << split_action.get_parameter() << std::endl;
		std::cout << "\tDo Split ...\n";
#endif
		std::shared_ptr<ShapeSymbol> right_symbol1(new ShapeSymbol(facade_grammar_));
		std::shared_ptr<ShapeSymbol> right_symbol2(new ShapeSymbol(facade_grammar_));
		bool split_result = state->DoSplit(split_action, right_symbol1, right_symbol2, width_, height_);
		int pixel_number = right_symbol1->height() * right_symbol1->width();
		if (split_result)
		{
			if (right_symbol1->is_terminal())
			{
				score += CalculateScore(right_symbol1);
				if (record_)
				{
					split_result_.push_back(right_symbol1);
				}
				
#ifdef _TRACE_LOG
				std::cout << "\tRight symbol1: " << right_symbol1->symbol_name() << " is terminal, score is: " << 
					score / pixel_number << std::endl;
#endif
			}
			else
			{
#ifdef _TRACE_LOG
				std::cout << "\tRight symbol1: " << right_symbol1->symbol_name() << " is not terminal, go in sub-task ... \n";
#endif
				// Get the sub-state index
				int sub_state;
				int dir = facade_grammar_->get_symbol_apply_direction(right_symbol1->symbol_name());
				if (dir == 0) // vertical
				{
					Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_y());
					sub_state = state_to_index_.at(facade_state);
				}
				else
				{
					Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_x());
					sub_state = state_to_index_.at(facade_state);
				}
				
				score += RunTask(right_symbol1, sub_state);
#ifdef _TRACE_LOG
				std::cout << "\tRight symbol1's sub-task completed, the score is: " << score / pixel_number << std::endl;
#endif
			}

			// Get the state after executing the action
			int dir = facade_grammar_->get_symbol_apply_direction(right_symbol2->symbol_name());
			if (dir == 0) // vertical
			{
				Facadestate facade_state(right_symbol2->symbol_name(), right_symbol2->position_y());
				next_state = state_to_index_.at(facade_state);
			}
			else
			{
				Facadestate facade_state(right_symbol2->symbol_name(), right_symbol2->position_x());
				next_state = state_to_index_.at(facade_state);
			}

			state = right_symbol2;	
		}
		else
		{
			score += CalculateScore(right_symbol1);
			if (record_)
			{
				split_result_.push_back(right_symbol1);
			}

#ifdef _TRACE_LOG
			std::cout << "\tSplit fail, exception symbol is " << right_symbol1->symbol_name() << ", and score is: " <<
				score / pixel_number << std::endl;
#endif
			end_of_task = true;
			state = right_symbol1;
		}
		episode_score += score; // to return
		score /= pixel_number;  // for learning

		if (learning_)
		{
			// update state-action value pair
			// When @end_of_task == true, @next_state_ is not used.
			Update(state_index, action, score, next_state, end_of_task, learning_rate_, gamma_);
		}
		state_index = next_state;
		
#ifdef _TRACE_LOG
		std::cout << "\tAfter split, the current state is: " << state->symbol_name() << "\t[" << state->position_x() << 
			", " << state->position_y() << ", " << state->width() << ", " << state->height() << "]\n";
#endif // _TRACE_LOG

#ifdef _PRINT_QTABLE
		PrintQTable();
#endif // _PRINT_QTABLE
	}

	return episode_score;
}

/*
double FacadeModel::RunGreedyTask(std::shared_ptr<ShapeSymbol>& state, int state_index) {
	double episode_score = 0.0;
	int action, next_state;
	
	while (!state->is_terminal())
	{	
		double score = 0.0;
		bool end_of_task = false;

		// Explore an action for current state
		MaxExplore(state_index, action, get_suitable_actions(state_index)); 

		SplitAction split_action = index_to_action_.at (action);

		std::shared_ptr<ShapeSymbol> right_symbol1(new ShapeSymbol(facade_grammar_));
		std::shared_ptr<ShapeSymbol> right_symbol2(new ShapeSymbol(facade_grammar_));
		if (state->DoSplit(split_action, right_symbol1, right_symbol2, width_, height_))
		{
			if (right_symbol1->is_terminal())
			{
				score += CalculateScore(right_symbol1);
				split_result_.push_back(right_symbol1);
			}
			else
			{
				// Get the sub-state index
				int sub_state;			
				int dir = facade_grammar_->get_symbol_apply_direction(right_symbol1->symbol_name());
				if (dir == 0) // vertical
				{
					Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_y());
					sub_state = state_to_index_.at(facade_state);
				}
				else
				{
					Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_x());
					sub_state = state_to_index_.at(facade_state);
				}
				score += RunGreedyTask(right_symbol1, sub_state);
			}

			// Get the state after executing the action
			int dir = facade_grammar_->get_symbol_apply_direction(right_symbol2->symbol_name());
			if (dir == 0) // vertical
			{
				Facadestate facade_state(right_symbol2->symbol_name(), right_symbol2->position_y());
				next_state = state_to_index_.at(facade_state);
			}
			else
			{
				Facadestate facade_state(right_symbol2->symbol_name(), right_symbol2->position_x());
				next_state = state_to_index_.at(facade_state);
			}

			state_index = next_state;
			state = right_symbol2;
		}
		else
		{
			score += CalculateScore(right_symbol1);
			split_result_.push_back(right_symbol1);
			state = right_symbol1;
		}	
		episode_score += score;
	}

	return episode_score;
}
*/
#if 0
double FacadeModel::Act (const int action)
{
	double return_value = 0;

	// get the very detailed action
	SplitAction split_action = index_to_action_.at (action);

	// get the corresponding shape grammar(split rule)
	for (int i = 0; i < facade_grammar_->get_grammar_number(); ++i)
	{
		if (split_action.get_grammar_name() == facade_grammar_->get_grammar(i).grammar_name_)
		{
			split_action.set_shape_grammar(facade_grammar_->get_grammar(i));
			break;
		}
	}

#ifdef _TRACE_LOG
	std::cout << "Act:\n";
	std::cout << "\tState: " << current_state_->symbol_name() << "[" <<
		current_state_->position_x() << ", " << current_state_->position_y() << ", " <<
		current_state_->width() << ", " << current_state_->height() << "]\n";
	std::cout << "\tAction: " << split_action.get_shape_grammar().grammar_name_ << "\t" << split_action.get_parameter() << std::endl;
#endif


	std::shared_ptr<ShapeSymbol> right_symbol1(new ShapeSymbol(facade_grammar_));
	std::shared_ptr<ShapeSymbol> right_symbol2(new ShapeSymbol(facade_grammar_));
	if (current_state_->DoSplit(split_action, right_symbol1, right_symbol2))
	{
		if (right_symbol1->is_terminal())
		{
			// calculate scores
			score_ += CalculateScore(right_symbol1);
			split_result_.push_back(right_symbol1);
#ifdef _TRACE_LOG
			std::cout << "\tAct result: right symbol1 is terminal.\n";
			std::cout << "\tAnd the score is : " << score_ << std::endl;
#endif
		}
		else
		{
			int new_state;
			int dir = facade_grammar_->get_symbol_apply_direction(right_symbol1->symbol_name());
			if (dir == 0) // vertical
			{
				Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_y());
				new_state = state_to_index_.at (facade_state);
			}
			else
			{
				Facadestate facade_state(right_symbol1->symbol_name(), right_symbol1->position_x());
				new_state = state_to_index_.at (facade_state);
			}
			score_ += Act2(new_state, right_symbol1);
		}

		internal_symbols_.push(right_symbol2);

		current_state_ = internal_symbols_.top();
		internal_symbols_.pop();
		end_of_episode_ = false;
	}
	else
	{
		// calculate scores
		score_ += CalculateScore (right_symbol1);
		split_result_.push_back(right_symbol1);
#ifdef _TRACE_LOG
		std::cout << "\tAct result: Act fail.\n";
		std::cout << "\tAnd the score is : " << score_ << std::endl;
#endif
		if (internal_symbols_.empty())
		{
#ifdef _TRACE_LOG
			std::cout << "\nEND OF EPISODE.\n\n";
#endif
			end_of_episode_ = true;
			// return scores
			return_value = score_;
		}
		else
		{
			current_state_ = internal_symbols_.top();
			internal_symbols_.pop();
			end_of_episode_ = false;
		}
	}
	int dir = facade_grammar_->get_symbol_apply_direction(current_state_->symbol_name());
	if (dir == 0) // vertical
	{
		Facadestate facade_state (current_state_->symbol_name(), current_state_->position_y());
		state_ = state_to_index_.at (facade_state);
	}
	else
	{
		Facadestate facade_state (current_state_->symbol_name(), current_state_->position_x());
		state_ = state_to_index_.at (facade_state);
	}

	return return_value;
}
#endif
double FacadeModel::CalculateScore ( std::shared_ptr<ShapeSymbol> shape_symbol )
{
	double score = 0.0;
	int height_begin, height_end, width_begin, width_end;

	for (int i = 0; i < facade_grammar_->get_terminal_symbol_size(); ++i)
	{
		if (shape_symbol->symbol_name() == facade_grammar_->get_terminal_symbol(i))
		{
			height_begin = shape_symbol->position_y();
			height_end = height_begin + shape_symbol->height();
			for (int h = height_begin; h < height_end; ++h)
			{
				width_begin = shape_symbol->position_x();
				width_end = width_begin + shape_symbol->width();

				for (int w = width_begin; w < width_end; ++w)
				{
					// Version 1
					score += grd_[i][h][w];

					// Version 2
					/*if (grd_[i][h][w] == 0)
					{
						score -= 4;
					}
					else
					{
						score += grd_[i][h][w];
					}*/
					
				}
			}
			break;
		}
	}
//	score /= (shape_symbol->height() * shape_symbol->width());
//	score /= (width_ * height_);
	return score;

}

//bool FacadeModel::EndOfEpisode()
//{
//	return end_of_episode_;
//}
//
//int FacadeModel::GetState ()
//{
//	return state_;
//}

void FacadeModel::Reset()
{
	// Initial state, there are two kinds of states:
//	state_ = 0 ; 	// @state is for learning algorithm, since learning algorithm only need a number
//	next_state_ = 0;

	// @current_state_ contains very detailed state information
	//	current_state_->SetScope(0, 0, width_, height_);
	current_state_->set_position_x ( 0 );
	current_state_->set_position_y ( 0 );
	current_state_->set_width ( width_ );
	current_state_->set_height ( height_ );

	current_state_->set_symbol_name (facade_grammar_->get_atom_symbol());

	//  terminal_symbol_.clear();
	split_result_.clear();
	//while ( !internal_symbols_.empty() )
	//{
	//	internal_symbols_.pop();
	//}
}

//void FacadeModel::SetState (const int state )
//{
//	state_ = state;
//}


void FacadeModel::ConfigureOneDimensionFacade(std::string path) {
	for (int x  = 0; x < 4; ++x)
	{
		for (int y = 0; y < 5; ++y)
		{
			SetProperty(x, y, 1, 0.0); // WINDOW
			SetProperty(x, y, 0, 1.0); 	// WALL
		}	
	}
	for (int x  = 4; x < 7; ++x)
	{
		for (int y = 0; y < 5; ++y)
		{
			SetProperty(x, y, 1, 1.0);
			SetProperty(x, y, 0, 0.0);		
		}	
	}
	for (int x  = 7; x < 10; ++x)
	{
		for (int y = 0; y < 5; ++y)
		{
			SetProperty(x, y, 1, 0.0);
			SetProperty(x, y, 0, 1.0);		
		}	
	}
	RenderGrd(path);
}

void FacadeModel::ConfigureComplexOneDimensioFacade(std::string path) {
	int pos = 0;
	while (pos != 200)
	{
		SetSymbol(pos, pos + 10, 0, 100, 0); // wall
		pos += 10;
		SetSymbol(pos, pos + 10, 0, 100, 1); // window
		pos += 10;
	}
	SetSymbol(200, 210, 0, 100, 0); // wall

	RenderGrd(path);
}

void FacadeModel::ConfigureTwoDimensionFacade(std::string path)
{
	// Set Wall
	SetSymbol(0, 5, 5, 10, 0);
	SetSymbol(0, 5, 15, 20, 0);

	SetSymbol(10, 15, 5, 10, 0);
	SetSymbol(10, 15, 15, 20, 0);

	SetSymbol(20, 25, 5, 10, 0);
	SetSymbol(20, 25, 15, 20, 0);

	SetSymbol(0, 25, 0, 5, 0);
	SetSymbol(0, 25, 10, 15, 0);
	SetSymbol(0, 25, 20, 25, 0);

	// Set Window
	SetSymbol(5, 10, 5, 10, 1);
	SetSymbol(5, 10, 15, 20, 1);
	SetSymbol(15, 20, 5, 10, 1);
	SetSymbol(15, 20, 15, 20, 1);

	RenderGrd(path);
}

void FacadeModel::RenderResult (std::string& filename )
{
	cv::Mat facade_image = cv::Mat::zeros(height_, width_, CV_8UC3 );

#ifdef _PRINT_RESULT
	std::cout << "\n\nSplit Result: \n";
#endif // _PRINT_RESULT
	for (size_t i = 0; i < split_result_.size(); ++i)
	{
#ifdef _PRINT_RESULT
		std::cout << "[ " << split_result_[i]->symbol_name() << ", " <<
			split_result_[i]->position_x() << ", " << split_result_[i]->position_y() <<
			", " << split_result_[i]->width() << ", " << split_result_[i]->height() << "]\n";
#endif // _PRINT_RESULT

		cv::Point p1 (split_result_[i]->position_x(), split_result_[i]->position_y());
		cv::Point p2 (p1.x + split_result_[i]->width(), p1.y + split_result_[i]->height());
		std::vector<int> symbol_color = get_symbol_color(split_result_[i]->symbol_name());
		cv::Scalar color (symbol_color[0], symbol_color[1], symbol_color[2]);
		cv::rectangle (facade_image, p1, p2, color, -1);
	}
	cv::imwrite (filename, facade_image);
}

void FacadeModel::PrintQTable() {
	std::cout << "Q Table: \n";
	bool has_elements = false;
	for (int i = 0; i < states_number_; ++i)
	{
		for (int j = 0; j < actions_number_; ++j)
		{
			if (q_[i][j] != 0)
			{
				Facadestate state = index_to_state_[i];
				SplitAction action = index_to_action_[j];
				std::cout << "{[" << state.get_grammar_name() << ", " << state.get_parameter() <<
					"], [" << action.get_grammar_name() << ", " << action.get_parameter() << "], " <<
					q_[i][j] << "} ";
				has_elements = true;
			}
			if (has_elements)
			{
				std::cout << std::endl;
				has_elements = false;
			}	
		}
	}
	
}



void FacadeModel::SaveEasyReadQTable() {
	std::ofstream fout("easyReadQtable.txt");

	bool has_elements = false;
	for (int i = 0; i < states_number_; ++i)
	{
		for (int j = 0; j < actions_number_; ++j)
		{
			if (q_[i][j] != 0)
			{
				Facadestate state = index_to_state_[i];
				SplitAction action = index_to_action_[j];
				fout << "{[" << state.get_grammar_name() << ", " << state.get_parameter() <<
					"], [" << action.get_grammar_name() << ", " << action.get_parameter() << "], " <<
					q_[i][j] << "} ";
				has_elements = true;
			}
			if (has_elements)
			{
				fout << std::endl;
				has_elements = false;
			}	
		}
	}
	fout.close();
}


void FacadeModel::SaveLearningResult(std::string path) {
	std::ofstream fout(path.c_str());

	fout << split_result_.size() << std::endl;
	fout << extents_min_.x << " " << extents_min_.y << " "<< resolution_ << " " << width_ << " " <<
		height_ << std::endl;

	for (size_t i = 0; i < split_result_.size(); ++i)
	{
		fout << split_result_[i]->symbol_name() << " " <<
			split_result_[i]->position_x() << " " << split_result_[i]->position_y() <<
			" " << split_result_[i]->width() << " " << split_result_[i]->height() << "\n";
	}

	fout.close();
}
