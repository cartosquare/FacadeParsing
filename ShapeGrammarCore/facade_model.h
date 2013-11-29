#ifndef FACADE_H
#define FACADE_H

#include "grammar_elements.h"
#include "facade_geography.h"
#include "facade_grammar.h"
#include "rl_algorithm.h"

#include "opencv2/opencv.hpp"
#include "pcl\point_types.h"

#include <map>
#include <stack>

typedef SplitAction Facadestate;

//! This is the agent in RL
/*!
This class is used to parsing facade.
*/
class FacadeModel : public FacadeGeography, public RLAlgorithm
{
public:
	//! Constructor 
	/*!
	\param widthX Facade width.
	\param widthY Facade Height.
	\param facade_grammar Facade grammar.
	*/
	FacadeModel(int widthX, int widthY, std::shared_ptr<FacadeGrammar> facade_grammar);

	//! Constructor 
	/*!
	\param res Facade resolution, stands for the meters of one pixel(grid).
	\param extMin The left-bottom corner of the facade.
	\param extRange The extent of the facade.
	\param facade_grammar Facade grammar.
	*/
	FacadeModel (std::shared_ptr<FacadeGrammar> facade_grammar);

	//! Initialize the model.
	/*!
	Called in the constructor.
	*/
	void InitModel();

	~FacadeModel();

	//! Run the parsing algorithm.
	void RunParsingAlgorithm(int episodes);

	//! Calculate the symbol score.
	double CalculateScore (std::shared_ptr<ShapeSymbol> shape_symbol);

	//! Render the parsing result.
	void RenderResult(std::string& filename);
	
	void ConfigureOneDimensionFacade(std::string path);
	void ConfigureComplexOneDimensioFacade(std::string path);
	void ConfigureTwoDimensionFacade(std::string path);

	//! Print Q table to console.
	void PrintQTable();

	//! Save easy to read Q table to file.
	void SaveEasyReadQTable();
	
	void SaveLearningResult(std::string path);
	// inline functions
	inline std::vector<std::shared_ptr<ShapeSymbol> > get_split_result() { return split_result_; }
	inline int get_states_number() { return states_number_; }
	inline int get_actions_number() { return actions_number_; }

	inline std::vector<int> get_suitable_actions(const int state) { return facade_grammar_->get_suitable_action(state); }
	inline std::vector<int> get_symbol_color(std::string symbol) { return facade_grammar_->get_symbol_color(symbol); }

	inline bool current_state_is_terminal() { return current_state_->is_terminal(); }

	inline void set_action_parameter_file(std::string file) { action_parameter_file_ = file; }
protected:
	//! Reset the model.
	void Reset();

	//! Run a learning sub-task.
	double RunTask(std::shared_ptr<ShapeSymbol>& state, int state_index);

	//! Initialize the state-index and index-state map.
	int InitStateMap();

	//! Initialize the action-index and index-action map.
	int InitActionMap();
	
	int InitActionMapFromFile();

	//! Initialize the index-suitable actions map.
	void InitSymbolSuitableActions();


	
	//double RunGreedyTask(std::shared_ptr<ShapeSymbol>& state, int state_index);

	// no longer used ... 
	//double Act(const int action );
	//int GetState() ;
	//void SetState (const int state);
	//bool EndOfEpisode() ;

private:
	int states_number_, actions_number_;		/*!< Number of state and action. */
//	int state_ , next_state_, action_;
	int episodes_;								/*!< Number of episodes. */
	bool learning_, record_;					/*!< Whether to learning, record result. */
	double epsilon_, learning_rate_, gamma_;	/*!< learing parameters. */
	
	std::map<SplitAction, int> action_to_index_;	/*!< action-index map. */
	std::map<int, SplitAction> index_to_action_;	/*!< index-action map. */
	std::map<Facadestate, int> state_to_index_;		/*!< state-index map. */
	std::map<int, Facadestate> index_to_state_;		/*!< index-state map. */

	std::shared_ptr<ShapeSymbol> current_state_;	/*!< current state. */
	std::vector<std::shared_ptr<ShapeSymbol> >  split_result_;	/*!< parsing result. */

	std::string action_parameter_file_;
	//std::stack<std::shared_ptr<ShapeSymbol> > internal_symbols_;
	//std::stack<std::shared_ptr<ShapeSymbol> > sub_internal_symbols_;
};

#endif // FACADE