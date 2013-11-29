#ifndef FACADE_GRAMMAR_H
#define FACADE_GRAMMAR_H

#include "rule_define.h"
#include <map>

//! This class describe the shape grammar of the facade.
/*!
The grammar is stored in a xml file. We should use this class to load the grammar.
*/
class FacadeGrammar
{
public:
	FacadeGrammar();
	~FacadeGrammar();

	//! Load the facade grammar from the xml file.
	/*!
	\param configure_file The file name of the xml file.
	*/
	bool ReadGrammar ( std::string& configure_file );

	//! Save the facade grammar to the xml file.
	/*!
	\param configure_file The file name of the xml file.
	*/
	void SaveGrammar ( std::string& configure_file );

	//! Print the facade grammar to the console.
	void PrintGrammars();

	//! Get the number of terminal shape symbols.
	inline int get_terminal_symbol_size() { return terminal_symbol_.size(); }

	//! Get the name of the atom symbol.
	inline std::string get_atom_symbol() { return atom_symbol_; }

	//! Get the names of all the terminal symbols.
	inline std::vector<std::string> get_terminal_symbols() { return terminal_symbol_; }

	//! Get the name of a terminal symbol by index.
	inline std::string get_terminal_symbol(int index) { return terminal_symbol_[index]; }

	//! Get the names of all the unterminal symbols.
	inline std::vector<std::string> get_unterminal_symbols() { return unterminal_symbol_; }

	//! Get the number of all the unterminal symbols.
	inline int get_unterminal_symbol_size() { return unterminal_symbol_.size(); }

	//! Get the name of a unterminal symbol by index.
	inline std::string get_unterminal_symbol(int index) { return unterminal_symbol_[index]; }

	//! Get the number of the grammar rules.
	inline int get_grammar_number() { return grammar_names_.size(); }

	//! Get a grammar by index.
	inline ShapeGrammar get_grammar(int index) { return grammar_set_[index];}

	//! Get a symbol's split direction.
	inline int get_symbol_apply_direction(std::string symbol) { return symbol_apply_direction_.at(symbol); }

	//! Get the length of the symbol apply direction map.
	inline int get_symbol_direction_map_size() { return symbol_apply_direction_.size(); }
	
	//! Add a state-actions pair.
	inline void push_back_suitable_actions(std::vector<int> action) {suitable_actions_index_.push_back(action); }

	//! Get a symbol's color.
	std::vector<int> get_symbol_color(std::string symbol);

	//! Get all the actions that is suitable to apply to a state(symbol).
	std::vector<int> get_suitable_action(const int state) { return suitable_actions_index_[state]; }
private:
	// facade grammar
	std::string atom_symbol_; /*!< name of the atom symbol. */
	std::vector<std::string> terminal_symbol_;		/*!< Name of the terminal symbol. */
	std::vector<std::string> unterminal_symbol_;	/*!< Name of the unterminal symbol.*/

	std::vector<std::string> grammar_names_;		/*!< Name of all the grammar rules. */
	std::vector<ShapeGrammar> grammar_set_;			/*!< Grammar rule set. */
	std::map<std::string, int> symbol_apply_direction_;		/*!< Symbol-actions map. */
	std::vector<std::vector<int> > symbol_colors_;			/*!< The colors of all the terminal symbols. */
	std::map<std::string, int> symbol_color_map_;			/*!< symbol-color map. */
	std::vector<std::vector<int> > suitable_actions_index_; /*!< symbol-suitable actions map. */
};


#endif
