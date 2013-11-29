#ifndef RULE_DEFINE_H
#define RULE_DEFINE_H

#include <vector>
#include <string>

//! Shape grammar rule define.
/*! 
 * It contains all the elements that a grammar rule has.
 * A rule is of this format: 
 * [left_hand_symbol parameter, apply_direction] --> [right_hand_symbol1_ + right_hand_symbol2_] or [exception_symbol].
 */
struct ShapeGrammar {
	std::string grammar_name_;				/*!< Rule name. */
	int apply_direction_;					/*!< 0 -- vertical rule; 1 -- horizonal rule. */
	std::string left_hand_symbol_;			/*!< Left hand symbol. */
	std::string right_hand_symbol1_;		/*!< First right hand symbol. */
	std::string right_hand_symbol2_;		/*!< Second right hand symbol. */
	std::string exception_symbol;			/*!< Exception symbol. */
	double parameter_min, parameter_max;	/*!< Parameter range. */
};


#endif