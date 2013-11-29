#ifndef GRAMMAR_ELEMENTS_H
#define GRAMMAR_ELEMENTS_H
/*
 * Binary Grammar Symbol Representation
 *
 * Xiang Xu, August 18th, 2013
 *
 * Email: xiangxu@mail.bnu.edu.cn
 */

#include "rule_define.h"
#include <memory>

//! Split action
/*! This class is used by RL Learning. It is the brief describe of ShapeGrammar structure.*/
class SplitAction {
public:
	SplitAction() {}
	//! Constructor
	/*!
	\param name Action name.
	\param parameter Split length.
	\param grammar_index The grammar rule that this Action belongs to.
	*/
	SplitAction(std::string name, double parameter, int grammar_index = -1) { 
		grammar_name_ = name, parameter_ = parameter; grammar_index_ = grammar_index; }

	//! Copy constructor
	SplitAction(const SplitAction& sa) {
		this->grammar_name_ = sa.grammar_name_;
		this->parameter_ = sa.parameter_;
		this->grammar_index_ = sa.grammar_index_;
	}

	//! Set the grammar rule index that this action belongs to.
	inline void set_shape_grammar(int grammar_index) { grammar_index_ = grammar_index;}
	//! Get the grammar rule index that this action belongs to.
	inline int get_shape_grammar_index() { return grammar_index_; }
	//! Get the rule name of this action.
	inline std::string get_grammar_name() { return grammar_name_; }
	//! Get the rule parameter of this action.
	inline double get_parameter() { return parameter_; }

	SplitAction& operator=(const SplitAction& action)
	{
		this->grammar_name_ = action.grammar_name_;
		this->parameter_ = action.parameter_;
		this->grammar_index_ = action.grammar_index_;

		return *this;
	}

	bool operator==(const SplitAction& sa) const {
        return (grammar_name_ == sa.grammar_name_ && parameter_ == sa.parameter_);
    }
    bool operator!=(const SplitAction& sa) const {
        return (grammar_name_ != sa.grammar_name_ || parameter_ != sa.parameter_);
    }

    bool operator>(const SplitAction& sa) const {
			if(parameter_ == sa.parameter_)
				return (grammar_name_ > sa.grammar_name_);
			else
				return (parameter_ > sa.parameter_);
	}

	bool operator<(const SplitAction& sa) const {
		if(parameter_ == sa.parameter_)
				return (grammar_name_ < sa.grammar_name_);
			else
				return (parameter_ < sa.parameter_);
	}

private:
	std::string grammar_name_; /*!< grammar rule name of this action. */
	double parameter_;		   /*!< grammar rule parameter of this action. */
	int grammar_index_;		   /*!< grammar rule index of this action. */
};


class FacadeGrammar;

//! Shape symbol definition of the Shape Grammar.
/*! A shape symbol has its rectangle envelope and its type. */
class ShapeSymbol {
public:
    //! Constructors
	/*!
	\param facade_grammar The facade grammar that this shape symbol is in.
	*/
    ShapeSymbol(std::shared_ptr<FacadeGrammar> facade_grammar);
	//! Constructors
	/*!
	\param x x position of the left bottom corner of this shape symbol.
	\param y y position of the left bottom corner of this shape symbol.
	\param w width of this shape symbol.
	\param h height of this shape symbol.
	\param name the name of this shape symbol.
	\param facade_grammar the facade grammar that this shape symbol is in.
	*/
    ShapeSymbol (int x, int y, int w, int h, std::string name, std::shared_ptr<FacadeGrammar> facade_grammar);
	~ShapeSymbol() {}

	//! Set the envelop 
	/*!
	\param x x position of the left bottom corner of this shape symbol.
	\param y y position of the left bottom corner of this shape symbol.
	\param w width of this shape symbol.
	\param h height of this shape symbol.
	\return void.
	*/
	void SetScope(int x, int y, int w, int h);

	//! Split this shape symbol to two sub symbols with a given split action.
	/*!
	\param split_action The action that used to split this symbol.
	\param right_symbol_1 The first sub symbol.
	\param right_symbol_2 The second sub symbol.
	\param width the facade width.
	\param height the facade height.
	\return true if the split success, otherwise false. If is false, the right_symbol_1 is the exception symbol.
	*/
	bool DoSplit ( SplitAction& split_action, std::shared_ptr<ShapeSymbol> right_symbol_1,
		std::shared_ptr<ShapeSymbol> right_symbol_2, double width, double height);

    inline int position_x() { return position_x_; }
    inline int position_y() { return position_y_; }
    inline int width() { return width_; }
    inline int height() { return height_; }
    inline std::string symbol_name() { return symbol_name_; }
    void set_symbol_name ( std::string value );
    inline void set_position_x ( int value ) { this->position_x_ = value; }
    inline void set_position_y ( int value ) { this->position_y_ = value; }
    inline void set_width( int value ) { this->width_ = value; }
    inline void set_height ( int value ) { this->height_ = value; }
    inline bool is_terminal() { return this->is_terminal_; }
    inline bool is_atom() { return this->is_atom_; }
    inline void set_is_terminal(bool value) { this->is_terminal_ = value; }
    inline void set_is_atom(bool value) { this->is_atom_ = value; }

private:
	int position_x_, position_y_, width_, height_; /*!< Envelop of this shape symbol. */
	std::string symbol_name_; /*< symbol name. */
    bool is_terminal_, is_atom_; /*< If this symbol is terminal symbol, is atom symbol. */

	std::shared_ptr<FacadeGrammar> facade_grammar_; /*< The facade grammar that this symbol is in. */
};


#endif // GRAMMAR_ELEMENTS_H