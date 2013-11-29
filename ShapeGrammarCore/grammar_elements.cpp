#include "opencv2/opencv.hpp"
#include "grammar_elements.h"
#include "facade_grammar.h"
#include <assert.h>

ShapeSymbol::ShapeSymbol(std::shared_ptr<FacadeGrammar> facade_grammar) {
	facade_grammar_ = facade_grammar;
}

ShapeSymbol::ShapeSymbol ( int x, int y, int w, int h, std::string name, std::shared_ptr<FacadeGrammar> facade_grammar)
{
	facade_grammar_ = facade_grammar;

	this->is_terminal_ = false;
	this->is_atom_ = false;
	this->SetScope(x, y, w, h);
	this->set_symbol_name(name);
}

void ShapeSymbol::SetScope ( int x, int y, int w, int h )
{
	position_x_ = x;
	position_y_ = y;
	width_ = w;
	height_ = h;
}

void ShapeSymbol::set_symbol_name ( std::string value )
{
	symbol_name_ = value;

	this->is_atom_ = false;
	this->is_terminal_= false;

	if ( symbol_name_ == facade_grammar_->get_atom_symbol() )
		this->is_atom_ = true;

	for ( int i = 0; i < facade_grammar_->get_terminal_symbol_size(); ++i)
	{
		if(symbol_name_ == facade_grammar_->get_terminal_symbol(i))
		{
			this->is_terminal_ = true;
			break;
		}
	}
}


bool ShapeSymbol::DoSplit ( SplitAction& split_action, std::shared_ptr<ShapeSymbol> right_symbol_1,
						   std::shared_ptr<ShapeSymbol> right_symbol_2, double width, double height)
{
	ShapeGrammar shape_grammar = facade_grammar_->get_grammar(split_action.get_shape_grammar_index());

	// assert that the shape grammar is suitable
	assert(shape_grammar.left_hand_symbol_ == this->symbol_name_);
	
	double split_parameter = split_action.get_parameter();

#if 0
	if (split_parameter == 0)
	{
		right_symbol_1->SetScope(this->position_x_, this->position_y_, this->width_, this->height_);
		right_symbol_1->set_symbol_name(shape_grammar.exception_symbol);

		return false;
	}
#endif
	
	if(shape_grammar.apply_direction_ == 0) // vertical action
	{
		if((split_parameter + position_y_) >= height) // split fail...
		{
			right_symbol_1->SetScope(this->position_x_, this->position_y_, this->width_, this->height_);
			right_symbol_1->set_symbol_name(shape_grammar.exception_symbol);

			return false;
		}
		else
		{
			right_symbol_1->SetScope(this->position_x_, this->position_y_, this->width_, split_parameter);
			right_symbol_1->set_symbol_name(shape_grammar.right_hand_symbol1_);

			right_symbol_2->SetScope(this->position_x_, this->position_y_ + split_parameter,
									this->width_, this->height_ - split_parameter);
			right_symbol_2->set_symbol_name(shape_grammar.right_hand_symbol2_);

			return true;
		}
	}
	else	// horizontal action
	{
		if((split_parameter + position_x_) >= width) // split fail ... 
		{
			right_symbol_1->SetScope(this->position_x_, this->position_y_, this->width_, this->height_);
			right_symbol_1->set_symbol_name(shape_grammar.exception_symbol);

			return false;
		}
		else
		{
			right_symbol_1->SetScope(this->position_x_, this->position_y_, split_parameter, this->height_);
			right_symbol_1->set_symbol_name(shape_grammar.right_hand_symbol1_);

			right_symbol_2->SetScope(this->position_x_ + split_parameter, this->position_y_,
				this->width_ - split_parameter, this->height_);
			right_symbol_2->set_symbol_name(shape_grammar.right_hand_symbol2_);

			return true;
		}	
	}
}
