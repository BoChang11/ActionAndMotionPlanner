#ifndef ABETARE__LOGIC_FORMULA_FALSE_HPP_
#define ABETARE__LOGIC_FORMULA_FALSE_HPP_

#include "Logic/Formula.hpp"

namespace Abetare
{
    class FormulaFalse : public Formula
    {
    public:
	FormulaFalse(void) : Formula()
	{
	}
	
	virtual ~FormulaFalse(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return false;
	}	    
    };
}
    
#endif



