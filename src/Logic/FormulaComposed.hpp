#ifndef ABETARE__LOGIC_FORMULA_COMPOSED_HPP_
#define ABETARE__LOGIC_FORMULA_COMPOSED_HPP_

#include "Logic/Formula.hpp"
#include "Utils/Misc.hpp"
#include <vector>

namespace Abetare
{
    class FormulaComposed : public Formula
    {
    public:
	FormulaComposed(void) : Formula()
	{
	}
	
	virtual ~FormulaComposed(void)
	{
	    DeleteItems<Formula *>(&m_subformulas);		
	}
	
	std::vector<Formula*> m_subformulas;
    };
}    

#endif



