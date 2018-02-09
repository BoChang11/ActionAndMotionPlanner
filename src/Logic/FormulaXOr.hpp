#ifndef ABETARE__LOGIC_FORMULA_XOR_HPP_
#define ABETARE__LOGIC_FORMULA_XOR_HPP_

#include "Logic/FormulaComposed.hpp"

namespace Abetare
{
    class FormulaXOr : public FormulaComposed
    {
    public:
	FormulaXOr(void) : FormulaComposed()
	{
	}
	
	FormulaXOr(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}

	virtual ~FormulaXOr(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return 
		m_subformulas[0]->IsSatisfied(n, props) != 
		m_subformulas[1]->IsSatisfied(n, props);
	}	    
    };
}
    
#endif



