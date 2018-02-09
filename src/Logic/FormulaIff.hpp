#ifndef ABETARE__LOGIC_FORMULA_IFF_HPP_
#define ABETARE__LOGIC_FORMULA_IFF_HPP_

#include "Logic/FormulaComposed.hpp"

namespace Abetare
{
    class FormulaIff : public FormulaComposed
    {
    public:
	FormulaIff(void) : FormulaComposed()
	{
	}
	
	FormulaIff(Formula * const f1, Formula * const f2) : FormulaComposed()
	{
	    m_subformulas.push_back(f1);
	    m_subformulas.push_back(f2);
	}

	virtual ~FormulaIff(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const
	{
	    return 
		m_subformulas[0]->IsSatisfied(n, props) == 
		m_subformulas[1]->IsSatisfied(n, props);
	}	    
    };
}
    
#endif



