#ifndef ABETARE__LOGIC_FORMULA_HPP_
#define ABETARE__LOGIC_FORMULA_HPP_

namespace Abetare
{
    class Formula 
    {
    public:
	Formula(void) 
	{
	}
	
	virtual ~Formula(void)
	{
	}
	
	virtual bool IsSatisfied(const int n, const int props[]) const = 0;	    
    };
    
}
    
#endif



