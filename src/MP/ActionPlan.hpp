#ifndef ABETARE__ACTION_PLAN_HPP_
#define ABETARE__ACTION_PLAN_HPP_

#include "MP/Action.hpp"
#include <vector>

namespace Abetare
{
    class ActionPlan
    {
    public:
	ActionPlan(void) : m_cost(0.0)
	{
	}
	
	virtual ~ActionPlan(void)
	{
	}

	bool Compute(const int ds[]);
	void Read(const char fname[]);
	void RecomputeCost(void);

	std::vector<Action> m_plan;
	double              m_cost;
	std::vector<int*>   m_discreteStates;
	
    };
    
	
}

#endif


