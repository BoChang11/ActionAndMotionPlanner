#ifndef ABETARE__MP_PLANNER_STATS_HPP_
#define ABETARE__MP_PLANNER_STATS_HPP_

#include <cstdio>

namespace Abetare
{
    struct PlannerStats
    {
	enum
	    { 
		COMPLETE_SETUP = 0,
		RUN,
		AI_TIME,
		AI_CALLS,
		AI_LENGTH,
		EXPAND,
		COLLISION,
		SIMULATE,
		ADD_VERTEX,
		NR_TIMES
	    };
	
	
	PlannerStats(void)
	{
	    for(int i = 0; i < NR_TIMES; ++i)
		m_times[i] = 0;
	    
   	}
	
	void Print(FILE * out) const
	{
	    for(int i = 0; i < NR_TIMES; ++i)
		fprintf(out, "%8.4f ", m_times[i]);
	    fprintf(out, "\n");
	}

	
	double m_times[NR_TIMES];
	
    };
    

}

#endif


