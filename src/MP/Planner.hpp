#ifndef ABETARE__PLANNER_HPP_
#define ABETARE__PLANNER_HPP_

#include "MP/Problem.hpp"
#include "MP/RobotSCar.hpp"
#include "MP/Action.hpp"
#include "MP/ActionPlan.hpp"
#include "MP/DiscreteSpace.hpp"
#include "MP/ContinuousSpace.hpp"
#include "MP/Vertex.hpp"
#include "MP/Groups.hpp"
#include "MP/Decomposition.hpp"
#include "MP/DecompositionSearchInfo.hpp"
#include "MP/PlannerStats.hpp"
#include <string>
#include <unordered_map>

namespace Abetare
{
    class Planner
    {
    public:
	Planner(void);

	enum Type
	    {
		PLANNER_NEW,
		PLANNER_SMAP,
		PLANNER_ONE
	    };
	
	    
	
	virtual ~Planner(void);

	static Planner* GetInstance(void)
	{
	    return m_instance;
	}
	

	virtual void CompleteSetup(void);
	
	bool IsSolved(void) const
	{
	    return m_vidAtGoal >= 0;
	}
	
	double PathCost(const std::vector<int> * const path) const;
	
	void GetSolution(std::vector<int> * const path) const;
	
	void ONERun(const double tmax);
	void SMAPRun(const double tmax);
	void Run(const double tmax);
	void Expand(void);
	bool Collision(const double cs[], const Action * const a);
	  	
	Vertex* AddVertex(const double cs[], 
			  const int parent, 
			  const Action * const a, 
			  const bool adone);

	void AddRoot(void);
	
	
	
	bool ExpandFrom(int vid, const double target[], const Action * const a);
	

	bool ActionComplete(double cs[], const Action * const a);
	

	void InitActionCosts(void);
	
	void PDDLProb(const int ds[], std::string * const msg);
	void PDDLPreamble(std::string * const msg);
	void PDDLConnects(std::string * const msg);
	void PDDLCosts(std::string * const msg);
 	void PDDLGoals(std::string * const msg);
	void PDDLMetric(std::string * const msg);
	void PDDLFixedPrefix(void);
	void PDDLFixedPostfix(void);
	

	void DrawVertices(void);
	void DrawEdges(void);
	void DrawRobotAndObjects(const double s[], const int attached);

	static Planner *m_instance;
	
	Problem         m_problem;
	RobotSCar       m_robot;
	ContinuousSpace m_continuousSpace;
	DiscreteSpace   m_discreteSpace;

	Groups               m_groups;
	std::vector<Vertex*> m_vertices;
	int                  m_vidAtGoal;
	
	Decomposition            m_decomposition;
	DecompositionSearchInfo  m_decompSearchInfo;
	GraphSearch<int>         m_decompSearch;

	std::unordered_map<Action, double> m_mapActionsToCosts;

	std::string m_pddlMsgFixedPrefix;
	std::string m_pddlMsgFixedPostfix;

	ActionPlan m_actionPlan;
	int        m_indexInPlan;
	char       m_actionPlanCmd[300];
	

	PlannerStats m_stats;

	Type m_plannerType;
	
	
    };    
}

#endif


