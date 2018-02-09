#ifndef ABETARE__MP_PARAMS_HPP_
#define ABETARE__MP_PARAMS_HPP_

#include "Utils/Constants.hpp"

namespace Abetare
{
    const int PARAM_PLANNER_MIN_NR_STEPS = 40;
    const int PARAM_PLANNER_MAX_NR_STEPS = 80;
    const int PARAM_PLANNER_MIN_NR_ITERS_EXPAND_ACTION = 100;
    const int PARAM_PLANNER_MAX_NR_ITERS_EXPAND_ACTION = 200;
    const double PARAM_PLANNER_PROB_STEER = 0.90;
    
    
    const double PARAM_ROBOT_DT                = 0.1;
    const double PARAM_ROBOT_MIN_DIST_ONE_STEP = 0.3;
    const double PARAM_ROBOT_MAX_DIST_ONE_STEP = 0.7;

    const double PARAM_PROBLEM_HEIGHT_OBSTACLES   = 3.0;
    const double PARAM_PROBLEM_HEIGHT_OBJECTS     = 1.0;
    const double PARAM_PROBLEM_DEPOT_RELEASE_FRAC = 0.5;
    
 
    const double PARAM_DECOMPOSITION_MIN_AREA_TO_ADD = 0.05;
    const int    PARAM_DECOMPOSITION_COV_GRID_DIMX = 128;
    const int    PARAM_DECOMPOSITION_COV_GRID_DIMY = 128;
    
    const int PARAM_COST_MOVE = 1;
    const int PARAM_COST_MOVE_WITH_OBJECT = 4;
    const int PARAM_COST_PICKUP = 2;
    const int PARAM_COST_RELEASE = 3;
    const int PARAM_COST_PENALTY = 2;
    

    const double PARAM_REGION_GROUP_SEL_PENALTY = 1.75;
    const double PARAM_GROUP_SEL_PENALTY        = 1.10;

    const double PARAM_HCOST_MIN = 0.000000001;
    const double PARAM_HCOST_MAX = HUGE_VAL; //ldexp(1, 64);
    

}

#endif


