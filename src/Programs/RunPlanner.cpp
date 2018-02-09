#include "MP/Planner.hpp"

using namespace Abetare;

extern "C" int RunPlanner(int argc, char **argv)
{
    char        *fname     = argc > 1 ? argv[1] : NULL;
    FILE        *in        = argc > 1 ? fopen(argv[1], "r") : NULL;    
    double       tcurr     = 0;
    int          count     = 1;
    double       tint      = 2;
    int          nrRuns    = 0;
    FILE        *out       = argc > 2 ? fopen(argv[2], "a+") : NULL;
    int          maxNrRuns = argc > 3 ? atoi(argv[3]) : 60;
    const double tmax      = argc > 4 ? atof(argv[4]) : 40;
    const char  *ai        = argc > 5 ? argv[5] : "seq-opt-fdss-1";
    const char  *type      = argc > 6 ? argv[6] : "";
    
    int          nrNotSolved= 0;

    printf("args\n");
    printf("  input file      = <%s>\n", argc > 1 ? argv[1] : "");
    printf("  statistics file = <%s>\n", argc > 2 ? argv[2] : "");
    printf("  maxNrRuns       = %d\n", maxNrRuns);
    printf("  tmax            = %f\n", tmax);
    printf("  ai              = <%s>\n", ai);
    printf("  type            = <%s>\n", type);
    

    if(!in)
	OnInputError(printf("could not open input file <%s>\n", fname));
    if(!out)
	OnInputError(printf("could not open stats file <%s>\n", argc > 2 ? argv[2] : ""));

    
    int    solved;
    double t;	
    double cost;
    
    while(fscanf(out, "%d %lf %lf", &solved, &t, &cost) == 3)
    {
	++nrRuns;
	if(solved == 0)
	    ++nrNotSolved;
    }
    
    if((maxNrRuns >= 0 && nrRuns >= maxNrRuns) || nrNotSolved >= 30)
    {
	fclose(out);
	fclose(in);

	if(nrRuns >= maxNrRuns)
	    printf("planner has already been run %d times\n", nrRuns);
	else 
	    printf("planner has failed to solve %d instances. What's the point?\n", nrNotSolved);
	return 0;	
    }
    
    Planner planner;

    sprintf(planner.m_actionPlanCmd, "../External/FastDownward/src/fast-downward.py --alias %s --log-level warning PDDLsoko/problem.pddl", ai);
    
    cost = HUGE_VAL;
    printf("reading input file\n");
    planner.m_problem.Read(in); fclose(in);
    printf("completing setup\n");
    planner.CompleteSetup();
    printf("running\n");
    
    if(strcmp(type, "SMAP") == 0)
    {
	planner.m_plannerType = Planner::PLANNER_SMAP;
	planner.SMAPRun(tmax);
    }
    else if(strcmp(type, "ONE") == 0)
    {
	planner.m_plannerType = Planner::PLANNER_ONE;
	planner.ONERun(tmax);
    }
    else
	planner.Run(tmax);
    
    if(planner.IsSolved())
    {
	std::vector<int> path;
	
	planner.GetSolution(&path);
	cost = planner.PathCost(&path);
	
    }
    
    fprintf(out, "%d %f %f\n", planner.IsSolved(), planner.m_stats.m_times[PlannerStats::RUN], cost);
    fclose(out);
    
    
    char cmd[300];
    sprintf(cmd, "%s_times", argv[2]);	
    out = fopen(cmd, "a+");
    planner.m_stats.Print(out);
    fclose(out);

}
