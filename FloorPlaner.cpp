
#include "Solver.cpp"


bool FloorPlan(string blocks_file, string nets_file, string output_file, int rand_seed, float time_limit, int strategy=0)  //(double alpha, string blockpath, string netpath, string output_file)
{
    Config cfg(blocks_file, nets_file, output_file, rand_seed, time_limit, strategy);
    Solver s(cfg);
    if(!s.run())
    {
        return false;
    }
    s.record_log("floorplan_log.csv");
    return true;
}
