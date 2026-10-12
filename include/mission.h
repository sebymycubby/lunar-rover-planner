#pragma once
#include "planner.h"
#include <string>
#include <utility>
#include <vector>

// Result of driving the rover through the world with a sense-plan-act loop.
struct MissionResult {
    bool success = false;
    std::string failure_reason;

    std::vector<PathStep> executed;  // states actually visited; [0] = start

    // plans[k] = route (x, y) the rover intended to follow at step k,
    // starting from executed[k]. Shows how the plan changes as terrain is
    // revealed.
    std::vector<std::vector<std::pair<int, int>>> plans;

    // reveal_step[y][x] = index of the step at which the cell was first
    // observed (-1 = never observed). Cell is known at step k if 0 <= value <= k.
    std::vector<std::vector<int>> reveal_step;

    int    route_changes = 0;      // replans that changed the intended route
    long   total_expansions = 0;   // A* expansions summed over all replans
    double planning_ms = 0;        // wall-clock time spent planning
    double total_time = 0;
    double final_battery = 0;
    double min_battery = 0;        // lowest charge seen during the mission
    int    waits = 0;              // number of park-and-recharge actions
};

// Drive from cfg.start to cfg.goal.
//  full_knowledge = false : the rover only knows terrain it has observed
//                           (within cfg.obs_radius); it re-plans after
//                           every step.
//  full_knowledge = true  : the whole map is known; plan once and execute.
//
// Lighting is treated as known in advance in both modes (on real missions
// it is predicted from orbital elevation maps and the sun ephemeris); what
// the rover has to discover is the traversability of the ground.
MissionResult runMission(const LunarMap& map, const RoverConfig& cfg,
                         bool full_knowledge);
