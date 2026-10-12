#pragma once
#include "map.h"
#include <string>
#include <vector>

// ─── Rover / mission parameters ────────────────────────────────────────────
struct RoverConfig {
    // Start / goal
    int start_x = 0, start_y = 0;
    int goal_x  = 0, goal_y  = 0;

    // Battery state of charge, as a fraction of capacity [0, 1]
    double battery_start = 0.35;
    double battery_min   = 0.15;   // rover is lost if charge drops below this
    double battery_max   = 1.0;

    // Energy model (per simulated second)
    //   driving : net = (light * max_gain_per_sec - drain_per_sec)      * dt
    //   waiting : net = (light * max_gain_per_sec - idle_drain_per_sec) * dt
    double drain_per_sec      = 0.008;
    double idle_drain_per_sec = 0.002;
    double max_gain_per_sec   = 0.006;

    // Sensing: cells within this radius of the rover are observed
    int obs_radius = 6;

    // Sun: shadows move as the sun angle advances
    double sun_angle_start_deg = 10.0;
    double sun_deg_per_sec     = 0.04;

    // Planner options
    bool   battery_aware  = true;   // false = plan for time only (baseline)
    bool   allow_wait     = true;   // allow "park and recharge" actions
    double wait_sec       = 5.0;    // duration of one wait action
    double unknown_cost   = 1.0;    // optimistic cost assumed for unseen cells
    double battery_resolution = 0.005;  // charge differences smaller than this
                                        // don't keep a slower label alive
    double time_limit_sec = 0.0;    // 0 = no limit
    int    max_expansions = 2000000;
};

// One step of a plan or of an executed mission.
struct PathStep {
    int    x = 0, y = 0;
    double time = 0;          // seconds since mission start (on arrival)
    double battery = 0;       // state of charge on arrival
    double sun_angle = 0;
    double light_level = 0;   // light at this cell on arrival
    double dt = 0;            // duration of the action that led here
    bool   waited = false;    // true if this step was a wait in place
};

struct PlanResult {
    bool success = false;
    std::string failure_reason;
    std::vector<PathStep> path;   // path[0] is the start state
    double total_time = 0;
    double final_battery = 0;
    long   expansions = 0;
};

// Where the search starts from (the rover's current state).
struct SearchStart {
    int x, y;
    double time;
    double battery;
};

// Belief over terrain: known[y][x] == 1 means the true cell cost is known.
// Unknown cells are assumed to cost cfg.unknown_cost (free-space assumption).
// Pass nullptr to plan with the full map known.
using KnownGrid = std::vector<std::vector<char>>;

// Time-dependent A* over (x, y, time, battery).
//  - Objective: minimise arrival time at the goal.
//  - Constraint (battery_aware): charge never drops below battery_min.
//  - Each cell keeps a Pareto set of (time, battery) labels, so a slower
//    arrival with more charge is never discarded in favour of a faster,
//    emptier one.
PlanResult planPath(const LunarMap& map, const RoverConfig& cfg,
                    const SearchStart& start, const KnownGrid* known);

// Convenience: plan from cfg's start state with the full map known.
PlanResult planPath(const LunarMap& map, const RoverConfig& cfg);
