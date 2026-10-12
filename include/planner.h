#pragma once
#include "map.h"
#include <vector>
#include <limits>

struct RoverConfig {
    // Start / goal
    int start_x, start_y;
    int goal_x,  goal_y;

    // Battery
    double battery_start;
    double battery_min;
    double battery_max = 1.0;

    // ── NEW battery model ──────────────────────────────────────────────
    // drain_per_sec   : battery consumed per simulated second of movement
    //                   (always active, regardless of light)
    // max_gain_per_sec: max battery gained per second when light = 1.0
    //                   gain scales linearly: gain = light * max_gain_per_sec
    //                   net per cell = (light * max_gain_per_sec - drain_per_sec) * dt
    double drain_per_sec;
    double max_gain_per_sec;

    // Observation radius (fog of war)
    int obs_radius;

    // Sun parameters
    double sun_angle_start_deg;
    double sun_deg_per_sec;

    // Time limit (0 = no limit)
    double time_limit_sec;
};

struct PathStep {
    int x, y;
    double time;
    double battery;
    double sun_angle;
    double light_level;
    double traverse_cost;  // dt for this cell
    bool   was_known;
};

struct PlanResult {
    bool   success;
    std::string failure_reason;
    std::vector<PathStep> path;
    double total_time;
    double final_battery;
    std::vector<std::vector<std::vector<bool>>> visibility_snapshots;
};

PlanResult planPath(const LunarMap& map, const RoverConfig& cfg);
