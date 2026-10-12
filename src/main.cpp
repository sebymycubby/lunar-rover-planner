#include "map.h"
#include "planner.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>

// ═══════════════════════════════════════════════════════════════════════════
//  TUNABLE PARAMETERS — edit these freely
// ═══════════════════════════════════════════════════════════════════════════

// ── Map ────────────────────────────────────────────────────────────────────
static const int    MAP_WIDTH  = 60;
static const int    MAP_HEIGHT = 60;
// seed: change to get a different terrain layout (any positive integer)
static const unsigned int MAP_SEED = 42;

// ── Terrain Feature Frequency & Size ──────────────────────────────────────
static MapGenConfig makeGenConfig() {
    MapGenConfig g;

    // Craters
    g.crater_count_min  = 4;    g.crater_count_max  = 7;
    g.crater_radius_min = 5;    g.crater_radius_max = 12;
    g.crater_depth_min  = 3.0;  g.crater_depth_max  = 8.0;

    // Dunes
    g.dune_count_min   = 3;     g.dune_count_max   = 6;
    g.dune_length_min  = 8;     g.dune_length_max  = 20;
    g.dune_width_min   = 2;     g.dune_width_max   = 5;
    g.dune_height_min  = 1.5;   g.dune_height_max  = 4.0;

    // Rocky patches (flat but rougher, no shadows)
    g.rocky_count_min  = 6;     g.rocky_count_max  = 14;
    g.rocky_radius_min = 2;     g.rocky_radius_max = 7;
    g.rocky_cost_min   = 1.3;   g.rocky_cost_max   = 2.2;

    // Traverse cost multipliers (relative to flat = 1.0)
    // Craters and dunes are 5–10x harder than flat terrain
    g.crater_cost_scale = 7.0;
    g.dune_cost_scale   = 6.0;

    return g;
}

// ── Rover Start / Goal ─────────────────────────────────────────────────────
static const int START_X = 2;
static const int START_Y = MAP_HEIGHT / 2;
static const int GOAL_X  = MAP_WIDTH - 3;
static const int GOAL_Y  = MAP_HEIGHT / 2;

// ── Battery ────────────────────────────────────────────────────────────────
// drain_per_sec   : battery consumed per simulated second of travel (always on)
// max_gain_per_sec: max battery gained per second at full sunlight (light=1.0)
//                   gain = light_level * max_gain_per_sec * dt
//                   net  = (light * max_gain_per_sec - drain_per_sec) * dt
static const double BATTERY_START       = 0.90;
static const double BATTERY_MIN         = 0.15;   // rover dies below this
static const double DRAIN_PER_SEC       = 0.005;  // ← key tuning parameter
static const double MAX_GAIN_PER_SEC    = 0.006;  // ← key tuning parameter

// ── Observation / Fog of War ───────────────────────────────────────────────
static const int OBS_RADIUS = 6;   // cells revealed around rover

// ── Sun / Shadow ──────────────────────────────────────────────────────────
static const double SUN_ANGLE_START = 10.0;   // degrees; 0=east horizon
static const double SUN_DEG_PER_SEC = 0.04;   // shadow sweep speed

// ── Time Limit ────────────────────────────────────────────────────────────
static const double TIME_LIMIT_SEC = 0.0;     // 0 = no limit

// ═══════════════════════════════════════════════════════════════════════════

static void writeJSON(const LunarMap& map, const RoverConfig& cfg,
                      const PlanResult& result, const std::string& path) {
    std::ofstream f(path);
    if (!f) { std::cerr << "Cannot open " << path << "\n"; return; }

    int W = map.width, H = map.height;
    f << "{\n";
    f << "  \"width\": " << W << ",\n";
    f << "  \"height\": " << H << ",\n";

    f << "  \"terrain\": [\n";
    for (int y = 0; y < H; y++) {
        f << "    [";
        for (int x = 0; x < W; x++) {
            const auto& c = map.cells[y][x];
            f << "{\"t\":" << (int)c.type
              << ",\"c\":" << c.base_traverse_cost
              << ",\"e\":" << c.elevation
              << ",\"cid\":" << c.crater_id << "}";
            if (x < W-1) f << ",";
        }
        f << "]";
        if (y < H-1) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    f << "  \"craters\": [\n";
    for (size_t i = 0; i < map.craters.size(); i++) {
        const auto& c = map.craters[i];
        f << "    {\"cx\":" << c.cx << ",\"cy\":" << c.cy
          << ",\"r\":" << c.radius << ",\"d\":" << c.depth << "}";
        if (i+1 < map.craters.size()) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    f << "  \"dunes\": [\n";
    for (size_t i = 0; i < map.dunes.size(); i++) {
        const auto& d = map.dunes[i];
        f << "    {\"cx\":" << d.cx << ",\"cy\":" << d.cy
          << ",\"l\":" << d.length << ",\"w\":" << d.width
          << ",\"a\":" << d.angle_deg << ",\"h\":" << d.height << "}";
        if (i+1 < map.dunes.size()) f << ",";
        f << "\n";
    }
    f << "  ],\n";

    f << "  \"config\": {\n";
    f << "    \"start\": [" << cfg.start_x << "," << cfg.start_y << "],\n";
    f << "    \"goal\":  [" << cfg.goal_x  << "," << cfg.goal_y  << "],\n";
    f << "    \"battery_start\": "  << cfg.battery_start    << ",\n";
    f << "    \"battery_min\": "    << cfg.battery_min      << ",\n";
    f << "    \"drain_per_sec\": "  << cfg.drain_per_sec    << ",\n";
    f << "    \"max_gain_per_sec\": " << cfg.max_gain_per_sec << ",\n";
    f << "    \"sun_angle_start\": " << cfg.sun_angle_start_deg << ",\n";
    f << "    \"sun_deg_per_sec\": "  << cfg.sun_deg_per_sec    << ",\n";
    f << "    \"obs_radius\": " << cfg.obs_radius << "\n";
    f << "  },\n";

    f << "  \"result\": {\n";
    f << "    \"success\": "        << (result.success ? "true" : "false") << ",\n";
    f << "    \"failure_reason\": \"" << result.failure_reason << "\",\n";
    f << "    \"total_time\": "     << result.total_time    << ",\n";
    f << "    \"final_battery\": "  << result.final_battery << ",\n";

    f << "    \"path\": [\n";
    for (size_t i = 0; i < result.path.size(); i++) {
        const auto& s = result.path[i];
        f << "      {\"x\":" << s.x << ",\"y\":" << s.y
          << ",\"t\":" << s.time
          << ",\"bat\":" << s.battery
          << ",\"sun\":" << s.sun_angle
          << ",\"light\":" << s.light_level
          << ",\"dt\":" << s.traverse_cost
          << ",\"known\":" << (s.was_known ? "true" : "false") << "}";
        if (i+1 < result.path.size()) f << ",";
        f << "\n";
    }
    f << "    ],\n";

    f << "    \"visibility\": [\n";
    for (size_t si = 0; si < result.visibility_snapshots.size(); si++) {
        const auto& snap = result.visibility_snapshots[si];
        f << "      [";
        for (int y = 0; y < H; y++)
            for (int x = 0; x < W; x++) {
                f << (snap[y][x] ? "1" : "0");
                if (y < H-1 || x < W-1) f << ",";
            }
        f << "]";
        if (si+1 < result.visibility_snapshots.size()) f << ",";
        f << "\n";
    }
    f << "    ]\n";
    f << "  }\n}\n";

    std::cout << "Output written to " << path << "\n";
}

int main(int argc, char* argv[]) {
    // CLI overrides (for sweep script): --seed S --drain D --gain G --output FILE
    int    W      = MAP_WIDTH;
    int    H      = MAP_HEIGHT;
    unsigned int seed = MAP_SEED;
    double drain  = DRAIN_PER_SEC;
    double gain   = MAX_GAIN_PER_SEC;
    std::string out_file = "output/plan.json";

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i],"--width")  && i+1<argc) W        = atoi(argv[++i]);
        if (!strcmp(argv[i],"--height") && i+1<argc) H        = atoi(argv[++i]);
        if (!strcmp(argv[i],"--seed")   && i+1<argc) seed     = (unsigned)atoi(argv[++i]);
        if (!strcmp(argv[i],"--drain")  && i+1<argc) drain    = atof(argv[++i]);
        if (!strcmp(argv[i],"--gain")   && i+1<argc) gain     = atof(argv[++i]);
        if (!strcmp(argv[i],"--output") && i+1<argc) out_file = argv[++i];
    }

    MapGenConfig gen_cfg = makeGenConfig();

    std::cout << "Building map " << W << "x" << H
              << " seed=" << seed
              << " drain=" << drain
              << " gain=" << gain << "\n";

    LunarMap map = buildMap(W, H, seed, gen_cfg);

    RoverConfig cfg;
    cfg.start_x = START_X;    cfg.start_y = START_Y;
    cfg.goal_x  = GOAL_X;     cfg.goal_y  = GOAL_Y;
    cfg.battery_start    = BATTERY_START;
    cfg.battery_min      = BATTERY_MIN;
    cfg.battery_max      = 1.0;
    cfg.drain_per_sec    = drain;
    cfg.max_gain_per_sec = gain;
    cfg.obs_radius       = OBS_RADIUS;
    cfg.sun_angle_start_deg = SUN_ANGLE_START;
    cfg.sun_deg_per_sec     = SUN_DEG_PER_SEC;
    cfg.time_limit_sec      = TIME_LIMIT_SEC;

    std::cout << "Running planner...\n";
    PlanResult result = planPath(map, cfg);

    if (result.success) {
        std::cout << "SUCCESS  steps=" << result.path.size()
                  << "  time=" << result.total_time << "s"
                  << "  battery=" << (result.final_battery*100.0) << "%\n";
    } else {
        std::cout << "FAILED: " << result.failure_reason << "\n";
    }

    writeJSON(map, cfg, result, out_file);
    return result.success ? 0 : 1;
}
