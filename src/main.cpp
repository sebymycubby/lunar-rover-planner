#include "map.h"
#include "mission.h"
#include "planner.h"
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// ═══════════════════════════════════════════════════════════════════════════
//  DEFAULT PARAMETERS — every one can be overridden from the command line
// ═══════════════════════════════════════════════════════════════════════════

static const int          MAP_WIDTH  = 60;
static const int          MAP_HEIGHT = 60;
static const unsigned int MAP_SEED   = 86;   // demo map used in the README

static MapGenConfig makeGenConfig() {
    MapGenConfig g;
    g.crater_count_min  = 4;    g.crater_count_max  = 7;
    g.crater_radius_min = 5;    g.crater_radius_max = 12;
    g.crater_depth_min  = 3.0;  g.crater_depth_max  = 8.0;

    g.dune_count_min   = 3;     g.dune_count_max   = 6;
    g.dune_length_min  = 8;     g.dune_length_max  = 20;
    g.dune_width_min   = 2;     g.dune_width_max   = 5;
    g.dune_height_min  = 1.5;   g.dune_height_max  = 4.0;

    g.rocky_count_min  = 6;     g.rocky_count_max  = 14;
    g.rocky_radius_min = 2;     g.rocky_radius_max = 7;
    g.rocky_cost_min   = 1.3;   g.rocky_cost_max   = 2.2;

    g.crater_cost_scale = 7.0;  // craters/dunes are 5-10x slower than flat
    g.dune_cost_scale   = 6.0;
    return g;
}

static RoverConfig makeRoverConfig(int W, int H) {
    RoverConfig c;
    c.start_x = 2;      c.start_y = H / 2;
    c.goal_x  = W - 3;  c.goal_y  = H / 2;
    // Demo scenario: the rover starts the traverse at 35% charge. Driving
    // costs more than full sunlight returns, so shadows are expensive and
    // parking in the sun is the only way to gain charge.
    c.battery_start      = 0.35;
    c.battery_min        = 0.15;
    c.drain_per_sec      = 0.008;
    c.idle_drain_per_sec = 0.002;
    c.max_gain_per_sec   = 0.006;
    c.obs_radius         = 6;
    c.sun_angle_start_deg = 10.0;
    c.sun_deg_per_sec     = 0.04;
    return c;
}

// ═══════════════════════════════════════════════════════════════════════════

static void usage() {
    std::cout <<
R"(lunar_rover — battery- and shadow-aware rover planner

Map
  --seed N            terrain seed (default 42)
  --width N --height N
Rover
  --start X Y  --goal X Y
  --battery-start F   initial charge, fraction of capacity
  --battery-min F     charge below which the rover is lost
  --drain F           battery drain per second while driving
  --idle-drain F      battery drain per second while parked
  --gain F            max solar charge per second in full light
  --obs-radius N      sensing radius in cells
  --sun-start DEG --sun-rate DEG_PER_SEC
Planner
  --blind             plan for time only, ignoring the battery (baseline)
  --no-wait           disable park-and-recharge actions
  --full-map          rover knows the whole map (no fog of war)
Output
  --output FILE       write full JSON for visualisation (default output/plan.json)
  --no-output         skip the JSON file
  --summary           print a one-line JSON summary to stdout
  --quiet             no human-readable log
)";
}

static std::string jsonEscape(const std::string& s) {
    std::string o;
    for (char ch : s) {
        if (ch == '"' || ch == '\\') o += '\\';
        o += ch;
    }
    return o;
}

static std::string summaryJSON(const MissionResult& r, const RoverConfig& cfg,
                               unsigned seed, bool full_map) {
    std::ostringstream f;
    f << std::setprecision(6);
    f << "{\"seed\":" << seed
      << ",\"mode\":\"" << (cfg.battery_aware ? "aware" : "blind") << "\""
      << ",\"full_map\":" << (full_map ? "true" : "false")
      << ",\"allow_wait\":" << (cfg.allow_wait ? "true" : "false")
      << ",\"drain\":" << cfg.drain_per_sec
      << ",\"gain\":" << cfg.max_gain_per_sec
      << ",\"battery_start\":" << cfg.battery_start
      << ",\"success\":" << (r.success ? "true" : "false")
      << ",\"failure_reason\":\"" << jsonEscape(r.failure_reason) << "\""
      << ",\"total_time\":" << r.total_time
      << ",\"final_battery\":" << r.final_battery
      << ",\"min_battery\":" << r.min_battery
      << ",\"steps\":" << (r.executed.size() - 1)
      << ",\"waits\":" << r.waits
      << ",\"route_changes\":" << r.route_changes
      << ",\"expansions\":" << r.total_expansions
      << ",\"planning_ms\":" << r.planning_ms << "}";
    return f.str();
}

template <typename T, typename F>
static void writeGrid(std::ostream& f, int W, int H, F get) {
    f << "[";
    for (int y = 0; y < H; y++) {
        f << (y ? ",\n    [" : "\n    [");
        for (int x = 0; x < W; x++) f << (x ? "," : "") << static_cast<T>(get(x, y));
        f << "]";
    }
    f << "]";
}

static void writeJSON(const LunarMap& map, const RoverConfig& cfg,
                      const MissionResult& r, unsigned seed, bool full_map,
                      const std::string& path) {
    std::ofstream f(path);
    if (!f) { std::cerr << "Cannot open " << path << "\n"; return; }
    f << std::setprecision(5);
    const int W = map.width, H = map.height;

    f << "{\n\"width\": " << W << ", \"height\": " << H << ", \"seed\": " << seed << ",\n";
    f << "\"terrain_type\": ";
    writeGrid<int>(f, W, H, [&](int x, int y) { return (int)map.cells[y][x].type; });
    f << ",\n\"cost\": ";
    writeGrid<double>(f, W, H, [&](int x, int y) { return map.cells[y][x].base_traverse_cost; });

    f << ",\n\"craters\": [";
    for (size_t i = 0; i < map.craters.size(); i++) {
        const auto& c = map.craters[i];
        f << (i ? "," : "") << "{\"cx\":" << c.cx << ",\"cy\":" << c.cy
          << ",\"r\":" << c.radius << ",\"d\":" << c.depth << "}";
    }
    f << "],\n\"dunes\": [";
    for (size_t i = 0; i < map.dunes.size(); i++) {
        const auto& d = map.dunes[i];
        f << (i ? "," : "") << "{\"cx\":" << d.cx << ",\"cy\":" << d.cy
          << ",\"l\":" << d.length << ",\"w\":" << d.width
          << ",\"a\":" << d.angle_deg << ",\"h\":" << d.height << "}";
    }
    f << "],\n";

    f << "\"config\": {\"start\":[" << cfg.start_x << "," << cfg.start_y << "]"
      << ",\"goal\":[" << cfg.goal_x << "," << cfg.goal_y << "]"
      << ",\"battery_start\":" << cfg.battery_start
      << ",\"battery_min\":" << cfg.battery_min
      << ",\"drain_per_sec\":" << cfg.drain_per_sec
      << ",\"idle_drain_per_sec\":" << cfg.idle_drain_per_sec
      << ",\"max_gain_per_sec\":" << cfg.max_gain_per_sec
      << ",\"sun_angle_start\":" << cfg.sun_angle_start_deg
      << ",\"sun_deg_per_sec\":" << cfg.sun_deg_per_sec
      << ",\"obs_radius\":" << cfg.obs_radius
      << ",\"battery_aware\":" << (cfg.battery_aware ? "true" : "false")
      << ",\"allow_wait\":" << (cfg.allow_wait ? "true" : "false")
      << ",\"full_map\":" << (full_map ? "true" : "false") << "},\n";

    // Light maps sampled every LIGHT_DT seconds over the mission, so the
    // visualiser uses exactly the planner's shadow model.
    const double LIGHT_DT = 5.0;
    const int n_frames = static_cast<int>(r.total_time / LIGHT_DT) + 2;
    f << "\"light_dt\": " << LIGHT_DT << ",\n\"light\": [";
    f << std::setprecision(3);
    for (int k = 0; k < n_frames; k++) {
        const double sun = cfg.sun_angle_start_deg + cfg.sun_deg_per_sec * k * LIGHT_DT;
        f << (k ? ",\n" : "\n");
        writeGrid<double>(f, W, H, [&](int x, int y) { return map.lightLevel(x, y, sun); });
    }
    f << "],\n" << std::setprecision(6);

    f << "\"result\": " << summaryJSON(r, cfg, seed, full_map) << ",\n";
    f << "\"executed\": [";
    for (size_t i = 0; i < r.executed.size(); i++) {
        const auto& s = r.executed[i];
        f << (i ? ",\n" : "\n") << "{\"x\":" << s.x << ",\"y\":" << s.y
          << ",\"t\":" << s.time << ",\"bat\":" << s.battery
          << ",\"sun\":" << s.sun_angle << ",\"light\":" << s.light_level
          << ",\"dt\":" << s.dt << ",\"wait\":" << (s.waited ? "true" : "false") << "}";
    }
    f << "],\n\"plans\": [";
    for (size_t i = 0; i < r.plans.size(); i++) {
        f << (i ? ",\n" : "\n") << "[";
        for (size_t j = 0; j < r.plans[i].size(); j++)
            f << (j ? "," : "") << "[" << r.plans[i][j].first << "," << r.plans[i][j].second << "]";
        f << "]";
    }
    f << "],\n\"reveal_step\": ";
    writeGrid<int>(f, W, H, [&](int x, int y) { return r.reveal_step[y][x]; });
    f << "\n}\n";
}

int main(int argc, char* argv[]) {
    int W = MAP_WIDTH, H = MAP_HEIGHT;
    unsigned seed = MAP_SEED;
    bool full_map = false, summary = false, quiet = false, write_file = true;
    std::string out_file = "output/plan.json";

    // First pass: map size (start/goal defaults depend on it)
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--width") && i + 1 < argc) W = atoi(argv[i + 1]);
        if (!strcmp(argv[i], "--height") && i + 1 < argc) H = atoi(argv[i + 1]);
    }
    RoverConfig cfg = makeRoverConfig(W, H);

    for (int i = 1; i < argc; i++) {
        auto is = [&](const char* flag, int nargs) {
            return !strcmp(argv[i], flag) && i + nargs < argc;
        };
        if      (is("--help", 0) || is("-h", 0)) { usage(); return 0; }
        else if (is("--width", 1) || is("--height", 1)) i++;
        else if (is("--seed", 1))          seed = (unsigned)strtoul(argv[++i], nullptr, 10);
        else if (is("--start", 2))         { cfg.start_x = atoi(argv[++i]); cfg.start_y = atoi(argv[++i]); }
        else if (is("--goal", 2))          { cfg.goal_x  = atoi(argv[++i]); cfg.goal_y  = atoi(argv[++i]); }
        else if (is("--battery-start", 1)) cfg.battery_start = atof(argv[++i]);
        else if (is("--battery-min", 1))   cfg.battery_min = atof(argv[++i]);
        else if (is("--drain", 1))         cfg.drain_per_sec = atof(argv[++i]);
        else if (is("--idle-drain", 1))    cfg.idle_drain_per_sec = atof(argv[++i]);
        else if (is("--gain", 1))          cfg.max_gain_per_sec = atof(argv[++i]);
        else if (is("--obs-radius", 1))    cfg.obs_radius = atoi(argv[++i]);
        else if (is("--sun-start", 1))     cfg.sun_angle_start_deg = atof(argv[++i]);
        else if (is("--sun-rate", 1))      cfg.sun_deg_per_sec = atof(argv[++i]);
        else if (is("--blind", 0))         cfg.battery_aware = false;
        else if (is("--no-wait", 0))       cfg.allow_wait = false;
        else if (is("--full-map", 0))      full_map = true;
        else if (is("--output", 1))        out_file = argv[++i];
        else if (is("--no-output", 0))     write_file = false;
        else if (is("--summary", 0))       summary = true;
        else if (is("--quiet", 0))         quiet = true;
        else { std::cerr << "Unknown option: " << argv[i] << "\n"; usage(); return 2; }
    }

    auto inBounds = [&](int x, int y) { return x >= 0 && x < W && y >= 0 && y < H; };
    if (!inBounds(cfg.start_x, cfg.start_y) || !inBounds(cfg.goal_x, cfg.goal_y)) {
        std::cerr << "Start or goal is outside the map\n";
        return 2;
    }

    LunarMap map = buildMap(W, H, seed, makeGenConfig());
    if (!quiet)
        std::cout << "Map " << W << "x" << H << " seed=" << seed
                  << "  planner=" << (cfg.battery_aware ? "battery-aware" : "battery-blind")
                  << (full_map ? "  (full map)" : "  (fog of war, replanning)") << "\n";

    MissionResult r = runMission(map, cfg, full_map);

    if (!quiet) {
        if (r.success)
            std::cout << "SUCCESS  time=" << r.total_time << "s  steps=" << r.executed.size() - 1
                      << "  waits=" << r.waits << "  battery=" << r.final_battery * 100
                      << "% (min " << r.min_battery * 100 << "%)  route changes=" << r.route_changes
                      << "  planning=" << r.planning_ms << "ms\n";
        else
            std::cout << "FAILED: " << r.failure_reason << " at t=" << r.total_time << "s\n";
    }
    if (summary) std::cout << summaryJSON(r, cfg, seed, full_map) << std::endl;
    if (write_file) {
        writeJSON(map, cfg, r, seed, full_map, out_file);
        if (!quiet) std::cout << "Wrote " << out_file << "\n";
    }
    return r.success ? 0 : 1;
}
