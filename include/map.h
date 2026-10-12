#pragma once
#include <vector>
#include <string>
#include <cmath>

// ─── Terrain Types ─────────────────────────────────────────────────────────
// FLAT         : smooth regolith, lowest cost, no shadows cast
// ROCKY_PATCH  : slightly rougher flat ground, no shadows, moderate cost
// DUNE         : sandy ridge, casts directional shadows, high cost
// CRATER       : crater rim/wall, casts deep shadows into bowl, high cost
// CRATER_FLOOR : deepest part of crater, most shadowed, highest cost
enum class TerrainType {
    FLAT         = 0,
    ROCKY_PATCH  = 1,
    DUNE         = 2,
    CRATER       = 3,
    CRATER_FLOOR = 4
};

struct TerrainCell {
    TerrainType type;
    double base_traverse_cost;  // seconds to cross this cell
    double elevation;
    double crater_depth;
    int    crater_id;           // -1 if not part of a crater
};

struct CraterDef {
    int cx, cy;
    int radius;
    double depth;
};

struct DuneDef {
    int cx, cy;
    int length, width;
    double angle_deg;
    double height;
};

struct RockyPatchDef {
    int cx, cy;
    int radius;
    double cost_multiplier;
};

// ─── Map Generation Config ─────────────────────────────────────────────────
struct MapGenConfig {
    int   crater_count_min  = 4;
    int   crater_count_max  = 7;
    int   crater_radius_min = 5;
    int   crater_radius_max = 12;
    double crater_depth_min = 3.0;
    double crater_depth_max = 8.0;

    int   dune_count_min   = 3;
    int   dune_count_max   = 6;
    int   dune_length_min  = 8;
    int   dune_length_max  = 20;
    int   dune_width_min   = 2;
    int   dune_width_max   = 5;
    double dune_height_min = 1.5;
    double dune_height_max = 4.0;

    int   rocky_count_min  = 6;
    int   rocky_count_max  = 14;
    int   rocky_radius_min = 2;
    int   rocky_radius_max = 7;
    double rocky_cost_min  = 1.3;
    double rocky_cost_max  = 2.2;

    double crater_cost_scale = 7.0;
    double dune_cost_scale   = 6.0;
};

// ─── Map ───────────────────────────────────────────────────────────────────
struct LunarMap {
    int width, height;
    std::vector<std::vector<TerrainCell>> cells;
    std::vector<CraterDef>     craters;
    std::vector<DuneDef>       dunes;
    std::vector<RockyPatchDef> rocky_patches;

    // Fraction of full sunlight reaching cell (x, y): 0 = full shadow, 1 = fully lit.
    double lightLevel(int x, int y, double sun_angle_deg) const;
};

LunarMap buildMap(int width, int height,
                  unsigned int seed = 42,
                  const MapGenConfig& gen_cfg = MapGenConfig{});
