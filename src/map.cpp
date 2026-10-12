#include "map.h"
#include <cmath>
#include <algorithm>
#include <random>

static double deg2rad(double d) { return d * M_PI / 180.0; }

// ─── Shadow Model ──────────────────────────────────────────────────────────

static double craterShadowFactor(const CraterDef& c, int x, int y,
                                  double sun_angle_deg) {
    double dx = x - c.cx, dy = y - c.cy;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist >= c.radius) return 0.0;

    double sun_rad = deg2rad(sun_angle_deg);
    double sx = -std::cos(sun_rad), sy = -std::sin(sun_rad);

    double b_val = 2.0*(dx*sx + dy*sy);
    double cv    = dist*dist - (double)c.radius*(double)c.radius;
    double disc  = b_val*b_val - 4.0*cv;
    if (disc < 0) return 0.0;
    double t_rim = (-b_val + std::sqrt(disc)) / 2.0;
    if (t_rim <= 0) return 0.0;

    double elev_angle = std::atan2(c.depth, t_rim);
    double sun_elev   = std::abs(std::sin(sun_rad)) * 0.5 + 0.1;
    double shadow     = std::max(0.0, std::min(1.0,
        (elev_angle - deg2rad(sun_elev * 90.0)) / deg2rad(30.0)));

    double cell_depth  = c.depth * (1.0 - dist / c.radius);
    double depth_factor = cell_depth / c.depth;
    return shadow * depth_factor;
}

static double duneShadowFactor(const DuneDef& d, int x, int y,
                                double sun_angle_deg) {
    double dx = x - d.cx, dy = y - d.cy;
    double a  = deg2rad(d.angle_deg);
    double lx =  dx*std::cos(a) + dy*std::sin(a);
    double ly = -dx*std::sin(a) + dy*std::cos(a);

    if (std::abs(lx) > d.length/2.0 || std::abs(ly) > d.width*3) return 0.0;

    double sun_perp  = std::cos(deg2rad(sun_angle_deg - d.angle_deg));
    double shadow_len = d.height * 2.5;
    if (sun_perp >= 0) {
        if (ly >= 0) return 0.0;
        return std::max(0.0, 1.0 - std::abs(ly) / shadow_len) * 0.7;
    } else {
        if (ly <= 0) return 0.0;
        return std::max(0.0, 1.0 - std::abs(ly) / shadow_len) * 0.7;
    }
}

double LunarMap::lightLevel(int x, int y, double sun_angle_deg) const {
    double shadow = 0.0;
    for (const auto& c : craters)
        shadow = std::max(shadow, craterShadowFactor(c, x, y, sun_angle_deg));
    for (const auto& d : dunes)
        shadow = std::max(shadow, duneShadowFactor(d, x, y, sun_angle_deg));
    return 1.0 - std::min(1.0, shadow);
}

// ─── Procedural Map Generation ─────────────────────────────────────────────

LunarMap buildMap(int width, int height,
                  unsigned int seed,
                  const MapGenConfig& cfg) {
    LunarMap m;
    m.width  = width;
    m.height = height;
    m.cells.assign(height, std::vector<TerrainCell>(width));

    std::mt19937 rng(seed);
    auto randi = [&](int lo, int hi) {
        return std::uniform_int_distribution<int>(lo, hi)(rng);
    };
    auto randf = [&](double lo, double hi) {
        return std::uniform_real_distribution<double>(lo, hi)(rng);
    };

    // ── Default flat terrain with slight noise ──
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++) {
            auto& c = m.cells[y][x];
            c.type              = TerrainType::FLAT;
            c.base_traverse_cost = 1.0 + randf(0.0, 0.2);
            c.elevation         = 0.0;
            c.crater_depth      = 0.0;
            c.crater_id         = -1;
        }

    // ── Rocky patches (flat but rougher, no shadows) ──
    int n_rocky = randi(cfg.rocky_count_min, cfg.rocky_count_max);
    for (int ri = 0; ri < n_rocky; ri++) {
        RockyPatchDef rp;
        rp.cx     = randi(0, width-1);
        rp.cy     = randi(0, height-1);
        rp.radius = randi(cfg.rocky_radius_min, cfg.rocky_radius_max);
        rp.cost_multiplier = randf(cfg.rocky_cost_min, cfg.rocky_cost_max);
        m.rocky_patches.push_back(rp);

        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                auto& cell = m.cells[y][x];
                if (cell.crater_id >= 0) continue;
                double dx = x - rp.cx, dy = y - rp.cy;
                if (dx*dx + dy*dy > rp.radius*rp.radius) continue;
                // Blend: center is roughest, fades at edge
                double t = std::sqrt(dx*dx + dy*dy) / rp.radius;
                double cost_mult = 1.0 + (rp.cost_multiplier - 1.0) * (1.0 - t);
                cell.type = TerrainType::ROCKY_PATCH;
                cell.base_traverse_cost = (1.0 + randf(0.0, 0.2)) * cost_mult;
            }
    }

    // ── Craters (5–10x cost, deep shadows) ──
    int n_craters = randi(cfg.crater_count_min, cfg.crater_count_max);
    for (int ci = 0; ci < n_craters; ci++) {
        CraterDef cd;
        cd.cx     = randi(width/6, 5*width/6);
        cd.cy     = randi(height/6, 5*height/6);
        cd.radius = randi(cfg.crater_radius_min, cfg.crater_radius_max);
        cd.depth  = randf(cfg.crater_depth_min, cfg.crater_depth_max);
        m.craters.push_back(cd);

        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                double dx = x - cd.cx, dy = y - cd.cy;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist > cd.radius) continue;

                auto& cell = m.cells[y][x];
                cell.crater_id = ci;
                double t = dist / cd.radius;

                // Cost scales with cfg.crater_cost_scale, randomized per cell
                double cost_scale = cfg.crater_cost_scale * randf(0.7, 1.3);

                if (t < 0.75) {
                    cell.type = TerrainType::CRATER_FLOOR;
                    cell.crater_depth  = cd.depth * (1.0 - t/0.75);
                    cell.elevation     = -cell.crater_depth;
                    cell.base_traverse_cost = (1.0 + randf(0.0, 0.2)) * cost_scale;
                } else {
                    cell.type = TerrainType::CRATER;
                    cell.crater_depth  = cd.depth * (1.0 - t) / 0.25 * 0.2;
                    cell.elevation     = -cell.crater_depth * 0.5;
                    // Rim is slightly less costly than floor
                    cell.base_traverse_cost = (1.0 + randf(0.0, 0.2)) * cost_scale * 0.7;
                }
            }
    }

    // ── Dunes (5–10x cost, directional shadows) ──
    int n_dunes = randi(cfg.dune_count_min, cfg.dune_count_max);
    for (int di = 0; di < n_dunes; di++) {
        DuneDef dd;
        dd.cx        = randi(width/8, 7*width/8);
        dd.cy        = randi(height/8, 7*height/8);
        dd.length    = randi(cfg.dune_length_min, cfg.dune_length_max);
        dd.width     = randi(cfg.dune_width_min, cfg.dune_width_max);
        dd.angle_deg = randf(-30, 30);
        dd.height    = randf(cfg.dune_height_min, cfg.dune_height_max);
        m.dunes.push_back(dd);

        double a = deg2rad(dd.angle_deg);
        for (int y = 0; y < height; y++)
            for (int x = 0; x < width; x++) {
                auto& cell = m.cells[y][x];
                if (cell.crater_id >= 0) continue;
                double dx = x - dd.cx, dy = y - dd.cy;
                double lx =  dx*std::cos(a) + dy*std::sin(a);
                double ly = -dx*std::sin(a) + dy*std::cos(a);
                if (std::abs(lx) > dd.length/2.0 || std::abs(ly) > dd.width/2.0) continue;

                double t = std::abs(ly) / (dd.width/2.0);
                cell.type      = TerrainType::DUNE;
                cell.elevation = dd.height * (1.0 - t);
                double cost_scale = cfg.dune_cost_scale * randf(0.7, 1.3);
                cell.base_traverse_cost = (1.0 + randf(0.0, 0.2)) * cost_scale;
            }
    }

    return m;
}
