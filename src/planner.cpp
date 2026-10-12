#include "planner.h"
#include <queue>
#include <unordered_map>
#include <cmath>
#include <algorithm>
#include <iostream>

static double euclidean(int x1, int y1, int x2, int y2) {
    double dx = x2-x1, dy = y2-y1;
    return std::sqrt(dx*dx + dy*dy);
}

static const int DX[8] = {1,-1,0,0, 1,-1, 1,-1};
static const int DY[8] = {0,0,1,-1, 1,-1,-1, 1};
static const double MOVE_MULT[8] = {
    1.0, 1.0, 1.0, 1.0,
    M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2
};

static const double TIME_BUCKET = 5.0;

struct AStarKey {
    int x, y, tb;
    bool operator==(const AStarKey& o) const { return x==o.x&&y==o.y&&tb==o.tb; }
};
struct AStarKeyHash {
    size_t operator()(const AStarKey& k) const {
        return (size_t)k.x*100003 ^ (size_t)k.y*200003 ^ (size_t)(unsigned)k.tb;
    }
};

struct AStarNode {
    double f, g, battery, time;
    int x, y, parent_idx;
    bool was_known;
    double traverse_cost; // dt for this step
};

static void revealCells(std::vector<std::vector<bool>>& known,
                        int x, int y, int radius, int W, int H) {
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx*dx+dy*dy > radius*radius) continue;
            int nx = x+dx, ny = y+dy;
            if (nx>=0&&nx<W&&ny>=0&&ny<H) known[ny][nx] = true;
        }
}

PlanResult planPath(const LunarMap& map, const RoverConfig& cfg) {
    PlanResult result;
    result.success = false;
    int W = map.width, H = map.height;

    std::vector<std::vector<bool>> known(H, std::vector<bool>(W, false));
    revealCells(known, cfg.start_x, cfg.start_y, cfg.obs_radius, W, H);

    std::vector<AStarNode> pool;
    pool.reserve(1 << 18);

    auto cmp = [&](int a, int b){ return pool[a].f > pool[b].f; };
    std::priority_queue<int,std::vector<int>,decltype(cmp)> pq(cmp);

    std::unordered_map<AStarKey,double,AStarKeyHash> closed;

    auto heuristic = [&](int x, int y) {
        // Admissible: assume minimum flat cost (1.0) straight line
        return euclidean(x, y, cfg.goal_x, cfg.goal_y) * 1.0;
    };

    AStarNode start{};
    start.x = cfg.start_x; start.y = cfg.start_y;
    start.time = 0.0; start.battery = cfg.battery_start;
    start.g = 0.0; start.f = heuristic(cfg.start_x, cfg.start_y);
    start.parent_idx = -1; start.was_known = true; start.traverse_cost = 0.0;
    pool.push_back(start);
    pq.push(0);

    int iterations = 0;
    const int MAX_ITER = 2000000;

    while (!pq.empty() && iterations++ < MAX_ITER) {
        int idx = pq.top(); pq.pop();
        const AStarNode& cur = pool[idx];

        int tb = (int)(cur.time / TIME_BUCKET);
        AStarKey key{cur.x, cur.y, tb};
        auto it = closed.find(key);
        if (it != closed.end() && it->second <= cur.g) continue;
        closed[key] = cur.g;

        revealCells(known, cur.x, cur.y, cfg.obs_radius, W, H);

        // Goal check
        if (cur.x == cfg.goal_x && cur.y == cfg.goal_y) {
            result.success      = true;
            result.total_time   = cur.time;
            result.final_battery = cur.battery;

            // Reconstruct path
            std::vector<PathStep> rev;
            int ci = idx;
            while (ci >= 0) {
                const AStarNode& n = pool[ci];
                double sa = cfg.sun_angle_start_deg + cfg.sun_deg_per_sec * n.time;
                PathStep ps;
                ps.x = n.x; ps.y = n.y;
                ps.time = n.time; ps.battery = n.battery;
                ps.sun_angle = sa;
                ps.light_level = map.lightLevel(n.x, n.y, sa);
                ps.traverse_cost = n.traverse_cost;
                ps.was_known = n.was_known;
                rev.push_back(ps);
                ci = n.parent_idx;
            }
            std::reverse(rev.begin(), rev.end());
            result.path = rev;

            // Rebuild fog snapshots
            std::vector<std::vector<bool>> fog(H, std::vector<bool>(W, false));
            revealCells(fog, cfg.start_x, cfg.start_y, cfg.obs_radius, W, H);
            for (const auto& step : result.path) {
                revealCells(fog, step.x, step.y, cfg.obs_radius, W, H);
                result.visibility_snapshots.push_back(fog);
            }
            return result;
        }

        if (cfg.time_limit_sec > 0 && cur.time > cfg.time_limit_sec) continue;

        for (int d = 0; d < 8; d++) {
            int nx = cur.x + DX[d];
            int ny = cur.y + DY[d];
            if (nx<0||nx>=W||ny<0||ny>=H) continue;

            bool cell_known = known[ny][nx];
            const TerrainCell& tc = map.cells[ny][nx];

            // Optimistic cost for unknown cells
            double cell_cost = cell_known ? tc.base_traverse_cost : 1.0;
            double dt        = cell_cost * MOVE_MULT[d];
            double new_time  = cur.time + dt;

            // Sun angle at time of arrival
            double sa = cfg.sun_angle_start_deg + cfg.sun_deg_per_sec * new_time;

            // Battery: net = (light * max_gain - drain) * dt
            double bd      = map.batteryDelta(nx, ny, sa, dt,
                                               cfg.max_gain_per_sec,
                                               cfg.drain_per_sec);
            double new_bat = std::min(cfg.battery_max, cur.battery + bd);
            if (new_bat < cfg.battery_min) continue;

            double new_g = new_time;
            double new_f = new_g + heuristic(nx, ny);

            AStarNode nn{};
            nn.x = nx; nn.y = ny;
            nn.time = new_time; nn.battery = new_bat;
            nn.g = new_g; nn.f = new_f;
            nn.parent_idx = idx;
            nn.was_known  = cell_known;
            nn.traverse_cost = dt;

            pool.push_back(nn);
            pq.push((int)pool.size()-1);
        }
    }

    result.failure_reason = iterations >= MAX_ITER
        ? "Max iterations reached"
        : "No valid path (battery constraint too strict or no path exists)";
    return result;
}
