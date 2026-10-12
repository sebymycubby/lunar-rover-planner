#include "planner.h"
#include <algorithm>
#include <cmath>
#include <queue>

namespace {

const int    DX[8]       = {1, -1, 0, 0, 1, -1, 1, -1};
const int    DY[8]       = {0, 0, 1, -1, 1, -1, -1, 1};
const double MOVE_MUL[8] = {1, 1, 1, 1, M_SQRT2, M_SQRT2, M_SQRT2, M_SQRT2};
const double EPS = 1e-9;

struct Node {
    double f, time, battery, dt;
    int    x, y, parent;
    bool   waited;
};

// A (time, battery) label that has been expanded at some cell.
struct Label {
    double time, battery;
};

// Octile distance: exact shortest grid distance with 8-connected moves.
double octile(int x1, int y1, int x2, int y2) {
    double dx = std::abs(x2 - x1), dy = std::abs(y2 - y1);
    return std::max(dx, dy) + (M_SQRT2 - 1.0) * std::min(dx, dy);
}

}  // namespace

PlanResult planPath(const LunarMap& map, const RoverConfig& cfg,
                    const SearchStart& s0, const KnownGrid* known) {
    PlanResult result;
    const int W = map.width, H = map.height;

    auto cellCost = [&](int x, int y) {
        if (known && !(*known)[y][x]) return cfg.unknown_cost;
        return map.cells[y][x].base_traverse_cost;
    };
    auto sunAt = [&](double t) {
        return cfg.sun_angle_start_deg + cfg.sun_deg_per_sec * t;
    };

    // Heuristic: octile distance times the cheapest cell cost the planner
    // could ever see. Never overestimates, so A* returns a time-optimal plan.
    double min_cost = cfg.unknown_cost;
    for (const auto& row : map.cells)
        for (const auto& c : row) min_cost = std::min(min_cost, c.base_traverse_cost);
    auto heuristic = [&](int x, int y) {
        return min_cost * octile(x, y, cfg.goal_x, cfg.goal_y);
    };

    // Label A dominates label B if it arrives no later AND with no less
    // charge (to within cfg.battery_resolution). Without the battery
    // constraint only time matters. The tolerance keeps the Pareto sets
    // small; plans stay feasible because every label carries its exact charge.
    const double bat_tol = std::max(EPS, cfg.battery_resolution);
    auto dominates = [&](const Label& a, double t, double b) {
        if (!cfg.battery_aware) return a.time <= t + EPS;
        return a.time <= t + EPS && a.battery >= b - bat_tol;
    };
    std::vector<std::vector<Label>> closed(W * H);
    auto isDominated = [&](int x, int y, double t, double b) {
        for (const auto& l : closed[y * W + x])
            if (dominates(l, t, b)) return true;
        return false;
    };

    std::vector<Node> pool;
    pool.reserve(1 << 16);
    auto cmp = [&](int a, int b) {
        if (pool[a].f != pool[b].f) return pool[a].f > pool[b].f;
        return pool[a].battery < pool[b].battery;  // tie-break: more charge first
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    pool.push_back({heuristic(s0.x, s0.y), s0.time, s0.battery, 0.0,
                    s0.x, s0.y, -1, false});
    open.push(0);

    long expansions = 0;
    while (!open.empty()) {
        if (expansions >= cfg.max_expansions) {
            result.failure_reason = "search budget exhausted";
            result.expansions = expansions;
            return result;
        }
        const int idx = open.top();
        open.pop();
        const Node cur = pool[idx];  // copy: pool may reallocate below

        if (isDominated(cur.x, cur.y, cur.time, cur.battery)) continue;
        auto& labels = closed[cur.y * W + cur.x];
        labels.erase(std::remove_if(labels.begin(), labels.end(),
                         [&](const Label& l) {
                             return cfg.battery_aware
                                 ? (cur.time <= l.time + EPS && cur.battery >= l.battery - EPS)
                                 : cur.time <= l.time + EPS;
                         }),
                     labels.end());
        labels.push_back({cur.time, cur.battery});
        ++expansions;

        // ── Goal reached: rebuild the path ──
        if (cur.x == cfg.goal_x && cur.y == cfg.goal_y) {
            std::vector<PathStep> rev;
            for (int ci = idx; ci >= 0; ci = pool[ci].parent) {
                const Node& n = pool[ci];
                PathStep ps;
                ps.x = n.x;  ps.y = n.y;
                ps.time = n.time;  ps.battery = n.battery;
                ps.sun_angle = sunAt(n.time);
                ps.light_level = map.lightLevel(n.x, n.y, ps.sun_angle);
                ps.dt = n.dt;  ps.waited = n.waited;
                rev.push_back(ps);
            }
            std::reverse(rev.begin(), rev.end());
            result.success = true;
            result.path = std::move(rev);
            result.total_time = cur.time;
            result.final_battery = cur.battery;
            result.expansions = expansions;
            return result;
        }

        if (cfg.time_limit_sec > 0 && cur.time > cfg.time_limit_sec) continue;

        auto push = [&](int nx, int ny, double dt, double net_rate, bool waited) {
            const double t = cur.time + dt;
            double b = std::min(cfg.battery_max, cur.battery + net_rate * dt);
            if (cfg.battery_aware && b < cfg.battery_min) return;
            if (isDominated(nx, ny, t, b)) return;
            pool.push_back({t + heuristic(nx, ny), t, b, dt, nx, ny, idx, waited});
            open.push(static_cast<int>(pool.size()) - 1);
        };

        // ── Drive to one of 8 neighbours ──
        for (int d = 0; d < 8; d++) {
            const int nx = cur.x + DX[d], ny = cur.y + DY[d];
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            const double dt = cellCost(nx, ny) * MOVE_MUL[d];
            // Lighting is evaluated at the destination cell on arrival.
            const double light = map.lightLevel(nx, ny, sunAt(cur.time + dt));
            push(nx, ny, dt, light * cfg.max_gain_per_sec - cfg.drain_per_sec, false);
        }

        // ── Park and recharge (only useful if it actually gains charge) ──
        if (cfg.battery_aware && cfg.allow_wait &&
            cur.battery < cfg.battery_max - EPS) {
            const double dt = cfg.wait_sec;
            const double light = map.lightLevel(cur.x, cur.y, sunAt(cur.time + dt));
            const double net = light * cfg.max_gain_per_sec - cfg.idle_drain_per_sec;
            if (net > 0) push(cur.x, cur.y, dt, net, true);
        }
    }

    result.failure_reason = cfg.battery_aware
        ? "no path keeps the battery above the minimum"
        : "goal unreachable";
    result.expansions = expansions;
    return result;
}

PlanResult planPath(const LunarMap& map, const RoverConfig& cfg) {
    return planPath(map, cfg,
                    SearchStart{cfg.start_x, cfg.start_y, 0.0, cfg.battery_start},
                    nullptr);
}
