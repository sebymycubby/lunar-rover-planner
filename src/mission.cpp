#include "mission.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace {

void reveal(KnownGrid& known, std::vector<std::vector<int>>& reveal_step,
            int x, int y, int radius, int step) {
    const int H = static_cast<int>(known.size());
    const int W = static_cast<int>(known[0].size());
    for (int dy = -radius; dy <= radius; dy++)
        for (int dx = -radius; dx <= radius; dx++) {
            if (dx * dx + dy * dy > radius * radius) continue;
            const int nx = x + dx, ny = y + dy;
            if (nx < 0 || nx >= W || ny < 0 || ny >= H) continue;
            if (!known[ny][nx]) {
                known[ny][nx] = 1;
                reveal_step[ny][nx] = step;
            }
        }
}

std::vector<std::pair<int, int>> route(const PlanResult& p) {
    std::vector<std::pair<int, int>> r;
    r.reserve(p.path.size());
    for (const auto& s : p.path) r.emplace_back(s.x, s.y);
    return r;
}

}  // namespace

MissionResult runMission(const LunarMap& map, const RoverConfig& cfg_in,
                         bool full_knowledge) {
    RoverConfig cfg = cfg_in;
    cfg.obs_radius = std::max(cfg.obs_radius, 1);  // must see the next cell

    const int W = map.width, H = map.height;
    MissionResult m;
    KnownGrid known(H, std::vector<char>(W, full_knowledge ? 1 : 0));
    m.reveal_step.assign(H, std::vector<int>(W, full_knowledge ? 0 : -1));

    auto sunAt = [&](double t) {
        return cfg.sun_angle_start_deg + cfg.sun_deg_per_sec * t;
    };

    PathStep cur;
    cur.x = cfg.start_x;  cur.y = cfg.start_y;
    cur.time = 0;  cur.battery = cfg.battery_start;
    cur.sun_angle = sunAt(0);
    cur.light_level = map.lightLevel(cur.x, cur.y, cur.sun_angle);
    m.executed.push_back(cur);
    m.min_battery = cur.battery;
    if (!full_knowledge) reveal(known, m.reveal_step, cur.x, cur.y, cfg.obs_radius, 0);

    std::vector<std::pair<int, int>> prev_route;
    PlanResult plan;
    size_t plan_cursor = 0;  // full-knowledge mode: index into the single plan
    const int max_steps = 20 * W * H;

    for (int step = 0; step < max_steps; step++) {
        if (cur.x == cfg.goal_x && cur.y == cfg.goal_y) {
            m.success = true;
            break;
        }

        // ── Plan ──
        if (!full_knowledge || step == 0) {
            auto t0 = std::chrono::steady_clock::now();
            plan = planPath(map, cfg, SearchStart{cur.x, cur.y, cur.time, cur.battery},
                            full_knowledge ? nullptr : &known);
            auto t1 = std::chrono::steady_clock::now();
            m.planning_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
            m.total_expansions += plan.expansions;
            if (!plan.success) {
                m.failure_reason = "stranded: " + plan.failure_reason;
                break;
            }
            plan_cursor = 0;

            auto r = route(plan);
            // Did new information change where the rover intends to go?
            if (!prev_route.empty()) {
                std::vector<std::pair<int, int>> expected(prev_route.begin() + 1,
                                                          prev_route.end());
                if (expected != r) m.route_changes++;
            }
            prev_route = r;
        }
        {
            std::vector<std::pair<int, int>> remaining;
            for (size_t i = plan_cursor; i < plan.path.size(); i++)
                remaining.emplace_back(plan.path[i].x, plan.path[i].y);
            m.plans.push_back(std::move(remaining));
        }

        // ── Act: execute the first action against the TRUE terrain ──
        const PathStep& target = plan.path[plan_cursor + 1];
        PathStep nxt;
        nxt.x = target.x;  nxt.y = target.y;
        nxt.waited = target.waited;
        double rate;
        if (nxt.waited) {
            nxt.dt = cfg.wait_sec;
            nxt.time = cur.time + nxt.dt;
            nxt.sun_angle = sunAt(nxt.time);
            nxt.light_level = map.lightLevel(nxt.x, nxt.y, nxt.sun_angle);
            rate = nxt.light_level * cfg.max_gain_per_sec - cfg.idle_drain_per_sec;
            m.waits++;
        } else {
            const bool diag = (nxt.x != cur.x) && (nxt.y != cur.y);
            nxt.dt = map.cells[nxt.y][nxt.x].base_traverse_cost * (diag ? M_SQRT2 : 1.0);
            nxt.time = cur.time + nxt.dt;
            nxt.sun_angle = sunAt(nxt.time);
            nxt.light_level = map.lightLevel(nxt.x, nxt.y, nxt.sun_angle);
            rate = nxt.light_level * cfg.max_gain_per_sec - cfg.drain_per_sec;
        }
        nxt.battery = std::min(cfg.battery_max, cur.battery + rate * nxt.dt);
        m.executed.push_back(nxt);
        m.min_battery = std::min(m.min_battery, nxt.battery);
        cur = nxt;
        plan_cursor++;

        if (cur.battery < cfg.battery_min) {
            m.failure_reason = "battery depleted";
            break;
        }

        // ── Sense ──
        if (!full_knowledge)
            reveal(known, m.reveal_step, cur.x, cur.y, cfg.obs_radius, step + 1);
    }

    if (!m.success && m.failure_reason.empty()) m.failure_reason = "step limit reached";
    m.total_time = cur.time;
    m.final_battery = cur.battery;
    return m;
}
