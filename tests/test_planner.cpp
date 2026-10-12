// Planner tests. Run with `ctest --test-dir build` or ./build/test_planner.
#include "map.h"
#include "mission.h"
#include "planner.h"
#include <cmath>
#include <cstdio>
#include <string>

static int failures = 0;
#define CHECK(cond, msg)                                                    \
    do {                                                                    \
        if (!(cond)) {                                                      \
            std::printf("  FAIL %s:%d  %s\n", __FILE__, __LINE__, msg);    \
            failures++;                                                     \
        }                                                                   \
    } while (0)

// Flat, shadow-free map where every cell costs exactly `cost` seconds.
static LunarMap flatMap(int W, int H, double cost = 1.0) {
    MapGenConfig g;
    g.crater_count_min = g.crater_count_max = 0;
    g.dune_count_min = g.dune_count_max = 0;
    g.rocky_count_min = g.rocky_count_max = 0;
    LunarMap m = buildMap(W, H, 1, g);
    for (auto& row : m.cells)
        for (auto& c : row) c.base_traverse_cost = cost;
    return m;
}

static double octile(int dx, int dy) {
    dx = std::abs(dx); dy = std::abs(dy);
    return std::max(dx, dy) + (std::sqrt(2.0) - 1.0) * std::min(dx, dy);
}

static RoverConfig demoConfig(int W, int H) {
    RoverConfig c;
    c.start_x = 2;     c.start_y = H / 2;
    c.goal_x = W - 3;  c.goal_y = H / 2;
    return c;  // struct defaults = README demo parameters
}

static MapGenConfig defaultGen() { return MapGenConfig{}; }

static void testShortestPathOnFlatMap() {
    std::printf("flat map: plan time equals octile distance\n");
    LunarMap m = flatMap(30, 30, 1.0);
    RoverConfig c;
    c.battery_start = 1.0;
    c.start_x = 3;  c.start_y = 4;  c.goal_x = 25;  c.goal_y = 17;
    PlanResult r = planPath(m, c);
    CHECK(r.success, "plan should succeed");
    CHECK(std::fabs(r.total_time - octile(22, 13)) < 1e-9, "time must equal octile distance");
}

static void testUnknownCellsAreOptimistic() {
    std::printf("fog: unseen cells are planned at unknown_cost\n");
    LunarMap m = flatMap(30, 30, 5.0);  // truly slow everywhere
    RoverConfig c;
    c.battery_start = 1.0;
    c.start_x = 1;  c.start_y = 1;  c.goal_x = 20;  c.goal_y = 1;
    KnownGrid known(30, std::vector<char>(30, 0));
    PlanResult r = planPath(m, c, SearchStart{1, 1, 0.0, 1.0}, &known);
    CHECK(r.success, "plan should succeed");
    CHECK(std::fabs(r.total_time - 19.0 * c.unknown_cost) < 1e-9,
          "unknown cells must cost unknown_cost");
}

static void testParkingWhenDrivingAloneCannotReachGoal() {
    std::printf("battery: parks to recharge when driving alone would run out\n");
    LunarMap m = flatMap(20, 5, 1.0);  // full sun everywhere
    RoverConfig c;
    c.start_x = 0;  c.start_y = 2;  c.goal_x = 15;  c.goal_y = 2;
    c.battery_min = 0.15;
    c.battery_start = 0.16;  // driving 15 s costs 15 * 0.002 = 0.03 > 0.01 margin

    RoverConfig no_wait = c;
    no_wait.allow_wait = false;
    CHECK(!planPath(m, no_wait).success, "without parking the goal is unreachable");

    PlanResult r = planPath(m, c);
    CHECK(r.success, "with parking the goal is reachable");
    int waits = 0;
    for (const auto& s : r.path) waits += s.waited;
    CHECK(waits > 0, "plan should contain at least one recharge stop");
    for (const auto& s : r.path)
        CHECK(s.battery >= c.battery_min - 1e-12, "battery must stay above minimum");
}

static void testBatteryNeverBelowMinimumOnRandomMaps() {
    std::printf("battery: aware missions never drop below the minimum (40 maps, fog)\n");
    for (unsigned seed = 1; seed <= 40; seed++) {
        LunarMap m = buildMap(60, 60, seed, defaultGen());
        RoverConfig c = demoConfig(60, 60);
        MissionResult r = runMission(m, c, false);
        CHECK(r.success, ("mission should succeed, seed " + std::to_string(seed)).c_str());
        CHECK(r.min_battery >= c.battery_min - 1e-12,
              ("battery below minimum, seed " + std::to_string(seed)).c_str());
    }
}

static void testFogOfWarIsHonest() {
    std::printf("fog: the rover only enters cells it has already observed\n");
    LunarMap m = buildMap(60, 60, 86, defaultGen());
    RoverConfig c = demoConfig(60, 60);
    MissionResult r = runMission(m, c, false);
    CHECK(r.success, "demo mission should succeed");
    for (size_t k = 1; k < r.executed.size(); k++) {
        const auto& s = r.executed[k];
        const int seen = r.reveal_step[s.y][s.x];
        CHECK(seen >= 0 && seen <= static_cast<int>(k) - 1,
              "cell entered before it was observed");
    }
    for (size_t k = 0; k < r.plans.size(); k++)
        CHECK(r.plans[k].front() == std::make_pair(r.executed[k].x, r.executed[k].y),
              "each plan must start at the rover's current cell");
    int never_seen = 0;
    for (const auto& row : r.reveal_step)
        for (int v : row) never_seen += (v < 0);
    CHECK(never_seen > 0, "a partial-knowledge mission should leave some cells unseen");
}

static void testBaselineFailsWhereAwareSucceeds() {
    std::printf("demo map: time-only planner runs out of charge, battery-aware does not\n");
    LunarMap m = buildMap(60, 60, 86, defaultGen());
    RoverConfig aware = demoConfig(60, 60);
    RoverConfig blind = aware;
    blind.battery_aware = false;
    MissionResult rb = runMission(m, blind, false);
    MissionResult ra = runMission(m, aware, false);
    CHECK(!rb.success && rb.failure_reason == "battery depleted", "baseline should run out");
    CHECK(ra.success, "battery-aware rover should reach the goal");
}

static void testSameAnswerWhenBatteryIsNotBinding() {
    std::printf("full battery, gentle drain: aware and time-only plans take equal time\n");
    for (unsigned seed = 1; seed <= 10; seed++) {
        LunarMap m = buildMap(60, 60, seed, defaultGen());
        RoverConfig c = demoConfig(60, 60);
        c.battery_start = 1.0;
        c.drain_per_sec = 0.001;
        RoverConfig b = c;
        b.battery_aware = false;
        PlanResult ra = planPath(m, c), rb = planPath(m, b);
        CHECK(ra.success && rb.success, "both should succeed");
        CHECK(std::fabs(ra.total_time - rb.total_time) < 1e-9,
              "with slack battery, both planners must find the same optimal time");
    }
}

int main() {
    testShortestPathOnFlatMap();
    testUnknownCellsAreOptimistic();
    testParkingWhenDrivingAloneCannotReachGoal();
    testBatteryNeverBelowMinimumOnRandomMaps();
    testFogOfWarIsHonest();
    testBaselineFailsWhereAwareSucceeds();
    testSameAnswerWhenBatteryIsNotBinding();
    if (failures) {
        std::printf("\n%d check(s) failed\n", failures);
        return 1;
    }
    std::printf("\nall tests passed\n");
    return 0;
}
