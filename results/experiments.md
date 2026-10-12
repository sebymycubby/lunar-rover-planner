Results on 200 random 60×60 maps (seeds 1–200), default parameters.

| Planner | Reached goal | Median time (s) | Mean recharge stops | Mean planning per replan (ms) |
|---|---|---|---|---|
| Time-only (baseline) | 110/200 (55%) | 74 | 0.0 | 0.12 |
| Battery-aware, no parking | 111/200 (56%) | 74 | 0.0 | 0.28 |
| Battery-aware | 200/200 (100%) | 88 | 1.6 | 0.37 |
| Battery-aware, full map | 200/200 (100%) | 78 | 0.2 | 9.5 (one plan) |

- The time-only planner lost the rover on **90/200** maps. The battery-aware planner reached the goal on **90** of those (3 by route choice alone, the rest also needed recharge stops).
- Where the baseline survived (110 maps), the battery-aware planner was exactly as fast on 103 of them: it only gives up time when it has to.
- With the whole map known, the battery-aware rover needed 0.2 recharge stops per mission on average, versus 1.6 under fog of war: seeing the terrain early lets it choose a sunlit route instead of parking.
- Exploring under fog of war took a median 8% longer than planning with the full map known.
