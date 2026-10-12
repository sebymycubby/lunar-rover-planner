#!/usr/bin/env bash
# Regenerate every figure in docs/media and results/ from scratch.
set -euo pipefail
cd "$(dirname "$0")/.."

cmake -S . -B build -DCMAKE_BUILD_TYPE=Release > /dev/null
cmake --build build -j > /dev/null
mkdir -p output docs/media results

# Demo 1 (default seed 86): route choice
./build/lunar_rover --quiet --blind --output output/demo_blind.json || true
./build/lunar_rover --quiet         --output output/demo_aware.json
python3 scripts/visualize.py animate-compare output/demo_blind.json output/demo_aware.json \
    --out docs/media/demo.gif
python3 scripts/visualize.py compare output/demo_blind.json output/demo_aware.json \
    --out docs/media/compare_route.png
python3 scripts/visualize.py overview output/demo_aware.json --out docs/media/overview.png

# Demo 2 (seed 77): parking to recharge
./build/lunar_rover --quiet --seed 77 --blind --output output/park_blind.json || true
./build/lunar_rover --quiet --seed 77         --output output/park_aware.json
python3 scripts/visualize.py compare output/park_blind.json output/park_aware.json \
    --out docs/media/compare_park.png

# Benchmarks
python3 scripts/experiments.py --seeds 200
python3 scripts/sweep.py --out docs/media/sweep.png
