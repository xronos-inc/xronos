#!/bin/bash

set -euo pipefail

tmpdir=$(mktemp -d)

NUM_ITERATIONS=25

python -m venv $tmpdir/.venv
source $tmpdir/.venv/bin/activate
pip install xronos

duration_accumulated=0

for i in $(seq 1 $NUM_ITERATIONS); do
    start=$(date +%s.%N)
    python hello.py
    end=$(date +%s.%N)
    duration=$(python3 -c "print($end - $start)")
    duration_accumulated=$(python3 -c "print($duration_accumulated + $duration)")
done


average_duration=$(python3 -c "print(f'{$duration_accumulated / $NUM_ITERATIONS:.5f}')")

echo ""
echo "Number of runs: $NUM_ITERATIONS"
echo "Number of errors: 0"
echo "Average execution time: ${average_duration}s"
