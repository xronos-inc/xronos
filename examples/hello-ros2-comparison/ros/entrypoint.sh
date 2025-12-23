#!/bin/bash

source /opt/ros/rolling/setup.bash

colcon build --packages-select hello

source ./install/setup.bash

set -euo pipefail

NUM_ITERATIONS=50

rm -f out.txt

error_count=0
duration_count=0
duration_accumulated=0

for i in $(seq 1 $NUM_ITERATIONS); do
  start=$(date +%s.%N)
  ros2 launch hello launch.py | tee out.txt
  end=$(date +%s.%N)
  if grep -q "printer did not receive" out.txt; then
      error_count=$((error_count + 1))
  else
      duration=$(python3 -c "print($end - $start)")
      duration_accumulated=$(python3 -c "print($duration_accumulated + $duration)")
      duration_count=$((duration_count + 1))
  fi
done

if [ "$duration_count" -eq 0 ]; then
  average_duration="N/A"
else
  average_duration=$(python3 -c "print(f'{$duration_accumulated / $duration_count:.5f}')")
fi

echo ""
echo "Number of runs: $NUM_ITERATIONS"
echo "Number of errors: $error_count"
echo "Average execution time: ${average_duration}s"
