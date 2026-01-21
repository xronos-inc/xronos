# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

from ..montecarlo import PointQueue, SimulationRequest, run_sim


def test_run_sim_point_count() -> None:
    """Test that run_sim generates the expected number of points."""

    # we need to override the queue class so it isn't cleared on shutdown
    class TestPointQueue(PointQueue):
        def request_shutdown(self) -> None:
            pass

    queue = TestPointQueue()
    expected_points = 2000
    simulation_request: SimulationRequest = {
        "total_points": expected_points,
        "batch_size": 1000,
        "batch_delay": 1,
        "enable_telemetry": False,
    }

    run_sim(simulation_request, queue)

    point_count = 0
    while True:
        point = queue.get_point()
        if point is None:
            break
        point_count += 1

    assert point_count == expected_points, (
        f"Expected {expected_points} points but got {point_count}"
    )
