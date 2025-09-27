# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import datetime
import math
import random
import time
import uuid
from collections import deque
from threading import Lock
from typing import Callable, List, Optional, TypedDict

import xronos
from typing_extensions import NotRequired


class SimulationPoint(TypedDict):
    x: float
    y: float
    is_inside_circle: bool
    id: str  # used in the visualizer
    estimate_as_of_point: NotRequired[Optional[float]]


class SimulationRequest(TypedDict):
    total_points: int
    batch_size: int
    batch_delay: int  # milliseconds between batches
    enable_telemetry: bool


class PointQueue:
    """Used to interface xronos environment with the websocket publisher."""

    def __init__(self) -> None:
        self._queue = deque[SimulationPoint]()
        self._lock = Lock()
        self._shutdown = False

    def add_point(self, point: SimulationPoint) -> None:
        if not self._shutdown:
            with self._lock:
                self._queue.append(point)

    def get_point(self) -> Optional[SimulationPoint]:
        with self._lock:
            return self._queue.popleft() if self._queue else None

    def get_length(self) -> int:
        with self._lock:
            return len(self._queue)

    def request_shutdown(self) -> None:
        self._shutdown = True
        with self._lock:
            self._queue.clear()


class PointGenerator(xronos.Reactor):
    simulation_count = xronos.InputPortDeclaration[int]()
    generation_result = xronos.OutputPortDeclaration[List[SimulationPoint]]()

    @xronos.reaction
    def on_generation_request(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        simulation_count_trigger = interface.add_trigger(self.simulation_count)
        result_effect = interface.add_effect(self.generation_result)

        def handler() -> None:
            res: List[SimulationPoint] = []
            simulation_count = simulation_count_trigger.get()
            for _ in range(simulation_count):
                x = random.uniform(-1, 1)
                y = random.uniform(-1, 1)
                res.append(
                    {
                        "x": x,
                        "y": y,
                        "is_inside_circle": x * x + y * y <= 1,
                        "id": str(uuid.uuid4()),
                    }
                )
            result_effect.set(res)

        return handler


class SimulationAggregator(xronos.Reactor):
    batch_request = xronos.OutputPortDeclaration[int]()
    batch_completion = xronos.InputPortDeclaration[List[SimulationPoint]]()

    points_generated_in_batch = xronos.OutputPortDeclaration[List[SimulationPoint]]()
    estimate = xronos.OutputPortDeclaration[float]()

    _current_estimate = xronos.MetricDeclaration("The current estimate of Pi")
    _estimation_error = xronos.MetricDeclaration(
        "The current estimation error", unit="%"
    )
    _number_of_points = xronos.MetricDeclaration(
        "The current number of points generated for the estimation"
    )

    _batch_timer = xronos.PeriodicTimerDeclaration()

    def __init__(self, simulation_request: SimulationRequest):
        super().__init__()

        self._simulation_request = simulation_request

        period_ms = self._simulation_request["batch_delay"]
        assert period_ms > 0
        self._batch_timer.period = datetime.timedelta(milliseconds=period_ms)
        self._batch_timer.offset = datetime.timedelta(milliseconds=100)

        self.total_inside = 0
        self.total_generated = 0

    @xronos.reaction
    def on_batch_timer(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self._batch_timer)
        batch_request_effect = interface.add_effect(self.batch_request)

        def handler() -> None:
            batch_request_effect.set(self._simulation_request["batch_size"])

        return handler

    @xronos.reaction
    def on_batch_completion(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        batch_completion_trigger = interface.add_trigger(self.batch_completion)
        points_generated_in_batch_effect = interface.add_effect(
            self.points_generated_in_batch
        )
        estimate_effect = interface.add_effect(self.estimate)
        current_estimate_effect = interface.add_effect(self._current_estimate)
        estimation_error_effect = interface.add_effect(self._estimation_error)
        number_of_points_effect = interface.add_effect(self._number_of_points)
        shutdown_effect = interface.add_effect(self.shutdown)

        def handler() -> None:
            points_generated = [*batch_completion_trigger.get()]
            self.total_inside += len(
                [p for p in points_generated if p["is_inside_circle"]]
            )
            self.total_generated += len(points_generated)
            pi_estimate = 4 * self.total_inside / self.total_generated

            points_generated_in_batch_effect.set(points_generated)
            estimate_effect.set(pi_estimate)

            # record metrics
            current_estimate_effect.record(pi_estimate)
            estimation_error_effect.record(100 * (math.pi - pi_estimate) / math.pi)
            number_of_points_effect.record(self.total_generated)

            if self.total_generated >= self._simulation_request["total_points"]:
                shutdown_effect.trigger_shutdown()

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            pi_estimate = 4 * self.total_inside / self.total_generated
            print("Final estimate: ", pi_estimate)

        return handler


class WebSocketDispatcher(xronos.Reactor):
    points_generated_in_batch = xronos.InputPortDeclaration[List[SimulationPoint]]()
    estimate = xronos.InputPortDeclaration[float]()

    def __init__(self, queue: PointQueue) -> None:
        super().__init__()
        self.queue = queue
        self._current_estimate = 0.0

    @xronos.reaction
    def on_batch_or_estimate(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        points_generated_trigger = interface.add_trigger(self.points_generated_in_batch)
        estimate_trigger = interface.add_trigger(self.estimate)

        def handler() -> None:
            if points_generated_trigger.is_present():
                points_generated = points_generated_trigger.get()
                for point in points_generated:
                    self.queue.add_point(
                        {**point, "estimate_as_of_point": self._current_estimate}
                    )
            if estimate_trigger.is_present():
                self._current_estimate = estimate_trigger.get()
                print(f"estimate: {self._current_estimate}")

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            self.queue.request_shutdown()

        return handler


def run_sim(simulation_request: SimulationRequest, queue: PointQueue) -> None:
    start_time = int(time.time() * 1000)
    env = xronos.Environment()
    if simulation_request["enable_telemetry"]:
        env.enable_telemetry()
    simulation_aggregator = env.create_reactor(
        "SimulationAggregator",
        SimulationAggregator,
        simulation_request=simulation_request,
    )
    point_generator = env.create_reactor("PointGenerator", PointGenerator)
    env.connect(simulation_aggregator.batch_request, point_generator.simulation_count)
    env.connect(
        point_generator.generation_result, simulation_aggregator.batch_completion
    )

    if queue:
        websocket_dispatcher = env.create_reactor(
            "WebsocketDispatch", WebSocketDispatcher, queue=queue
        )
        env.connect(
            simulation_aggregator.points_generated_in_batch,
            websocket_dispatcher.points_generated_in_batch,
        )
        env.connect(simulation_aggregator.estimate, websocket_dispatcher.estimate)

    try:
        env.execute()
    except Exception as e:
        print(e)
        raise
    end_time = int(time.time() * 1000)
    print(f"Xronos execution took: {end_time - start_time} ms")


if __name__ == "__main__":
    run_sim(
        {
            "total_points": 1000000,
            "batch_size": 10000,
            "batch_delay": 1,
            "enable_telemetry": False,
        },
        queue=PointQueue(),
    )
