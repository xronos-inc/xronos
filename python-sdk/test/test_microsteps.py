import datetime
from typing import Callable

import xronos

"""
This test emits two different outputs from a reactor
at 0ms intervals, effectively dispatching the two outputs
at the "same" physical time, though one step
later in logical times.

We want to assert that the receiving reactor
only receives one at a time by testing with is_present
"""


class DoubleEmittingSource(xronos.Reactor):
    output_x = xronos.OutputPortDeclaration[int]()
    output_y = xronos.OutputPortDeclaration[int]()
    internal_event = xronos.ProgrammableTimerDeclaration[None]()

    @xronos.reaction
    def emit_on_start(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        ctx.add_trigger(self.startup)
        output_x = ctx.add_effect(self.output_x)
        internal_event = ctx.add_effect(self.internal_event)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            internal_event.schedule(value=None, delay=datetime.timedelta(0))
            output_x.set(1)

        return body

    @xronos.reaction
    def react_to_logical_action(
        self, ctx: xronos.ReactionContext
    ) -> Callable[[], None]:
        ctx.add_trigger(self.internal_event)
        output_y = ctx.add_effect(self.output_y)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            output_y.set(1)

        return body


class Receiver(xronos.Reactor):
    input_x = xronos.InputPortDeclaration[int]()
    input_y = xronos.InputPortDeclaration[int]()

    @xronos.reaction
    def react_to_input(self, ctx: xronos.ReactionContext) -> Callable[[], None]:
        input_x = ctx.add_trigger(self.input_x)
        input_y = ctx.add_trigger(self.input_y)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            present_count: int = 0
            if input_x.is_present():
                present_count += 1
            if input_y.is_present():
                present_count += 1

            assert present_count == 1

        return body


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(seconds=5), fast=fast)
    source = env.create_reactor("source", DoubleEmittingSource)
    receiver = env.create_reactor("receiver", Receiver)
    env.connect(source.output_x, receiver.input_x)
    env.connect(source.output_y, receiver.input_y)
    env.execute()


def test_microsteps() -> None:
    main(fast=True)


if __name__ == "__main__":
    main()
