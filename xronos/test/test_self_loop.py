import datetime
from typing import Callable

import xronos

"""
This tests a self contained reactor that reacts to itself.
It is stopped from within a reactor using the stop
"""


class SelfLoop(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()
    output_ = xronos.OutputPortDeclaration[int]()
    internal_event = xronos.ProgrammableTimerDeclaration[int]()

    # an arbitrary value used for testing
    start_at_count: int = 40
    expected: int = start_at_count + 1
    kill_at_count: int = start_at_count + 10

    @xronos.reaction
    def on_internal_event(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        i_event = interface.add_trigger(self.internal_event)
        output_ = interface.add_effect(self.output_)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            output_.set(i_event.get() + 1)

        return handler

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)
        i_event = interface.add_effect(self.internal_event)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            i_event.schedule(value=input_.get())
            assert input_.get() == self.expected
            self.expected += 1

            if self.expected >= self.kill_at_count:
                self.environment.request_shutdown()
            else:
                i_event.schedule(value=input_.get())

        return handler

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        i_event = interface.add_effect(self.internal_event)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            i_event.schedule(value=self.start_at_count)

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            assert self.expected >= self.kill_at_count

        return handler


def main() -> None:
    env = xronos.Environment(timeout=datetime.timedelta(microseconds=200))
    selfloop = env.create_reactor("selfloop", SelfLoop)
    env.connect(selfloop.output_, selfloop.input_)
    env.execute()


def test_self_loop() -> None:
    main()


if __name__ == "__main__":
    main()
