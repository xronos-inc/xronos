import datetime
from typing import Callable

import xronos

"""

This test tests a looped action with breaks.

We aim to test that:
1) Logical actions can react to themselves.
2) Reactions can have multiple side effects. In this case:
    a) setting an output
    b) scheduling a new action
3) Within this loop, we look to ensure that delayed dispatches correctly update
    logical time.

"""


class ActionSender(xronos.Reactor):
    internal_event = xronos.ProgrammableTimerDeclaration[None]()

    output_ = xronos.OutputPortDeclaration[int]()

    def __init__(
        self,
        messages_until_break: int,
        length_of_break: datetime.timedelta,
    ):
        super().__init__()

        self.messages_until_break = messages_until_break
        self.length_of_break = length_of_break
        self.messages_sent_in_current_batch = 0

    @xronos.reaction
    def schedule_next(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        interface.add_trigger(self.internal_event)
        new_action = interface.add_effect(self.internal_event)
        output_ = interface.add_effect(self.output_)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            if self.messages_sent_in_current_batch < self.messages_until_break - 1:
                output_.set(self.messages_sent_in_current_batch)
                self.messages_sent_in_current_batch += 1
                new_action.schedule(value=None, delay=datetime.timedelta(0))
            else:
                output_.set(self.messages_sent_in_current_batch)
                self.messages_sent_in_current_batch = 0
                new_action.schedule(value=None, delay=self.length_of_break)

        return body


class ActionReceiver(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()

    def __init__(
        self,
        messages_until_break: int,
        length_of_break: datetime.timedelta,
    ):
        super().__init__()
        self.delays_recorded = 0
        self.length_of_break = length_of_break
        self.messages_until_break = messages_until_break

    @xronos.reaction
    def on_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            # Elapsed logical time is the same in messages of the same batch.
            # We test that for each message in the batch, elapsed time is
            # equal to the length of break * number of breaks taken.

            assert (
                self._get_elapsed_logical_time().microseconds
                / self.length_of_break.microseconds
            ) == self.delays_recorded

            if input_.get() == self.messages_until_break - 1:
                self.delays_recorded += 1

        return body


def main(fast: bool = False) -> None:
    env = xronos.Environment(timeout=datetime.timedelta(milliseconds=100), fast=fast)
    length_of_break = datetime.timedelta(milliseconds=10)

    action_sender = env.create_reactor(
        "action_sender", ActionSender, 5, length_of_break
    )
    action_receiver = env.create_reactor(
        "action_receiver", ActionReceiver, 5, length_of_break
    )

    env.connect(action_sender.output_, action_receiver.input_)
    env.execute()


def test_looped_action() -> None:
    main(True)


if __name__ == "__main__":
    main()
