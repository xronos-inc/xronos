from typing import Callable

import xronos

"""
This is a basic composition test that passes a source output
though a gain reactor (multiplying by a scalar) before sending to receiver.

Unneeded components added to test dangling outputs:
1. Add dangling outputs to Scale and Receiver Reactor
2. Add dangling input to the Scale Reactor
"""


SCALAR: int = 2


class Scale(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()
    input_dangling = xronos.InputPortDeclaration[int]()
    output_ = xronos.OutputPortDeclaration[int]()
    output_dangling = xronos.OutputPortDeclaration[int]()

    def __init__(
        self,
    ) -> None:
        super().__init__()

    @xronos.reaction
    def scale(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)
        output_ = interface.add_effect(self.output_)
        output_dangling = interface.add_effect(self.output_dangling)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            output_.set((input_.get() or 0) * SCALAR)
            output_dangling.set((input_.get() or 0) * SCALAR)

        return handler


class Source(xronos.Reactor):
    output_ = xronos.OutputPortDeclaration[int]()

    @xronos.reaction
    def emit(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        interface.add_trigger(self.startup)
        output_ = interface.add_effect(self.output_)

        def handler() -> None:
            print(f"Reaction in {self.fqn} triggered")
            output_.set(1)

        return handler


class Receiver(xronos.Reactor):
    input_ = xronos.InputPortDeclaration[int]()
    output_dangling = xronos.OutputPortDeclaration[int]()

    @xronos.reaction
    def receive(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        input_ = interface.add_trigger(self.input_)
        output_dangling = interface.add_effect(self.output_dangling)

        def body() -> None:
            print(f"Reaction in {self.fqn} triggered")
            output_dangling.set(1)
            assert input_.get() == SCALAR

        return body


def main() -> None:
    env = xronos.Environment()
    source = env.create_reactor("source", Source)
    scale = env.create_reactor("scale", Scale)
    receiver = env.create_reactor("receiver", Receiver)
    env.connect(source.output_, scale.input_)
    env.connect(scale.output_, receiver.input_)
    env.execute()


def test_dangling_gain() -> None:
    main()


if __name__ == "__main__":
    main()
