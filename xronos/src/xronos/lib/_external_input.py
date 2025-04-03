# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import threading
from collections.abc import Generator
from typing import Callable, Generic, TypeVar, final

import xronos

T = TypeVar("T")


class ExternalInput(xronos.Reactor, Generic[T]):
    """A reactor that reads inputs from an external source.

    This reactor can perform blocking I/O operations to read inputs from external
    sources such as files, network sockets, stdin, etc. The external inputs are
    scheduled on a :class:`xronos.PhysicalEvent` and forwarded to the output port.

    The reactor reads inputs using the provided :func:`read_input` generator. This
    generator should perform the necessary setup and teardown operations and yield
    the input values read from the external source.

    Two simple example generators are given below.

    .. tabs::

        .. code-tab:: py Using context management

            def read_input() -> Generator[str]:
                with open("input.txt") as f:
                    yield from f

        .. code-tab:: py Using explicit setup and teardown

            def read_input() -> Generator[str]:
                f = open("input.txt")
                try:
                    for line in f:
                        yield line
                finally:
                    f.close()


    Args:
        read_input: A generator that reads inputs from an external source and yields
            them. The generator is expected to perform the necessary setup and
            teardown operations. See the example above.
    """

    #: OutputPort[T]: Emits the read inputs as events.
    output = xronos.OutputPortDeclaration[T]()
    __external_input_event = xronos.PhysicalEventDeclaration[T]()
    # If an unhandeled exception occurs in the read_input generator, it is caught and
    # scheduled as a physical event and raised within the runtime.
    __external_exception_event = xronos.PhysicalEventDeclaration[Exception]()

    # The timeout for the thread to join when the reactor is shutting down.
    JOIN_THREAD_TIMEOUT_S = 1

    def __init__(
        self,
        read_input: Generator[T],
    ) -> None:
        super().__init__()
        self.__read_input = read_input
        self.__stop_thread_event = threading.Event()
        self.__thread = threading.Thread(
            target=self.__run_user_input_thread, daemon=True
        )

    @final
    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Start the user input thread."""
        interface.add_trigger(self.startup)

        def handler() -> None:
            self.__thread.start()

        return handler

    @final
    @xronos.reaction
    def on_external_input_event(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Forward the external event to the output port."""
        external_input = interface.add_trigger(self.__external_input_event)
        output_effect = interface.add_effect(self.output)

        def handler() -> None:
            output_effect.set(external_input.get())

        return handler

    @final
    @xronos.reaction
    def on_external_exception_event(
        self, interface: xronos.ReactionInterface
    ) -> Callable[[], None]:
        """Raise the exception within the runtime."""
        external_exception_trigger = interface.add_trigger(
            self.__external_exception_event
        )

        def handler() -> None:
            raise external_exception_trigger.get()

        return handler

    @final
    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Stop the user input thread."""
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            self.__stop_thread_event.set()
            self.__thread.join(timeout=self.JOIN_THREAD_TIMEOUT_S)

        return handler

    @final
    def __run_user_input_thread(self) -> None:
        """Read external inputs and schedule them as events.

        This method is run in a separate thread and calls the provided read_input
        generator. A `threading.Event` is used to signal the thread to stop. If
        an unhandled exception occurs in the read_input generator, it is caught and
        scheduled as a physical event and further raised within the runtime.

        """
        try:
            for external_input in self.__read_input:
                if self.__stop_thread_event.is_set():
                    # The stop event signals that the reactor is shutting down.
                    # So the last read value is discarded and the thread is stopped.
                    break

                self.__external_input_event.trigger(external_input)
        except Exception as e:
            print(
                f"ERROR: Exception: `{e}` caught in `read_input` generator of "
                f"{self.fqn}. Forwarding the exception to the runtime."
            )
            self.__external_exception_event.trigger(e)

        finally:
            self.__read_input.close()


class ConsoleInput(ExternalInput[T]):
    """A reactor that reads and parses lines of text from `stdin`.

    It is derived from :class:`ExternalInput` and provides it a generator that read
    Reads lines of text from `stdin`, applies the provided parser function and sends the
    results via the :attr:`output` port. If the parser function raises a
    :exc:`ValueError`, the input is discarded and the reactor will continue reading
    inputs. If the parser raises an :exc:`~ConsoleInput.RequestShutdown` exception, the
    reactor will stop reading inputs and request a shutdown.

    A simple example parser function that parses integers and a exit keyword from the
    console input is given below::

        def simple_parser(x: str) -> int:
            if x == "exit":
                raise ConsoleInput.RequestShutdown
            else:
                return int(x)

    Args:
        parser: A function that parses the console input and returns the
            parsed value. Raise :exc:`ValueError` to discard the input, and raise
            :exc:`~ConsoleInput.RequestShutdown` to shutdown the program.

    Attributes:
        output(xronos.OutputPort[T]): A port that forwards the parsed input.
    """

    def __init__(
        self,
        parser: Callable[[str], T],
    ):
        super().__init__(read_input=self.__read_console_input())
        self.parser = parser

    class RequestShutdown(Exception):
        """An exception that can be raised by the parser to request a shutdown."""

        pass

    def __read_console_input(self) -> Generator[T]:
        """Read lines of text from `stdin` and parse them using the provided parser.

        The generator will read lines of text from `stdin` until EOFError is raised.
        Lines are parsed with the provided `{func}:parser` function. If the parser
        raises a :exc:`ValueError`, the input is discarded and the generator will
        continue reading inputs. If the parser raises a StopIteration exception, the
        generator will stop reading inputs and request a shutdown.

        Yields:
            T: The parsed value from the console input.

        Returns:
            None
        """
        try:
            while True:
                console_input = input("> ")
                try:
                    yield self.parser(console_input)
                except ValueError:
                    pass
                except ConsoleInput.RequestShutdown:
                    self.environment.request_shutdown()
                    break
        except EOFError:
            pass

        return
