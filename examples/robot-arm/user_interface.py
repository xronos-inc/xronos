# SPDX-FileCopyrightText: Copyright (c) 2025 Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import threading
from typing import Callable, Generic, TypeVar

import xronos

T = TypeVar("T")


class UserInterface(xronos.Reactor, Generic[T]):
    """A user interface reactor that reads input from the console.

    Args:
        blocking: After parsing and outputting a command. If blocking is True, the
            reactor will not accept a new console input until it receives an event on
            the unblock input port. If blocking is False, the reactor will immediately
            accept new console inputs.
        parser: A user-provided function that parses the console input and returns
            either a parsed command to output, None if no command was parsed, or
            KeyboardInterrupt which indicates that the reactor should shut down.
    """

    unblock = xronos.InputPortDeclaration[bool]()
    output = xronos.OutputPortDeclaration[T]()
    _user_input = xronos.PhysicalEventDeclaration[str]()
    _unblock = xronos.ProgrammableTimerDeclaration[None]()

    def __init__(
        self, blocking: bool, parser: Callable[[str], T | None | KeyboardInterrupt]
    ):
        super().__init__()
        self.blocking = blocking
        self.parser = parser
        self.stop_thread_event = threading.Event()
        self.thread = threading.Thread(
            target=self.run_user_input_thread, args=(self.stop_thread_event,)
        )
        self.semaphore = threading.Semaphore(value=0)

    @xronos.reaction
    def on_startup(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Start the thread listening for console input."""
        interface.add_trigger(self.startup)

        def handler() -> None:
            # Print out the help message on startup.
            self.parser("help")
            self.thread.start()

        return handler

    @xronos.reaction
    def on_user_input(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Parse the user input and output any parsed command on the output port."""
        user_input_trigger = interface.add_trigger(self._user_input)
        output_effect = interface.add_effect(self.output)
        unblock_effect = interface.add_effect(self._unblock)
        shutdown_effect = interface.add_effect(self.shutdown)

        def handler() -> None:
            cmd = self.parser(user_input_trigger.get())
            if isinstance(cmd, KeyboardInterrupt):
                shutdown_effect.trigger_shutdown()
            elif cmd:
                output_effect.set(cmd)

            if not cmd or not self.blocking:
                # If the user interface is non-blocking, schedule an unblock event.
                unblock_effect.schedule(value=None)

        return handler

    @xronos.reaction
    def on_unblock(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Release the semaphore to unblock the user interface.

        Allows accepting new inputs from the conslole.
        """
        interface.add_trigger(self._unblock)
        interface.add_trigger(self.unblock)

        def handler() -> None:
            self.semaphore.release()

        return handler

    @xronos.reaction
    def on_shutdown(self, interface: xronos.ReactionInterface) -> Callable[[], None]:
        """Stop the running thread."""
        interface.add_trigger(self.shutdown)

        def handler() -> None:
            self.stop_thread_event.set()
            self.semaphore.release()

        return handler

    def run_user_input_thread(self, stop_event: threading.Event) -> None:
        """Listen for console input and schedule the next user input action.

        This function will run in a separate thread. After receiving an input
        it will block until the semaphore is released by the runtime.
        """
        while not stop_event.is_set():
            cmd = input("> ")
            self._user_input.trigger(cmd)
            self.semaphore.acquire()
