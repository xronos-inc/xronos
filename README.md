# Xronos

The Xronos library for writing reactor programs.

## Resources

Github: https://github.com/xronos-inc/xronos

Documentation: https://xronos-inc.github.io/xronos

pypi: https://pypi.org/project/xronos

## Prerequisites

Architectures:
- amd64
- arm64

Platforms:
- Ubuntu 22.04
- Ubuntu 24.04

Python:
- Python 3.10
- Python 3.11
- Python 3.12

## Installation

We recommend working in a virtual environment. Use the following commands to create and load a new environment `.venv`.

```shell
python -m venv .venv
source .venv/bin/activate
```

Install the `xronos` pip packge:

```shell
pip install xronos
```

## Hello World

This section shows you how to write a hello world program using xronos. Create a file called `hello.py` and paste the following content into it.

```python
import xronos

class Hello(xronos.Reactor):
    @xronos.reaction
    def hello(self, interface):
        interface.add_trigger(self.startup)
        return lambda: print(f"Hello, world!")

env = xronos.Environment()
env.create_reactor("hello", Hello)
env.execute()
```

If you see the following output, the library is installed successfully.

```shell
$ python hello.py
Hello, world!
```
