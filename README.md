# Xronos

[![GitHub Release](https://img.shields.io/github/v/release/xronos-inc/xronos?logo=GitHub)](https://github.com/xronos-inc/xronos/releases/latest)
[![PyPI - Version](https://img.shields.io/pypi/v/xronos?logo=Python&logoColor=silver)](https://pypi.org/project/xronos/)
[![GitHub License](https://img.shields.io/github/license/xronos-inc/xronos)](https://github.com/xronos-inc/xronos/blob/main/LICENSE)
[![PyPI - Python Version](https://img.shields.io/pypi/pyversions/xronos?logo=Python&logoColor=silver)](https://pypi.org/project/xronos/)
[![Static Badge](https://img.shields.io/badge/docs-blue?logo=Read%20the%20Docs&logoColor=white)](https://docs.xronos.com/)

**Xronos** is a lightweight framework for building deterministic, component-based robotics applications in Python or C++.
It supports real-time telemetry, system visualization, and modular design with a focus on clarity, scalability, and correctness.

## Features

- 🦾 **Turnkey Telemetry**: Built-in support for monitoring, tracing, and timing.
- ⚙️ **Modern APIs**: Intuitive and consistent interfaces in both Python and C++.
- 🪶 **Lightweight**: 20MB pip install; no OS version lock-in.
- 🧩 **Modular Architecture**: Component-based design for separation of concerns.
- ⏱️ **Deterministic Concurrency**: Reactor-based execution model guarantees predictable behavior.
- 🧠 **Automatic System Visualization**: Architecture diagrams generated from code.

## Usage

- **Python users**: See [docs.xronos.com/python\_sdk/getting\_started.html](https://docs.xronos.com/python_sdk/getting_started.html)
- **C++ users**: See [docs.xronos.com/cpp\_sdk/getting\_started.html](https://docs.xronos.com/cpp_sdk/getting_started.html)
- **VS Code users**: The optional [Xronos VS Code Extension](https://marketplace.visualstudio.com/items?itemName=xronos.xronos) renders diagrams of your programs

## Repository Structure

```bash
.
├── cpp-sdk/        # C++ SDK
├── docker-bake.hcl # build definitions
├── docs/           # source for https://docs.xronos.com
├── examples/       # python SDK examples
├── lib/            # core Xronos libraries
├── third-party/    # third-party libraries
├── xronos/         # Python SDK
└── README.md       # project overview (this file)
```

## Building from Source


We use [docker bake](https://docs.docker.com/build/bake/) as our top-level build
tool. To build locally, install [docker](https://docker.com) and the [buildx
extension](https://github.com/docker/buildx) (if not included in your docker
installation). Then run:

```bash
docker buildx bake build
```

## Links

- Documentation: [https://docs.xronos.com](https://docs.xronos.com)
- Company website: [https://xronos.com](https://xronos.com)
- Python package: [https://pypi.org/project/xronos](https://pypi.org/project/xronos/)
