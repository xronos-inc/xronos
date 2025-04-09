# Xronos

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

For Python users, the [xronos PyPi package](https://pypi.org/project/xronos/) is published as a standalone Python
package. Building from source is not required.

- **Python users**: Get started at [docs.xronos.com](https://docs.xronos.com)
- **C++ users**: See [docs.xronos.com/cpp](https://docs.xronos.com/cpp)
- **VS Code users**: The [Xronos VS Code Extension](https://marketplace.visualstudio.com/items?itemName=xronos.xronos) is optional and enables diagrams of your programs

## Repository Structure

```bash
.
├── base-images/:   # Earthly base images
├── cpp-sdk/:       # C++ SDK
├── docs/           # source for https://docs.xronos.com
├── examples/:      # python SDK examples
├── lib/            # library components
├── third-party/    # third-party libraries
├── xronos/         # source for the xronos framework
├── Earthfile       # earthly build definitions
└── README.md       # project overview (this file)
```

## Building from Source

To build locally:

1. [Install Docker](https://docker.com)
2. [Install Earthly](https://earthly.dev/get-earthly)
3. Run:

   ```bash
   earthly +build
   ```

## Links

- Website: [https://xronos.com](https://xronos.com)  
- Python Package: [https://pypi.org/project/xronos](https://pypi.org/project/xronos/)  
- Documentation: [https://docs.xronos.com](https://docs.xronos.com)  
- C++ Docs: [https://docs.xronos.com/cpp](https://docs.xronos.com/cpp)
