# Xronos C++ SDK

The Xronos C++ SDK is a programmatic interface to the Xronos library, allowing applications
to be built using reactors and executed using the Xronos runtime.

## System Requirements

- Ubuntu 22.04 or later
- amd64 or arm64 architecture
- GNU-compliant C++ compiler that supports the C++20 standard
  - gcc 11 or later
  - clang 16 or later
- CMake 3.28 or later

See [docs.xronos.com](https://docs.xronos.com) for the most up-to-date system requirements.

## Install Prerequisites

### Install CMake

Ubuntu 24.04 (Noble) apt repositories include CMake 3.28 or later. If on an earlier
version of Ubuntu, first add the Kitware apt repository:

```shell
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -sc) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null
sudo apt update
```

Install CMake:

```shell
sudo apt install cmake
```

### Install gRPC

Install gRPC (1.30 or later):

```shell
sudo apt install libgrpc++-dev
```

## Create a Xronos C++ application

### Create the Hello World application

In a new directory, create the following source file `hello_world.cc`:

```cpp
#include "xronos/sdk.hh"

namespace sdk = xronos::sdk;

class HelloWorld : public sdk::Reactor {
  using sdk::Reactor::Reactor;

  class HelloWorldReaction : public sdk::Reaction<HelloWorld> {
    using sdk::Reaction<HelloWorld>::Reaction;
    Trigger<void> startup_trigger{self().startup(), context()};
    void handler() final { std::cout << "Hello, World!\n"; }
  };

  void assemble() final { add_reaction<HelloWorldReaction>("hello"); }
};

auto main() -> int {
  sdk::Environment env{};
  HelloWorld hello_world{"hello_world", env.context()};
  env.execute();
  return 0;
}
```

Then create the CMake project in `CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.28)

project(hello_world LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20 CACHE STRING "The C++ standard." FORCE)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  "xronos-sdk"
  GIT_REPOSITORY https://github.com/xronos-inc/xronos
  GIT_TAG main
  SOURCE_SUBDIR cpp-sdk
)
FetchContent_MakeAvailable(xronos-sdk)

add_executable(hello_world hello_world.cc)
target_link_libraries(hello_world xronos::xronos-sdk)
```

### Build and Run

Generate the CMake project and build the Xronos SDK:

```shell
cmake -B build
cmake --build build --target xronos-sdk -j
```

then compile and run the hello world application:

```shell
cmake --build build --target hello_world
./build/hello_world
```

You should see the output

```shell
> Hello, World!
```

## Additional Examples

See [https://github.com/xronos-inc/xronos/cpp-sdk/examples](https://github.com/xronos-inc/xronos/cpp-sdk/examples) for additional examples.
