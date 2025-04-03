# Xronos C++ SDK

Build the Xronos C++ SDK `xronos-sdk` and the Xronos runtime `xronos-lib`.

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

## Build `xronos-sdk`

Generate a new CMake project in the `build` directory and build the `xronos-sdk` target.
The `xronos-sdk` target will automatically build the Xronos runtime `xronos-lib` located
in this source repository.

```shell
cmake -B build
cmake --build build --target xronos-sdk -j
```

## Run the Hello World Example

Build and run the hello world example [examples/hello_world.cc](examples/hello_world.cc):

```shell
cmake -B build -DXRONOS_SDK_BUILD_EXAMPLES=ON
cmake --build build --target hello_world
./build/examples/hello_world
```

You should see the output

```shell
> Hello, World!
```
