# Xronos C++ SDK Doxygen

## Install Dependencies

### Install doxygen

```shell
sudo apt update -q
sudo apt install -y bison flex
sudo apt install -y libz-dev libzstd-dev libcurl4-openssl-dev libedit-dev
git clone --branch Release_1_13_2 https://github.com/doxygen/doxygen
cd doxygen
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release -DENABLE_CLANG_TIDY=OFF -Duse_libclang=ON
cmake --build build --config Release
cmake --build build --target install --config Release
doxygen --version
```

### Install dependencies

- doxygen
- graphviz (graphs and diagrams)
- python3-pygments (syntax highlighting)
- global (clickable references in source browser)

```shell
sudo apt install graphviz python3-pygments global
```

## Generate Doxygen Documentation

### Build the C++ SDK

From the `cpp-sdk` folder, build the sdk followed by the `docs` target:

```shell
cmake -S . -B build -DXRONOS_SDK_BUILD_DOCS=ON
cmake --build build --target all
make --build build --target docs
```

### View the C++ SDK API

```shell
(cd build/docs/html && python3 -m http.server 8000)
```

## Doxygen Configuration: C++ SDK vs. Runtime

The Doxygen configuration reflects the separation between the **C++ runtime** and the **C++ SDK**. The public API documentation is generated entirely from the `cpp-sdk` API, ensuring it remains independent of the underlying runtime. This approach helps keep the API documentation **focused** and **implementation-agnostic**.

### Build Options

- **`XRONOS_SDK_BUILD_DOCS`**  
  Enables the `make` target **`doxygen-cpp-api`** for generating C++ SDK documentation.  
  **Default:** `OFF`
