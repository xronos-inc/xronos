# syntax=docker/dockerfile:1-labs

FROM scratch AS src
WORKDIR /xronos/cpp-sdk
COPY CMakeLists.txt Config.cmake.in LICENSE THIRD_PARTY_NOTICE .
COPY --parents cmake src include examples test third-party-licenses .

FROM base AS common
WORKDIR /xronos
COPY --from=configs /.clang-format /.clang-tidy .
WORKDIR /xronos/cpp-sdk
COPY --from=src /xronos/cpp-sdk .

FROM common AS cmake-init
COPY --from=lib /install /install-lib
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DCMAKE_PREFIX_PATH=/install-lib -DXRONOS_SDK_BUILD_TESTS=OFF -DXRONOS_SDK_BUILD_DOCS=OFF

FROM cmake-init AS lint
RUN cmake --build build --target xronos-sdk-check-format
RUN cmake --build build --target xronos-sdk-lint

FROM cmake-init AS build
RUN cmake --build build --target all -j$(nproc) --config Release
RUN cmake --install build --strip

FROM scratch AS install
COPY --from=build /install /install

FROM build AS pkgs-builder
WORKDIR /xronos/cpp-sdk/build
RUN cpack

FROM scratch AS pkgs
COPY --from=pkgs-builder /xronos/cpp-sdk/pkgs /pkgs

FROM build AS test
COPY --from=third-party-files /third-party/googletest ../third-party/googletest
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DXRONOS_SDK_BUILD_TESTS=ON -DXRONOS_SDK_BUILD_DOCS=OFF
RUN cmake --build build --target all -j$(nproc) --config Release
WORKDIR /xronos/cpp-sdk/build
RUN make test

FROM build AS build-docs
COPY --from=doxygen /install /install
COPY --parents docs .
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DXRONOS_SDK_BUILD_DOCS=ON
RUN cmake --build build --target docs --config Release

FROM scratch AS docs
COPY --from=build-docs /xronos/cpp-sdk/build/docs /build/docs
