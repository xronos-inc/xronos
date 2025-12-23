# syntax=docker/dockerfile:1-labs

FROM scratch AS src
WORKDIR /xronos/lib
COPY --parents core graph-exporter graph-messages logical-time runtime-default runtime-interfaces source-location telemetry telemetry-otel util validator .
COPY CMakeLists.txt Config.cmake.in .

FROM base AS common
WORKDIR /xronos/lib
COPY --from=configs /.clang-format /.clang-tidy ./
COPY --from=src /xronos/lib .

FROM common AS cmake-init
COPY --from=opentelemetry /install /install
COPY --from=third-party-files /third-party /xronos/third-party
RUN cmake -S . -B build \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/install \
    -DXRONOS_LIB_BUILD_TESTS=OFF \
    -DXRONOS_ABSL_PROVIDER=package \
    -DXRONOS_PROTOBUF_PROVIDER=package \
    -DXRONOS_GRPC_PROVIDER=package \
    -DXRONOS_OPENTELEMETRY_PROVIDER=package

FROM cmake-init AS lint
RUN cmake --build build --target xronos-graph-messages -j$(nproc)
RUN cmake --build build --target xronos-lib-check-format
RUN cmake --build build --target xronos-lib-lint

FROM cmake-init AS build
RUN cmake --build build --target all -j$(nproc)
RUN cmake --build build --target install

FROM scratch AS install
COPY --from=build /install /install

FROM cmake-init AS debug-builder
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install-debug -DCMAKE_BUILD_TYPE=Debug
RUN cmake --build build --target all -j$(nproc)
RUN cmake --build build --target install

FROM debug-builder AS test
COPY --from=catch2 /install /install
RUN cmake -S . -B build -DXRONOS_LIB_BUILD_TESTS=ON
RUN cmake --build build --target all -j$(nproc)
WORKDIR /xronos/lib/build
RUN make test

FROM scratch AS install-debug
COPY --from=debug-builder /install-debug /install
