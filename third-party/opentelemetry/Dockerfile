FROM base AS build
COPY --from=grpc /install /install
COPY CMakeLists.txt opentelemetry.cmake .
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DCMAKE_BUILD_TYPE=Release
RUN LD_LIBRARY_PATH=/install/lib  cmake --build build -j$(nproc)
RUN cmake --build build --target install

FROM scratch AS install
COPY --from=build /install /install
COPY opentelemetry.cmake /xronos/third-party/opentelemetry
