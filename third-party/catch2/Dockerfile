FROM base AS build
COPY CMakeLists.txt catch2.cmake .
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DCMAKE_BUILD_TYPE=Release
RUN cmake --build build -j$(nproc)
RUN cmake --build build --target install

FROM scratch AS install
COPY --from=build /install /install
COPY catch2.cmake /xronos/third-party/catch2
