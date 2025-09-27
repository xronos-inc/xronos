FROM base AS build
COPY --from=absl /install /install
COPY CMakeLists.txt protobuf.cmake .
RUN cmake -S . -B build -DCMAKE_INSTALL_PREFIX=/install -DCMAKE_BUILD_TYPE=Release
RUN LD_LIBRARY_PATH=/install/lib  cmake --build build -j$(nproc)
RUN cmake --build build --target install

FROM scratch AS install
COPY --from=build /install /install
COPY protobuf.cmake /xronos/third-party/protobuf