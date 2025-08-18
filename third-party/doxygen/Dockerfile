FROM base AS build
WORKDIR /xronos

# install build dependencies -- not needed once doxygen is built
RUN apt-get install -y -q --no-install-recommends \
      bison \
      flex \
      libz-dev \
      libzstd-dev \
      libcurl4-openssl-dev \
      libedit-dev

# build and install doxygen
WORKDIR /xronos/doxygen
COPY --from=doxygen-src / ./
RUN cmake -S . -B build \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX=/install \
      -DENABLE_CLANG_TIDY=OFF \
      -Duse_libclang=ON \
      -DCMAKE_CXX_FLAGS="-w"
RUN cmake --build build --config Release -- -j$(nproc)
RUN cmake --build build --target install --config Release
RUN /install/bin/doxygen --version

FROM scratch AS install
COPY --from=build /install /install
