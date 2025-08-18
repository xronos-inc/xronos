# syntax=docker/dockerfile:1-labs

FROM scratch AS rust-src
WORKDIR /graph-messages
COPY --parents src xronos .
COPY build.rs Cargo.lock Cargo.toml CMakeLists.txt .
