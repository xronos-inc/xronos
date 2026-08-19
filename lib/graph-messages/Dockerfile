# syntax=docker/dockerfile:1-labs@sha256:63e440b412b6acba117974e793b7e7f702e58ee65e044bdff1b8d388ee0d853b

FROM scratch AS rust-src
WORKDIR /graph-messages
COPY --parents src xronos .
COPY build.rs Cargo.lock Cargo.toml CMakeLists.txt .
