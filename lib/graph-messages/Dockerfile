# syntax=docker/dockerfile:1-labs@sha256:ad43eec41369d9ef94412c0d1eb3203d12218d01da225afc92a3aeba3a0e7796

FROM scratch AS rust-src
WORKDIR /graph-messages
COPY --parents src xronos .
COPY build.rs Cargo.lock Cargo.toml CMakeLists.txt .
