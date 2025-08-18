# syntax=docker/dockerfile:1-labs

FROM scratch AS src
COPY . /examples

FROM py-venv AS base
WORKDIR /xronos
RUN mkdir wheels
COPY --from=xronos-wheel /xronos-*.whl ./wheels
WORKDIR /xronos/examples
COPY mypy.ini ruff.toml pyrightconfig.json pytest.ini .
