# syntax=docker/dockerfile:1-labs@sha256:7d49dad25a050e14338ba7028b0460243f9d911dedc160a8fe20c34738fef3af

FROM py-venv AS common
WORKDIR /xronos/examples
COPY --from=common-files /xronos/examples/* ./
RUN --mount=from=xronos-wheel,target=/dist . /venv/bin/activate && pip install --no-cache-dir /dist/xronos-*.whl
WORKDIR /xronos/examples/hello-ros2-comparison
COPY --parents xronos ros .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini xronos
RUN . /venv/bin/activate && pyright --project .. xronos
