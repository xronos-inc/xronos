# syntax=docker/dockerfile:1-labs

FROM py-venv AS common
WORKDIR /xronos/examples
COPY --from=common-files /xronos/examples/* ./
RUN --mount=from=xronos-wheel,source=/dist,target=/dist . /venv/bin/activate && pip install --no-cache-dir /dist/xronos-*.whl
WORKDIR /xronos/examples/hello-ros2-comparison
COPY --parents xronos ros .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini xronos
RUN . /venv/bin/activate && pyright --project .. xronos
