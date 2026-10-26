# syntax=docker/dockerfile:1-labs@sha256:ad43eec41369d9ef94412c0d1eb3203d12218d01da225afc92a3aeba3a0e7796

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
