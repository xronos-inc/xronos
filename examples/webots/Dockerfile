# syntax=docker/dockerfile:1-labs@sha256:63e440b412b6acba117974e793b7e7f702e58ee65e044bdff1b8d388ee0d853b

FROM py-venv AS common
WORKDIR /xronos/examples
COPY --from=common-files /xronos/examples/* ./
RUN --mount=from=xronos-wheel,target=/dist . /venv/bin/activate && pip install --no-cache-dir /dist/xronos-*.whl

WORKDIR /xronos/examples/webots
COPY --parents controllers .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini .
RUN . /venv/bin/activate && pyright --project .. .
