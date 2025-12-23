# syntax=docker/dockerfile:1-labs

FROM base AS common
WORKDIR /xronos
RUN . /venv/bin/activate && pip install --no-index --find-links=wheels xronos
WORKDIR /xronos/examples/hello-ros2-comparison
COPY --parents xronos ros .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini xronos
RUN . /venv/bin/activate && pyright --project .. xronos
