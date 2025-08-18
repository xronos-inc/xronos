# syntax=docker/dockerfile:1-labs

FROM base AS common
WORKDIR /xronos
RUN . /venv/bin/activate && pip install --no-index --find-links=wheels xronos
WORKDIR /xronos/examples/style-guide
COPY *.py .
COPY --parents test .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini .
RUN . /venv/bin/activate && pyright --project .. .

FROM common AS test
RUN . /venv/bin/activate && pytest --config-file=../pytest.ini
