# syntax=docker/dockerfile:1-labs

FROM py-venv AS dependencies
COPY requirements.txt ./
ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=localhost:0.0
RUN apt-get install -y -qq --no-install-recommends libportaudio2
RUN sed -i /^xronos/d requirements.txt
RUN . /venv/bin/activate && pip wheel -r requirements.txt --wheel-dir=wheels

FROM base AS common
WORKDIR /xronos
COPY --from=dependencies /xronos/wheels wheels
COPY requirements.txt .
RUN . /venv/bin/activate && pip install --no-index --find-links=wheels -r requirements.txt
WORKDIR /xronos/examples/keyboard-synth
COPY *.py .
COPY --parents static .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini .
RUN . /venv/bin/activate && pyright --project .. .
