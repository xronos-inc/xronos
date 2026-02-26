# syntax=docker/dockerfile:1-labs

FROM py-venv AS common
WORKDIR /xronos/examples
COPY requirements.txt ./
RUN sed -i /^xronos/d requirements.txt
RUN . /venv/bin/activate && pip install --no-cache-dir -r requirements.txt
COPY --from=common-files /xronos/examples/* ./
RUN --mount=from=xronos-wheel,source=/dist,target=/dist . /venv/bin/activate && pip install --no-cache-dir /dist/xronos-*.whl

WORKDIR /xronos/examples/YOLO
COPY *.py .
COPY --parents test .

FROM common AS lint
RUN . /venv/bin/activate && ruff --config=../ruff.toml check
RUN . /venv/bin/activate && ruff --config=../ruff.toml format --diff
RUN . /venv/bin/activate && mypy --config-file=../mypy.ini .
RUN . /venv/bin/activate && pyright --project .. .

FROM common AS test
RUN . /venv/bin/activate && pytest --config-file=../pytest.ini
