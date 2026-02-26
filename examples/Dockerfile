# syntax=docker/dockerfile:1-labs

FROM scratch AS src
COPY . /examples

FROM scratch AS common-files
WORKDIR /xronos/examples
COPY mypy.ini ruff.toml pyrightconfig.json pytest.ini .
