# syntax=docker/dockerfile:1-labs@sha256:63e440b412b6acba117974e793b7e7f702e58ee65e044bdff1b8d388ee0d853b

FROM scratch AS src
COPY . /examples

FROM scratch AS common-files
WORKDIR /xronos/examples
COPY mypy.ini ruff.toml pyrightconfig.json pytest.ini .
