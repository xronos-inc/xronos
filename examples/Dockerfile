# syntax=docker/dockerfile:1-labs@sha256:7d49dad25a050e14338ba7028b0460243f9d911dedc160a8fe20c34738fef3af

FROM scratch AS src
COPY . /examples

FROM scratch AS common-files
WORKDIR /xronos/examples
COPY mypy.ini ruff.toml pyrightconfig.json pytest.ini .
