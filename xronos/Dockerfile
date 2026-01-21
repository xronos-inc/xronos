# syntax=docker/dockerfile:1-labs

FROM scratch AS src
WORKDIR /xronos/xronos
COPY pyproject.toml README.md CMakeLists.txt LICENSE THIRD_PARTY_NOTICE .
COPY --parents cpp src third-party-licenses .

FROM py-venv AS common
COPY --from=cpp-sdk /install /install
COPY --from=src /xronos/xronos /xronos/xronos
WORKDIR /xronos/xronos


FROM common AS build-wheel
ENV CMAKE_PREFIX_PATH=/install
ENV LD_LIBRARY_PATH=/install/lib
RUN . /venv/bin/activate && pip wheel . --no-deps
RUN . /venv/bin/activate && auditwheel repair xronos-*.whl --strip

FROM scratch AS wheel
COPY --from=build-wheel /xronos/xronos/wheelhouse/xronos-*.whl /dist/


FROM py-venv AS build-sdist
COPY --from=cpp-sdk-src /xronos/cpp-sdk /xronos/cpp-sdk
COPY --from=lib-src /xronos/lib /xronos/lib
COPY --from=third-party-src /third-party /xronos/third-party
COPY --from=src /xronos/xronos /xronos/xronos
WORKDIR /xronos/xronos
COPY exported_symbols.txt .
RUN . /venv/bin/activate && python -m build . --sdist

FROM scratch AS sdist
COPY --from=build-sdist /xronos/xronos/dist /dist


FROM scratch AS test-src
COPY --parents test /


FROM py-venv AS test
COPY --parents src test .
COPY pyproject.toml .
COPY --from=wheel /dist/xronos-*.whl .
RUN . /venv/bin/activate && pip install xronos-*.whl
RUN . /venv/bin/activate && pytest


FROM py-venv AS lint-py
COPY --parents src test .
COPY pyproject.toml .
RUN . /venv/bin/activate && ruff format --diff
RUN . /venv/bin/activate && ruff check
COPY --from=wheel /dist/xronos-*.whl .
RUN . /venv/bin/activate && pip install xronos-*.whl
RUN . /venv/bin/activate && pyright
RUN . /venv/bin/activate && mypy .


FROM build-wheel AS lint-cpp
COPY --from=configs / .
RUN . /venv/bin/activate && \
    pip wheel . --no-deps \
    -C skbuild.build.targets=xronos-py-bindings-lint \
    -C skbuild.build.targets=xronos-py-bindings-check-format
