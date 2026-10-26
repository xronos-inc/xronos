FROM ubuntu:jammy-20260930@sha256:0a9887931b0e38af2e3487ed39e29436b0535c4f3f52e57745baaf79d1898956 AS base
WORKDIR /xronos

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=localhost:0.0

ARG clang_tools_version=23

RUN apt-get update -qq
RUN apt-get install -y -q --no-install-recommends apt-utils
RUN echo 'debconf debconf/frontend select Noninteractive' | debconf-set-selections

# Some base packages required for adding ppas
RUN apt-get install -y -q --no-install-recommends \
        ca-certificates \
        curl \
        gnupg \
        gpg \
        software-properties-common \
        wget

# add kitware ppa for cmake
RUN wget -qO- https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor - > /usr/share/keyrings/kitware-archive-keyring.gpg
RUN echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" > /etc/apt/sources.list.d/kitware.list

# add ppa for clang tools
RUN wget -qO- https://apt.llvm.org/llvm.sh | bash -s -- ${clang_tools_version}

# add ppa for Python
RUN gpg --keyserver keyserver.ubuntu.com --recv-keys 6A755776
RUN gpg --export 6A755776 | gpg --dearmor > /etc/apt/trusted.gpg.d/deadsnakes-ppa.gpg
RUN echo "deb [signed-by=/etc/apt/trusted.gpg.d/deadsnakes-ppa.gpg] http://ppa.launchpad.net/deadsnakes/ppa/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/deadsnakes-ppa.list

# install all packages
RUN apt-get update -qq && apt-get install -y -q --no-install-recommends \
        build-essential \
        cmake=3.28.6-0kitware1ubuntu22.04.1 \
        cmake-data=3.28.6-0kitware1ubuntu22.04.1 \
        clang-format-${clang_tools_version} \
        clang-tidy-${clang_tools_version} \
        dpkg-dev \
        file \
        git \
        global \
        gnupg \
        graphviz \
        llvm-${clang_tools_version}-dev \
        libclang-${clang_tools_version}-dev \
        libgl1 \
        python3.11-dev python3.11-venv \
        python3.12-dev python3.12-venv \
        python3.13-dev python3.13-venv python3.13-nogil \
        python3.14-dev python3.14-venv python3.14-nogil \
        python3.15-dev python3.15-venv python3.15-nogil \
        python3-pygments \
        zlib1g-dev

# This is a workaround for a gcc bug. See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=118700
RUN set -e; for v in 3.13 3.14 3.15; do \
        cp /usr/include/python${v}t/pyconfig.h ./; \
        rm -rf /usr/include/python${v}t; \
        cp -r /usr/include/python${v} /usr/include/python${v}t; \
        mv ./pyconfig.h /usr/include/python${v}t/pyconfig.h; \
    done

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${clang_tools_version} 100


FROM base AS py-venv
ARG python_version
COPY dev-requirements.txt ./
RUN python$python_version -m venv /venv
# mypy cannot be installed on free-threaded Python 3.13. Its ast-serialize
# dependency ships no wheel for that interpreter, and building it from source
# fails because PyO3 supports free-threaded builds only from Python 3.14 on.
# Both packages need to be dropped, as installing mypy alone pulls in
# ast-serialize again. Linting runs on Python 3.12, so this venv does not need
# mypy.
RUN if [ "$python_version" = "3.13t" ]; then \
        grep -vE '^(mypy|ast-serialize)==' dev-requirements.txt > venv-requirements.txt; \
    else \
        cp dev-requirements.txt venv-requirements.txt; \
    fi
RUN . /venv/bin/activate && pip install --no-cache-dir -r venv-requirements.txt
# Run pyright once to install its node dependencies
RUN . /venv/bin/activate && pyright --version
ENV MYPY_CACHE_DIR=/dev/null


FROM scratch AS configs
COPY .clang-tidy .clang-format dev-requirements.txt /

FROM hashicorp/terraform:1.17.0@sha256:4d1b496a256f2989bccdb04b9104a4ede97234b10fba90259e173e905ec4fb58 AS check-format
WORKDIR /xronos
# Need to rename the file because terraform fmt errors out for .hcl (without tftest)
COPY docker-bake.hcl docker-bake.tftest.hcl
RUN terraform fmt --check docker-bake.tftest.hcl
