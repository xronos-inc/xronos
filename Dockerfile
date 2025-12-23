FROM ubuntu:22.04 AS base
WORKDIR /xronos

ENV DEBIAN_FRONTEND=noninteractive
ENV DISPLAY=localhost:0.0

ARG clang_tools_version=20

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
        python3.10-dev python3.10-venv \
        python3.11-dev python3.11-venv \
        python3.12-dev python3.12-venv \
        python3.13-dev python3.13-venv python3.13-nogil \
        python3.14-dev python3.14-venv python3.14-nogil \
        python3-pygments \
        zlib1g-dev

# This is a workaround for a gcc bug. See https://gcc.gnu.org/bugzilla/show_bug.cgi?id=118700
RUN cp /usr/include/python3.13t/pyconfig.h ./
RUN rm -rf /usr/include/python3.13t
RUN cp -r /usr/include/python3.13 /usr/include/python3.13t
RUN rm /usr/include/python3.13t/pyconfig.h
RUN mv ./pyconfig.h /usr/include/python3.13t/pyconfig.h
# Same for Python 3.14
RUN cp /usr/include/python3.14t/pyconfig.h ./
RUN rm -rf /usr/include/python3.14t
RUN cp -r /usr/include/python3.14 /usr/include/python3.14t
RUN rm /usr/include/python3.14t/pyconfig.h
RUN mv ./pyconfig.h /usr/include/python3.14t/pyconfig.h

RUN update-alternatives --install /usr/bin/clang clang /usr/bin/clang-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang++ clang++ /usr/bin/clang++-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${clang_tools_version} 100
RUN update-alternatives --install /usr/bin/clang-tidy clang-tidy /usr/bin/clang-tidy-${clang_tools_version} 100


FROM base AS py-venv
ARG python_version
COPY dev-requirements.txt ./
RUN python$python_version -m venv /venv
RUN . /venv/bin/activate && pip install -r dev-requirements.txt
# Run pyright once to install its node dependencies
RUN . /venv/bin/activate && pyright --version


FROM scratch AS configs
COPY .clang-format .clang-tidy dev-requirements.txt /

FROM hashicorp/terraform:1.11 AS check-format
WORKDIR /xronos
# Need to rename the file because terraform fmt errors out for .hcl (without tftest)
COPY docker-bake.hcl docker-bake.tftest.hcl
RUN terraform fmt --check docker-bake.tftest.hcl
