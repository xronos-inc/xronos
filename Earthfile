
VERSION 0.8

FROM ubuntu:24.04

configs:
    WORKDIR /xronos
    COPY .clang-format .clang-tidy dev-requirements.txt ./
    SAVE ARTIFACT .clang-format
    SAVE ARTIFACT .clang-tidy
    SAVE ARTIFACT dev-requirements.txt

build:
  BUILD ./cpp-sdk+build
  BUILD ./lib+build
  BUILD ./xronos+build

test:
  BUILD ./cpp-sdk+test
  BUILD ./examples+test
  BUILD ./lib+test
  BUILD ./xronos+test

lint:
  BUILD ./cpp-sdk+lint
  BUILD ./examples+lint
  BUILD ./lib+lint
  BUILD ./xronos+lint

docs:
  BUILD ./cpp-sdk+docs

all:
    BUILD +build
    BUILD +test
    BUILD +lint
    BUILD +docs
