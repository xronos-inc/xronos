variable "CONTEXT_PREFIX" { default = "." }

variable "XRONOS_VERSION" { default = "0.9.0" }

target "base" {
  target  = "base"
  context = CONTEXT_PREFIX
  output  = [{ type = "cacheonly" }]
}

target "py-venv" {
  name = "py-venv-${replace(version, ".", "")}"
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13", "3.13t", "3.14", "3.14t"]
  }
  args = {
    python_version = version
  }
  target  = "py-venv"
  context = CONTEXT_PREFIX
  output  = [{ type = "cacheonly" }]
}

target "configs" {
  target  = "configs"
  context = CONTEXT_PREFIX
  output  = [{ type = "cacheonly" }]
}

target "third-party-cmake-files" {
  context = "${CONTEXT_PREFIX}/third-party"
  output  = [{ type = "cacheonly" }]
}

target "_third-party-common" {
  target = "install"
  contexts = {
    base = "target:base"
  }
  output = [{ type = "cacheonly" }]
}

target "third-party-absl" {
  inherits = ["_third-party-common"]
  context  = "${CONTEXT_PREFIX}/third-party/absl"
}

target "third-party-protobuf" {
  inherits = ["_third-party-common"]
  context  = "${CONTEXT_PREFIX}/third-party/protobuf"
  contexts = {
    absl = "target:third-party-absl"
  }
}

target "third-party-grpc" {
  inherits = ["_third-party-common"]
  context  = "${CONTEXT_PREFIX}/third-party/grpc"
  contexts = {
    protobuf = "target:third-party-protobuf"
  }
}

target "third-party-opentelemetry" {
  inherits = ["_third-party-common"]
  context  = "${CONTEXT_PREFIX}/third-party/opentelemetry"
  contexts = {
    grpc = "target:third-party-grpc"
  }
}

target "third-party-catch2" {
  inherits = ["_third-party-common"]
  context  = "${CONTEXT_PREFIX}/third-party/catch2"
}

target "third-party-doxygen" {
  inherits = ["_third-party-common"]
  contexts = {
    doxygen-src = "https://github.com/doxygen/doxygen.git#Release_1_16_1",
  }
  context = "${CONTEXT_PREFIX}/third-party/doxygen"
}

target "xronos-lib-src" {
  target  = "src"
  context = "${CONTEXT_PREFIX}/lib"
  output  = [{ type = "cacheonly" }]
}

target "xronos-lib-lint" {
  inherits = ["xronos-lib-src"]
  target   = "lint"
  contexts = {
    base              = "target:base"
    configs           = "target:configs"
    opentelemetry     = "target:third-party-opentelemetry"
    third-party-files = "target:third-party-cmake-files"
  }
}

target "xronos-lib-install" {
  inherits = ["xronos-lib-lint"]
  target   = "install"
}

target "xronos-lib-install-debug" {
  inherits = ["xronos-lib-lint"]
  target   = "install-debug"
}

target "xronos-lib-test" {
  inherits = ["xronos-lib-lint"]
  target   = "test"
  contexts = {
    catch2 = "target:third-party-catch2"
  }
}

target "xronos-lib-graph-messages-rust-src" {
  target  = "rust-src"
  context = "${CONTEXT_PREFIX}/lib/graph-messages"
  output  = [{ type = "cacheonly" }]
}

target "xronos-cpp-sdk-src" {
  target  = "src"
  context = "${CONTEXT_PREFIX}/cpp-sdk"
  output  = [{ type = "cacheonly" }]
}

target "xronos-cpp-sdk-install" {
  inherits = ["xronos-cpp-sdk-src"]
  target   = "install"
  contexts = {
    base    = "target:base"
    configs = "target:configs"
    lib     = "target:xronos-lib-install"
  }
}

target "xronos-cpp-sdk-test" {
  inherits = ["xronos-cpp-sdk-install"]
  target   = "test"
  contexts = {
    third-party-files = "target:third-party-cmake-files"
    lib-debug         = "target:xronos-lib-install-debug"
  }
}

target "xronos-cpp-sdk-lint" {
  inherits = ["xronos-cpp-sdk-install"]
  target   = "lint"
}

target "xronos-cpp-sdk-pkgs" {
  inherits = ["xronos-cpp-sdk-install"]
  target   = "pkgs"
  output   = ["${CONTEXT_PREFIX}/cpp-sdk"]
}

target "xronos-cpp-sdk-docs" {
  inherits = ["xronos-cpp-sdk-install"]
  target   = "docs"
  contexts = {
    doxygen = "target:third-party-doxygen"
  }
  output = ["${CONTEXT_PREFIX}/cpp-sdk"]
}

target "xronos-py-wheel" {
  name = "xronos-py-wheel-${replace(version, ".", "")}"
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13", "3.13t", "3.14", "3.14t"]
  }
  target  = "wheel"
  context = "${CONTEXT_PREFIX}/python-sdk"
  contexts = {
    py-venv = "target:py-venv-${replace(version, ".", "")}"
    cpp-sdk = "target:xronos-cpp-sdk-install"
  }
  output = ["${CONTEXT_PREFIX}/python-sdk"]
}

target "xronos-py-sdist" {
  target  = "sdist"
  context = "${CONTEXT_PREFIX}/python-sdk"
  contexts = {
    py-venv         = "target:py-venv-313"
    cpp-sdk-src     = "target:xronos-cpp-sdk-src"
    lib-src         = "target:xronos-lib-src"
    third-party-src = "target:third-party-cmake-files"
  }
  output = ["${CONTEXT_PREFIX}/python-sdk"]
}

target "xronos-py-test-src" {
  target  = "test-src"
  context = "${CONTEXT_PREFIX}/python-sdk"
  output  = [{ type = "cacheonly" }]
}

target "xronos-py-test" {
  inherits = ["xronos-py-wheel-${replace(version, ".", "")}"]
  name     = "xronos-py-test-${replace(version, ".", "")}"
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13", "3.13t", "3.14", "3.14t"]
  }
  target = "test"
  output = [{ type = "cacheonly" }]
}

target "xronos-py-lint-py" {
  inherits = ["xronos-py-test-312"]
  target   = "lint-py"
}

target "xronos-py-lint-cpp" {
  inherits = ["xronos-py-test-312"]
  target   = "lint-cpp"
  contexts = {
    configs = "target:configs"
  }
}

target "xronos-examples-common-files" {
  target  = "common-files"
  context = "${CONTEXT_PREFIX}/examples"
  output  = [{ type = "cacheonly" }]
}

target "_xronos-examples-common" {
  name = "_xronos-examples-common-${replace(version, ".", "")}"
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13", "3.13t", "3.14", "3.14t"]
  }
  contexts = {
    common-files = "target:xronos-examples-common-files"
    py-venv      = "target:py-venv-${replace(version, ".", "")}"
    xronos-wheel = "target:xronos-py-wheel-${replace(version, ".", "")}"
  }
  output = [{ type = "cacheonly" }]
}

target "xronos-examples-hello-ros2-comparison-lint" {
  name     = "xronos-examples-ros2-comparison-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/hello-ros2-comparison/"
}

target "xronos-examples-isaac-lint" {
  inherits = ["_xronos-examples-common-311"]
  target   = "lint"
  context  = "${CONTEXT_PREFIX}/examples/isaac/"
}

target "xronos-examples-keyboard-synth-lint" {
  name     = "xronos-examples-keyboard-synth-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/keyboard-synth/"
}

target "xronos-examples-montecarlo-test" {
  name     = "xronos-examples-montecarlo-test-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13"]
  }
  target  = "test"
  context = "${CONTEXT_PREFIX}/examples/montecarlo/"
}

target "xronos-examples-montecarlo-lint" {
  name     = "xronos-examples-montecarlo-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/montecarlo/"
}

target "xronos-examples-robot-arm-lint" {
  name     = "xronos-examples-robot-arm-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/robot-arm/"
}

target "xronos-examples-simple-neural-net-lint" {
  name     = "xronos-examples-simple-neural-net-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/simple-neural-net/"
}

target "xronos-examples-style-guide-test" {
  name     = "xronos-examples-style-guide-test-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.10", "3.11", "3.12", "3.13", "3.14"]
  }
  target  = "test"
  context = "${CONTEXT_PREFIX}/examples/style-guide/"
}

target "xronos-examples-style-guide-lint" {
  name     = "xronos-examples-style-guide-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/style-guide/"
}

target "xronos-examples-yolo-test" {
  name     = "xronos-examples-yolo-test-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "test"
  context = "${CONTEXT_PREFIX}/examples/YOLO/"
}

target "xronos-examples-webots-lint" {
  name     = "xronos-examples-webots-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/webots/"
}

target "xronos-examples-yolo-lint" {
  name     = "xronos-examples-yolo-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/YOLO/"
}

target "xronos-examples-yolo-lint" {
  name     = "xronos-examples-yolo-lint-${replace(version, ".", "")}"
  inherits = ["_xronos-examples-common-${replace(version, ".", "")}"]
  matrix = {
    version = ["3.12"]
  }
  target  = "lint"
  context = "${CONTEXT_PREFIX}/examples/YOLO/"
}

target "xronos-examples-src" {
  target  = "src"
  context = "${CONTEXT_PREFIX}/examples"
  output  = [{ type = "cacheonly" }]
}

target "bake-check-format" {
  context = CONTEXT_PREFIX
  target  = "check-format"
  output  = [{ type = "cacheonly" }]
}

variable "LINT_TARGETS" {
  default = [
    "bake-check-format",
    "xronos-lib-lint",
    "xronos-cpp-sdk-lint",
    "xronos-py-lint-py",
    "xronos-py-lint-cpp",
    "xronos-examples-hello-ros2-comparison-lint",
    "xronos-examples-isaac-lint",
    "xronos-examples-keyboard-synth-lint",
    "xronos-examples-montecarlo-lint",
    "xronos-examples-robot-arm-lint",
    "xronos-examples-simple-neural-net-lint",
    "xronos-examples-style-guide-lint",
    "xronos-examples-webots-lint",
    "xronos-examples-yolo-lint",
  ]
}

group "lint" {
  targets = LINT_TARGETS
}

variable "BUILD_TARGETS" {
  default = [
    "xronos-cpp-sdk-pkgs",
    "xronos-py-sdist",
    "xronos-py-wheel",
  ]
}

group "build" {
  targets = BUILD_TARGETS
}

variable "TEST_TARGETS" {
  default = [
    "xronos-lib-test",
    "xronos-cpp-sdk-test",
    "xronos-py-test",
    "xronos-examples-montecarlo-test",
    "xronos-examples-style-guide-test",
    "xronos-examples-yolo-test",
  ]
}

group "test" {
  targets = TEST_TARGETS
}

variable "DOCS_TARGETS" {
  default = ["xronos-cpp-sdk-docs"]
}

group "docs" {
  targets = DOCS_TARGETS
}

group "all" {
  targets = ["lint", "build", "test", "docs"]
}

group "default" {
  targets = ["all"]
}
