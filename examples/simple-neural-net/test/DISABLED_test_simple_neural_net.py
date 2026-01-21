# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

# pyright: standard

import importlib.util
import logging
import os
import sys
import typing

import numpy as np
import pytest

# add the example to syspath
example_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.insert(0, example_dir)

# load the example
example_path = os.path.join(example_dir, "simple-neural-net.py")
module_name = "simple-neural-net"
spec = importlib.util.spec_from_file_location(module_name, example_path)
if not spec:
    logging.error(f"Failed to find module {module_name} from {example_path}")
    sys.exit(1)
example_module = importlib.util.module_from_spec(spec)
sys.modules[module_name] = example_module
if not spec.loader:
    logging.error(f"Failed to load module {module_name}")
    sys.exit(1)
spec.loader.exec_module(example_module)

nn = example_module  # equivalent to `import simple-neural-net as nn`

EXPECTED_NEURAL_NET_LAYERS = 2


@pytest.fixture(scope="module")
def predictor() -> typing.Any:
    """Create and train the neural network once for all tests.

    Using module scope to ensure it's only created once per test module.
    """
    return nn.train(
        layer_dims=[2, 6, 1],
        epochs=700,
        learning_rate=0.08,
        X=np.array([[0, 0], [0, 1], [1, 0], [1, 1]]),
        y=np.array([[0], [1], [1], [0]]),
        patience=30,
        convergence_threshold=0.01,
    )


def test_xor_predictions(predictor: typing.Any) -> None:
    # use a wide tolerance to shorten test time; this should still
    # have a high confidence that the algorithm is converging
    # since the expected values are binary
    TEST_TOLERANCE = 0.2

    test_cases = [([0, 0], 0), ([0, 1], 1), ([1, 0], 1), ([1, 1], 0)]

    for input_data, expected in test_cases:
        prediction = predictor.predict(input_data)
        assert abs(prediction - expected) < TEST_TOLERANCE, (
            f"Failed XOR test for input {input_data}: "
            f"expected {expected}, but got {prediction}"
        )

    with pytest.raises(ValueError):
        predictor.predict([0, 0, 0])


def test_model_structure(predictor: typing.Any) -> None:
    assert len(predictor.architecture) == EXPECTED_NEURAL_NET_LAYERS, (
        f"Model should have {EXPECTED_NEURAL_NET_LAYERS} layers"
    )

    assert predictor.architecture[0]["weights"].shape == (
        2,
        6,
    ), "First layer weights should be 2x6"
    assert predictor.architecture[0]["biases"].shape == (
        1,
        6,
    ), "First layer biases should be 1x6"

    assert predictor.architecture[1]["weights"].shape == (
        6,
        1,
    ), "Second layer weights should be 6x1"
    assert predictor.architecture[1]["biases"].shape == (
        1,
        1,
    ), "Second layer biases should be 1x1"
