# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import importlib.util
import logging
import os
import sys
import warnings

# add the example to syspath
example_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.insert(0, example_dir)

# suppress warnings that occur during imports
warnings.filterwarnings(
    "ignore", "distutils Version classes are deprecated", DeprecationWarning
)

# load the example
example_path = os.path.join(example_dir, "YOLO.py")
module_name = "YOLO"
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


def test_yolo() -> None:
    test_file = os.path.join(example_dir, "test/test.webm")
    example_module.main(test_file, True, False)
