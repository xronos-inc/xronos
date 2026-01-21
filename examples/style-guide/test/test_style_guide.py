# SPDX-FileCopyrightText: Copyright (c) Xronos Inc.
# SPDX-License-Identifier: BSD-3-Clause

import importlib.util
import logging
import os
import sys

# add the example to syspath
example_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "../"))
sys.path.insert(0, example_dir)

# load the example
example_path = os.path.join(example_dir, "style-guide.py")
module_name = "style_guide"
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


def test_style_guide() -> None:
    example_module.main(fast=True)
