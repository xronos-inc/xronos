# Object detection using YOLOv11

This example shows how a simple object detection pipeline can be built in xronos.

## Requirements

This example requires `libgl1`:

```shell
apt-get install libgl1
```

## Getting started

A virtual environment with xronos installed is assumed.

1. Install the dependencies
    ```shell
    pip install -r requirements.txt
    ```
1. Run example with logging from torch disabled.
    ```shell
    YOLO_VERBOSE=False python YOLO.py
    ```
