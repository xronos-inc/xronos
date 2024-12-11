# Yahboom Robot Arm Example

This repository implements a xronos controller of the [Yahboom DOFBOT](http://www.yahboom.net/study/Dofbot-Jetson_nano)
6 degree-of-freedom robotic arm.

## Requirements

- Yahboom DOFBOT running Ubuntu 22.04.
  - See [Configure Jetson Nano with Ubuntu 22.04](https://github.com/xronos-inc/jetson-nano-ubuntu-22.04) to configure a Jetson Nano with Ubuntu 22.04
  - This repo should work with Ubuntu 22.04 on a Raspberry Pi but has not been tested.
- Python 3.10 (we recommend using a virtual environment)

## Setup

Create a Python virtual environment (optional but recommended):

```bash
cd examples/robot-arm
python -m venv .venv
source .venv/bin/activate
```

Install Python depencencies:

```bash
pip install -r requirements.txt
```

Run the program and use the command line interface to send a trajectory to the robot before exiting the program:

```bash
python arm.py
> moveto green blue red init
> exit
```

## References

- `Arm_Lib` is part of the source package provided by Yahboom, see [http://www.yahboom.net/study/Dofbot-Jetson_nano](http://www.yahboom.net/study/Dofbot-Jetson_nano)
