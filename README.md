# V-MACS Dot Scripts

This repository test hardware board compatibility and integration to V-MACS dots based on RP2350-POE-ETH https://docs.waveshare.com/RP2350-POE-ETH.

Test could be used with usual rapberry pi connected to the hardware moduiles using GPIO pins.

rpi-gpio.jpg

## Setup

The repo is managed with [uv](https://docs.astral.sh/uv/):

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh   # once
uv sync                                           # create .venv with dependencies
uv run ir-rx/df_robot_dfr0094.py live              # run a script
```
