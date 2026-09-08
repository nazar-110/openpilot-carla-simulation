# Third-party software

This repository contains the evaluation toolkit, simulator-adapter patches and
original recorded evaluation media. It does not distribute CARLA packages,
OpenPilot model weights, or an OpenPilot checkout.

- [OpenPilot](https://github.com/commaai/openpilot), copyright comma.ai and its
  contributors, MIT license. The integration targets commit
  `4df40d2c1946a57242230186edd073c4073060a6`. Simulator-interface conventions and
  camera projection calculations follow this upstream implementation.
- [CARLA](https://github.com/carla-simulator/carla), copyright the CARLA
  contributors, MIT license for code; simulator assets are subject to the
  licenses distributed with CARLA. Demo footage is captured from CARLA 0.9.16.
- NumPy, Pillow, PyYAML, pytest and Ruff are installed dependencies and retain
  their respective licenses. FFmpeg is an external video-encoding dependency.

OpenPilot and CARLA names identify the software being evaluated. This independent
portfolio project is not affiliated with or endorsed by their maintainers.
