<!-- [![pipeline status](https://gitlab.com/aruw/controls/taproot-aimbots-dev/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/taproot-aimbots-dev/-/commits/develop) -->

<!-- Start sections that may be removed after forking this repository -->
# Texas Aimbots Embedded Development

The comprehensive controls library for Texas A&M Robomaster Robotics; the premier robotics organization of Texas A&M University! This repository is based on the [Taproot](https://gitlab.com/aruw/controls/taproot) framework and falls under the GPL-3.0 License

## System Setup

### Toolchain/Library Installation

Setup is derived from the following guides within the Taproot wiki. Select the guide appropriate for the platform you are using.

**Note**: ***You will not be able to download the ST-Link driver***. For TAMU RM members, you need to download Ozone and the respective J-Link drivers. We use exclusively J-Link's for live debugging and flashing of code to our main control boards.

**Second Note** You will not be able to install ClangFormat Version 10. This will be fixed in the future but your code should still be able to compile without issue

- Debian/Ubuntu: <https://gitlab.com/aruw/controls/taproot/-/wikis/Debian-Linux-Setup>
- macOS: <https://gitlab.com/aruw/controls/taproot/-/wikis/macOS-Setup>
- Windows: <https://gitlab.com/aruw/controls/taproot/-/wikis/Windows-Setup>

After installing all the required toolchains listed in the above guides as well as updating PATH variables, you will clone this repository. You can do this through VSCode, or by running:

`git clone https://github.com/TAMU-Robomasters/aimbots-dev`

Be sure you clone the repository into a sensible place on your computer (ie. not Downloads or in your OneDrive)

### `pipenv` installation

Pipenv is a Python tool that creates a virtual environment as well as manages packages. We will use this to run build tasks in an isolated environment.

Run:

```bash
pip3 install pipenv
cd path/to/repo/aimbots-dev
pipenv install
```

This will install all necessary python packages that will assist in building code

## Building Code

In order to build a firmware/code image for a main control board/dev board, first `cd` to the head of `aimbots-src`. Your file path will look something similar to `~/path/to/repo/aimbots-dev/aimbots-src`

Next, run
`pipenv run scons build robot=[TARGET_HERE]`. A list of valid targets is located in `extract_robot_type.py`
