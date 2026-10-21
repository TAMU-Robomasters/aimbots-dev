<!-- [![pipeline status](https://gitlab.com/aruw/controls/taproot-aimbots-dev/badges/develop/pipeline.svg)](https://gitlab.com/aruw/controls/taproot-aimbots-dev/-/commits/develop) -->

<!-- Start sections that may be removed after forking this repository -->
<hr/>

# Texas Aimbots Embedded Development

<hr/>


# Setup

Install [uv](https://docs.astral.sh/uv/getting-started/installation/), then from the repo root:

```bash
uv run scripts/setup_tools.py   # pinned ARM GCC + OpenOCD into .tools/ (macOS, Linux, Windows)
cd aimbots-src
uv run scons build robot=SENTRY_SWERVE
```

uv installs Python and the build dependencies on first run. SConstruct puts `.tools/` ahead of any system toolchain on `PATH`. Debugger USB drivers (ST-Link/J-Link on Windows, udev rules on Linux) are still a manual install.
