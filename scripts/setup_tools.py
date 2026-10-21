"""Install the pinned xPack ARM toolchain and OpenOCD into .tools/ at the repo root.

    uv run scripts/setup_tools.py

SConstruct puts the bin directories listed in .tools/path.txt first on PATH,
so builds use these versions instead of whatever is installed system-wide.
"""

import hashlib
import platform
import shutil
import sys
import tarfile
import urllib.request
import zipfile
from pathlib import Path

TOOLS = {
    "arm-none-eabi-gcc": "14.2.1-1.1",
    "openocd": "0.12.0-7",
}

TOOLS_DIR = Path(__file__).resolve().parent.parent / ".tools"
DOWNLOAD_DIR = TOOLS_DIR / "downloads"


def platform_suffix():
    system = platform.system()
    machine = platform.machine().lower()
    arch = {"x86_64": "x64", "amd64": "x64", "arm64": "arm64", "aarch64": "arm64"}.get(machine)
    os_name = {"Darwin": "darwin", "Linux": "linux", "Windows": "win32"}.get(system)
    if arch is None or os_name is None or (os_name == "win32" and arch != "x64"):
        sys.exit(f"No xPack build for {system} {machine}")
    ext = "zip" if os_name == "win32" else "tar.gz"
    return f"{os_name}-{arch}", ext


def download(url, dest):
    print(f"  downloading {url}")
    with urllib.request.urlopen(url) as resp, open(dest, "wb") as f:
        shutil.copyfileobj(resp, f)


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def extract(archive, dest):
    if archive.suffix == ".zip":
        with zipfile.ZipFile(archive) as z:
            z.extractall(dest)
    else:
        with tarfile.open(archive) as t:
            t.extractall(dest, filter="data")


def install(name, version, suffix, ext):
    folder = TOOLS_DIR / f"xpack-{name}-{version}"
    if (folder / "bin").is_dir():
        print(f"{name} {version}: already installed")
        return folder / "bin"

    print(f"{name} {version}: installing")
    asset = f"xpack-{name}-{version}-{suffix}.{ext}"
    base = f"https://github.com/xpack-dev-tools/{name}-xpack/releases/download/v{version}/{asset}"
    archive = DOWNLOAD_DIR / asset
    sha_file = DOWNLOAD_DIR / f"{asset}.sha"
    download(base, archive)
    download(f"{base}.sha", sha_file)

    expected = sha_file.read_text().split()[0]
    if sha256(archive) != expected:
        archive.unlink()
        sys.exit(f"SHA-256 mismatch for {asset}")

    extract(archive, TOOLS_DIR)
    archive.unlink()
    sha_file.unlink()
    return folder / "bin"


def main():
    suffix, ext = platform_suffix()
    DOWNLOAD_DIR.mkdir(parents=True, exist_ok=True)
    bin_dirs = [install(name, version, suffix, ext) for name, version in TOOLS.items()]
    (TOOLS_DIR / "path.txt").write_text("".join(f"{d}\n" for d in bin_dirs))
    print("\nSConstruct will use:")
    for d in bin_dirs:
        print(f"  {d}")


if __name__ == "__main__":
    main()
