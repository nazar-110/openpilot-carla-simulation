"""OpenPilot-in-CARLA evaluation toolkit."""

from importlib.metadata import PackageNotFoundError, version

try:
  __version__ = version("opencarla-eval")
except PackageNotFoundError:  # pragma: no cover - source tree without install
  __version__ = "0.1.0"

__all__ = ["__version__"]
