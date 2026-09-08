"""Preserve prior artifacts when explicitly rerunning a logical run id."""

from __future__ import annotations

import shutil
from pathlib import Path

from .recorder import atomic_write_json


def archive_existing_attempt(output_dir: Path) -> Path | None:
  output_dir = output_dir.resolve()
  if not output_dir.is_dir():
    return None
  artifacts = [path for path in output_dir.iterdir() if path.name != "attempts"]
  if not artifacts:
    return None
  attempts = output_dir / "attempts"
  attempts.mkdir(parents=True, exist_ok=True)
  index = 1
  while (attempts / f"attempt_{index:03d}").exists():
    index += 1
  target = attempts / f"attempt_{index:03d}"
  target.mkdir()
  moved: list[str] = []
  for artifact in artifacts:
    destination_name = artifact.name
    if artifact.name == "summary.json":
      destination_name = "summary.archived.json"
    elif artifact.name == "metadata.json":
      destination_name = "metadata.archived.json"
    shutil.move(str(artifact), str(target / destination_name))
    moved.append(destination_name)
  atomic_write_json(target / "attempt_manifest.json", {"attempt": index, "artifacts": moved})
  return target
