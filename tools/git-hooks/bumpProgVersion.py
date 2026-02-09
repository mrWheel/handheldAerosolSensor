#!/usr/bin/env python3
#— Keep PROG_VERSION ("vX.Y.Z") and tools/PROG_VERSION.json in sync.

from __future__ import annotations

import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


#— Matches e.g.:
#—   const char* PROG_VERSION = "v1.2.3";
#— We only require the token PROG_VERSION and a string literal "vX.Y.Z".
PROG_VERSION_LINE_REGEX = re.compile(
  r"""
  (?P<prefix>.*?\bPROG_VERSION\b.*?=\s*")
  v(?P<major>\d+)\.(?P<minor>\d+)\.(?P<patch>\d+)
  (?P<suffix>"\s*;.*)
  """,
  re.VERBOSE,
)

#— Fallback: allow formats where '=' and ';' might differ, but still contain "vX.Y.Z"
PROG_VERSION_ANYWHERE_REGEX = re.compile(r'\bPROG_VERSION\b.*?"v(\d+)\.(\d+)\.(\d+)"')


@dataclass
class SemVer:
  major: int
  minor: int
  patch: int

  def __str__(self) -> str:
    return f"v{self.major}.{self.minor}.{self.patch}"


def run(cmd: list[str]) -> tuple[int, str]:
  proc = subprocess.run(cmd, text=True, capture_output=True)
  out = (proc.stdout or "") + (proc.stderr or "")
  return proc.returncode, out.strip()


def repo_root() -> Path:
  rc, out = run(["git", "rev-parse", "--show-toplevel"])
  if rc != 0 or not out:
    raise RuntimeError("Not a git repository.")
  return Path(out)


def staged_source_files(repo: Path) -> list[Path]:
  #— Only staged (added/copied/modified) C/C++ header/source files.
  rc, out = run(["git", "diff", "--cached", "--name-only", "--diff-filter=ACM"])
  if rc != 0:
    return []

  paths: list[Path] = []
  for line in out.splitlines():
    p = (repo / line.strip())
    if p.suffix.lower() in [".c", ".cpp", ".h"] and p.exists():
      paths.append(p)
  return paths


def read_prog_version(path: Path) -> Optional[SemVer]:
  text = path.read_text(encoding="utf-8", errors="replace")

  #— Fast fail if token not present.
  if "PROG_VERSION" not in text:
    return None

  m = PROG_VERSION_LINE_REGEX.search(text)
  if m:
    return SemVer(int(m.group("major")), int(m.group("minor")), int(m.group("patch")))

  m2 = PROG_VERSION_ANYWHERE_REGEX.search(text)
  if m2:
    return SemVer(int(m2.group(1)), int(m2.group(2)), int(m2.group(3)))

  return None


def write_prog_version(path: Path, new_ver: SemVer) -> bool:
  text = path.read_text(encoding="utf-8", errors="replace")

  def repl(match: re.Match) -> str:
    return f'{match.group("prefix")}{new_ver}{match.group("suffix")}'

  new_text, n = PROG_VERSION_LINE_REGEX.subn(repl, text, count=1)
  if n == 0:
    #— Try weaker replacement (first "vX.Y.Z" near PROG_VERSION).
    m2 = PROG_VERSION_ANYWHERE_REGEX.search(text)
    if not m2:
      return False

    old = f'v{m2.group(1)}.{m2.group(2)}.{m2.group(3)}'
    new_text = text.replace(f'"{old}"', f'"{new_ver}"', 1)

  if new_text != text:
    path.write_text(new_text, encoding="utf-8")
  return True


def load_json(path: Path) -> SemVer:
  data = json.loads(path.read_text(encoding="utf-8"))
  return SemVer(int(data["major"]), int(data["minor"]), int(data["patch"]))


def save_json(path: Path, ver: SemVer) -> None:
  path.parent.mkdir(parents=True, exist_ok=True)
  data = {"major": ver.major, "minor": ver.minor, "patch": ver.patch}
  path.write_text(json.dumps(data, indent=2) + "\n", encoding="utf-8")


def compute_updated(new_in_code: SemVer, old_in_json: SemVer) -> SemVer:
  if new_in_code.major > old_in_json.major:
    return SemVer(new_in_code.major, 0, 0)

  if new_in_code.minor > old_in_json.minor:
    return SemVer(new_in_code.major, new_in_code.minor, 0)

  return SemVer(new_in_code.major, new_in_code.minor, old_in_json.patch + 1)


def git_add(repo: Path, paths: list[Path]) -> None:
  rels = [str(p.relative_to(repo)) for p in paths]
  run(["git", "add", "--"] + rels)


def main() -> int:
  repo = repo_root()
  json_path = repo / "tools" / "PROG_VERSION.json"

  files = staged_source_files(repo)
  if not files:
    return 0

  version_file: Optional[Path] = None
  new_in_code: Optional[SemVer] = None

  for f in files:
    ver = read_prog_version(f)
    if ver is not None:
      version_file = f
      new_in_code = ver
      break

  if version_file is None or new_in_code is None:
    return 0

  #— If json doesn't exist: create it from current PROG_VERSION and stop.
  if not json_path.exists():
    save_json(json_path, new_in_code)
    git_add(repo, [json_path])
    print(f"[bumpProgVersion] Created tools/PROG_VERSION.json from PROG_VERSION={new_in_code}.")
    return 0

  old_json = load_json(json_path)
  updated = compute_updated(new_in_code, old_json)

  changed: list[Path] = []

  if (updated.major, updated.minor, updated.patch) != (old_json.major, old_json.minor, old_json.patch):
    save_json(json_path, updated)
    changed.append(json_path)

  if str(updated) != str(new_in_code):
    if not write_prog_version(version_file, updated):
      print(f"[bumpProgVersion] ERROR: Could not update PROG_VERSION in {version_file.relative_to(repo)}", file=sys.stderr)
      return 1
    changed.append(version_file)

  if changed:
    git_add(repo, changed)
    print(f"[bumpProgVersion] PROG_VERSION -> {updated} (synced with tools/PROG_VERSION.json).")

  return 0


if __name__ == "__main__":
  raise SystemExit(main())
  
