#!/usr/bin/env python3
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path
from typing import Iterable


TEMPLATE_REPO = "https://github.com/mrWheel/templateRepo"


# Wat je standaard wil “syncen” vanuit de template.
# Voeg gerust extra paden toe als je templateRepo later groeit.
DEFAULT_PATHS = [
    ".github/workflows",
    "tools/git-hooks",
    ".clangFormat",
]


def run(cmd: list[str], cwd: Path | None = None) -> str:
    """Run command, raise on error, return stdout."""
    p = subprocess.run(
        cmd,
        cwd=str(cwd) if cwd else None,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    if p.returncode != 0:
        raise RuntimeError(
            f"Command failed ({p.returncode}): {' '.join(cmd)}\n"
            f"STDOUT:\n{p.stdout}\nSTDERR:\n{p.stderr}"
        )
    return p.stdout.strip()


def is_git_repo(root: Path) -> bool:
    return (root / ".git").exists() or (
        # support worktrees/submodules where .git is a file
        (root / ".git").is_file()
    )


def copy_tree_skip_existing(src: Path, dst: Path) -> tuple[int, int]:
    """
    Copy src into dst, recursively. Do NOT overwrite existing files.
    Returns: (copied_files, skipped_files)
    """
    copied = 0
    skipped = 0

    if src.is_file():
        dst.parent.mkdir(parents=True, exist_ok=True)
        if dst.exists():
            return (0, 1)
        shutil.copy2(src, dst)
        return (1, 0)

    for path in src.rglob("*"):
        rel = path.relative_to(src)
        target = dst / rel

        if path.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue

        target.parent.mkdir(parents=True, exist_ok=True)
        if target.exists():
            skipped += 1
            continue

        shutil.copy2(path, target)
        copied += 1

    return (copied, skipped)


def ensure_exec_bits(hooks_dir: Path) -> None:
    """
    Try to set executable bit for hook files on POSIX.
    On Windows this is harmless/no-op-ish for Git Bash scenarios.
    """
    if not hooks_dir.exists():
        return

    for f in hooks_dir.iterdir():
        if f.is_file():
            try:
                mode = f.stat().st_mode
                # add u+x,g+x,o+x
                f.chmod(mode | 0o111)
            except Exception:
                # don’t fail the whole script for chmod issues
                pass


def set_hooks_path(repo_root: Path, hooks_path: str) -> None:
    # Set hooks path (relative is fine).
    run(["git", "config", "core.hooksPath", hooks_path], cwd=repo_root)


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Apply mrWheel/templateRepo files to current repo (skip existing) and enable git hooks."
    )
    ap.add_argument(
        "--template",
        default=TEMPLATE_REPO,
        help=f"Template repo URL (default: {TEMPLATE_REPO})",
    )
    ap.add_argument(
        "--paths",
        nargs="*",
        default=DEFAULT_PATHS,
        help=f"Paths to copy from template (default: {', '.join(DEFAULT_PATHS)})",
    )
    ap.add_argument(
        "--hooks-path",
        default="tools/git-hooks",
        help="Where hooks live in target repo; will be set as git core.hooksPath (default: tools/git-hooks)",
    )
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path.cwd()

    if not is_git_repo(repo_root):
        print("❌ Dit lijkt geen git-repo root (geen .git gevonden). Run dit vanuit de root van je repo.", file=sys.stderr)
        return 2

    # Check that git exists
    try:
        run(["git", "--version"])
    except Exception as e:
        print(f"❌ Git lijkt niet beschikbaar: {e}", file=sys.stderr)
        return 2

    with tempfile.TemporaryDirectory(prefix="templateRepo_") as td:
        tmp = Path(td)
        template_dir = tmp / "template"

        print(f"📥 Cloning template: {args.template}")
        run(["git", "clone", "--depth", "1", args.template, str(template_dir)])

        total_copied = 0
        total_skipped = 0

        for rel in args.paths:
            src = template_dir / rel
            dst = repo_root / rel

            if not src.exists():
                print(f"⚠️  Bestaat niet in template, overslaan: {rel}")
                continue

            if src.is_file():
                copied, skipped = copy_tree_skip_existing(src, dst)
            else:
                dst.mkdir(parents=True, exist_ok=True)
                copied, skipped = copy_tree_skip_existing(src, dst)

            total_copied += copied
            total_skipped += skipped
            print(f"✅ {rel}: +{copied} gekopieerd, {skipped} overgeslagen (bestonden al)")

    # Enable hooks
    hooks_dir = repo_root / args.hooks_path
    if hooks_dir.exists():
        ensure_exec_bits(hooks_dir)
        set_hooks_path(repo_root, args.hooks_path)
        print(f"🪝 Git hooks geactiveerd: core.hooksPath = {args.hooks_path}")
    else:
        print(f"⚠️  Hooks-map niet gevonden ({args.hooks_path}); core.hooksPath niet gezet.", file=sys.stderr)

    print(f"🎉 Klaar. Totaal: +{total_copied} gekopieerd, {total_skipped} overgeslagen.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
  