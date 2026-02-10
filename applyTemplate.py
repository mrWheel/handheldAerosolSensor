#!/usr/bin/env python3

#-- Version Date: 09-02-2026 -- (dd-mm-eeyy)
from __future__ import annotations

import argparse
import difflib
import hashlib
import shutil
import subprocess
import sys
import tempfile
import re
from dataclasses import dataclass
from pathlib import Path


TEMPLATE_REPO = "https://github.com/mrWheel/templateRepo"


# Wat je standaard wil “syncen” vanuit de template.
# Voeg gerust extra paden toe als je templateRepo later groeit.
DEFAULT_PATHS = [
    ".github/workflows",
    "tools/git-hooks",
    ".clang-format",
]


@dataclass(frozen=True)
class FileStats:
    sizeBytes: int
    mtime: float
    lineCount: int
    sha256: str


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


def _read_text_safe(path: Path) -> str | None:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except Exception:
        return None


def _count_lines(path: Path) -> int:
    text = _read_text_safe(path)
    if text is None:
        return 0
    return text.count("\n") + (0 if text.endswith("\n") or text == "" else 1)


def _calc_sha256_bytes(data: bytes) -> str:
    h = hashlib.sha256()
    h.update(data)
    return h.hexdigest()


def _calc_sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def _is_tag_release_yml(path: Path) -> bool:
    p = path.as_posix()
    return p.endswith(".github/workflows/tag-release.yml") or (
        "workflows" in p and p.endswith("tag-release.yml")
    )


def _normalize_for_compare(path: Path, text: str) -> str:
    # Ignore project-specific values in tag-release.yml
    if not _is_tag_release_yml(path):
        return text

    # Replace the entire value of these keys, keeping indentation and key formatting
    # Works for lines like:
    #   PROGRAM_NAME: "<Enter System Name>"
    patterns = [
        r"^(\s*PROGRAM_NAME\s*:\s*).*$",
        r"^(\s*PROGRAM_SRC\s*:\s*).*$",
        r"^(\s*PROGRAM_DIR\s*:\s*).*$",
    ]

    lines = text.splitlines(keepends=True)
    out: list[str] = []

    for line in lines:
        replaced = False
        for pat in patterns:
            m = re.match(pat, line)
            if m:
                out.append(m.group(1) + '"<ignored>"\n')
                replaced = True
                break
        if not replaced:
            out.append(line)

    return "".join(out)


def _calc_sha256_for_compare(path: Path) -> str:
    text = _read_text_safe(path)
    if text is None:
        return _calc_sha256_file(path)

    normalized = _normalize_for_compare(path, text)
    return _calc_sha256_bytes(normalized.encode("utf-8", errors="replace"))


def get_file_stats(path: Path) -> FileStats:
    st = path.stat()
    return FileStats(
        sizeBytes=st.st_size,
        mtime=st.st_mtime,
        lineCount=_count_lines(path),
        sha256=_calc_sha256_for_compare(path),
    )


def make_unified_diff(src: Path, dst: Path) -> str:
    # Diff is based on normalized text (so ignored lines don't trigger diffs)
    srcText = _read_text_safe(src)
    dstText = _read_text_safe(dst)

    if srcText is None or dstText is None:
        return ""

    srcNorm = _normalize_for_compare(src, srcText).splitlines(keepends=True)
    dstNorm = _normalize_for_compare(dst, dstText).splitlines(keepends=True)

    diffLines = difflib.unified_diff(
        dstNorm,
        srcNorm,
        fromfile=str(dst),
        tofile=str(src),
        lineterm="",
    )
    return "\n".join(diffLines)


def files_differ(src: Path, dst: Path, compare: str) -> bool:
    # dst is assumed to exist
    if compare == "size":
        return src.stat().st_size != dst.stat().st_size

    if compare == "mtime":
        return src.stat().st_mtime != dst.stat().st_mtime

    if compare == "lines":
        return _count_lines(src) != _count_lines(dst)

    if compare == "diff":
        d = make_unified_diff(src, dst)
        return d.strip() != ""

    # default: hash (normalized for compare where needed)
    return _calc_sha256_for_compare(src) != _calc_sha256_for_compare(dst)


def _format_stats(label: str, stats: FileStats) -> str:
    return (
        f"{label}: size={stats.sizeBytes}B, "
        f"mtime={stats.mtime:.0f}, "
        f"lines={stats.lineCount}, "
        f"sha256={stats.sha256[:12]}…"
    )


def _next_backup_path(dst: Path, backup_suffix: str) -> Path:
    base = Path(str(dst) + backup_suffix)
    if not base.exists():
        return base

    i = 1
    while True:
        candidate = Path(str(base) + f".{i}")
        if not candidate.exists():
            return candidate
        i += 1


def prompt_existing_file_action(
    src: Path,
    dst: Path,
    compare: str,
    show_diff: bool,
    backup_suffix: str,
) -> str:
    # returns: "skip" | "overwrite" | "backup_overwrite"
    srcStats = get_file_stats(src)
    dstStats = get_file_stats(dst)

    diffText = ""
    if compare == "diff" or show_diff:
        diffText = make_unified_diff(src, dst)

    print("")
    print(f"⚠️  Bestaat al: {dst}")
    print(_format_stats("  target ", dstStats))
    print(_format_stats("  template", srcStats))

    if _is_tag_release_yml(dst) or _is_tag_release_yml(src):
        print("ℹ️  Let op: PROGRAM_NAME/PROGRAM_SRC/PROGRAM_DIR verschillen in tag-release.yml worden genegeerd voor compare/diff.")

    if show_diff and diffText.strip():
        print("")
        print("----- diff (normalized) -----")
        print(diffText)
        print("-----------------------------")

    if not sys.stdin.isatty():
        print("ℹ️  Geen interactieve TTY gedetecteerd; overslaan.")
        return "skip"

    while True:
        print("")
        print("Kies actie: [s]kip, [o]verwrite, [b]ackup+overwrite, [d]iff tonen")
        choice = input("> ").strip().lower()

        if choice in ["s", "skip", ""]:
            return "skip"

        if choice in ["o", "overwrite"]:
            return "overwrite"

        if choice in ["b", "backup"]:
            backupPath = _next_backup_path(dst, backup_suffix)
            print(f"🧷 Backup wordt gemaakt als: {backupPath}")
            return "backup_overwrite"

        if choice in ["d", "diff"]:
            if not diffText:
                diffText = make_unified_diff(src, dst)

            if diffText.strip():
                print("")
                print("----- diff (normalized) -----")
                print(diffText)
                print("-----------------------------")
            else:
                print("ℹ️  Geen tekst-diff beschikbaar (mogelijk binair of identiek na normalisatie).")
            continue

        print("❓ Onbekende keuze. Gebruik s/o/b/d.")


def copy_tree_with_policy(
    src: Path,
    dst: Path,
    on_existing: str,
    compare: str,
    show_diff: bool,
    backup_suffix: str,
) -> tuple[int, int, int]:
    """
    Copy src into dst, recursively.

    Returns: (copied_files, skipped_files, overwritten_files)
    """
    copied = 0
    skipped = 0
    overwritten = 0

    def handle_file(srcFile: Path, dstFile: Path) -> None:
        nonlocal copied, skipped, overwritten

        dstFile.parent.mkdir(parents=True, exist_ok=True)

        if not dstFile.exists():
            shutil.copy2(srcFile, dstFile)
            copied += 1
            return

        # dst exists
        if on_existing == "skip":
            skipped += 1
            return

        if on_existing == "overwrite":
            shutil.copy2(srcFile, dstFile)
            overwritten += 1
            return

        # on_existing == "ask"
        if not files_differ(srcFile, dstFile, compare):
            skipped += 1
            return

        action = prompt_existing_file_action(
            src=srcFile,
            dst=dstFile,
            compare=compare,
            show_diff=show_diff,
            backup_suffix=backup_suffix,
        )

        if action == "skip":
            skipped += 1
            return

        if action == "backup_overwrite":
            backupPath = _next_backup_path(dstFile, backup_suffix)
            shutil.copy2(dstFile, backupPath)
            shutil.copy2(srcFile, dstFile)
            overwritten += 1
            return

        shutil.copy2(srcFile, dstFile)
        overwritten += 1

    if src.is_file():
        handle_file(src, dst)
        return (copied, skipped, overwritten)

    for path in src.rglob("*"):
        rel = path.relative_to(src)
        target = dst / rel

        if path.is_dir():
            target.mkdir(parents=True, exist_ok=True)
            continue

        handle_file(path, target)

    return (copied, skipped, overwritten)


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
        description="Apply mrWheel/templateRepo files to current repo (ask on differences by default) and enable git hooks."
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
    ap.add_argument(
        "--on-existing",
        choices=["skip", "ask", "overwrite"],
        default="ask",
        help="What to do when target file already exists (default: ask)",
    )
    ap.add_argument(
        "--compare",
        choices=["hash", "mtime", "size", "lines", "diff"],
        default="hash",
        help="How to detect differences when --on-existing ask is used (default: hash)",
    )
    ap.add_argument(
        "--show-diff",
        action="store_true",
        help="When asking on existing files, show unified diff automatically (normalized for known ignored keys).",
    )
    ap.add_argument(
        "--backup-suffix",
        default=".bak",
        help="Suffix for backups when choosing backup+overwrite (default: .bak)",
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
        total_overwritten = 0

        for rel in args.paths:
            src = template_dir / rel
            dst = repo_root / rel

            if not src.exists():
                print(f"⚠️  Bestaat niet in template, overslaan: {rel}")
                continue

            dst.mkdir(parents=True, exist_ok=True) if src.is_dir() else None

            copied, skipped, overwritten = copy_tree_with_policy(
                src=src,
                dst=dst,
                on_existing=args.on_existing,
                compare=args.compare,
                show_diff=args.show_diff,
                backup_suffix=args.backup_suffix,
            )

            total_copied += copied
            total_skipped += skipped
            total_overwritten += overwritten

            extra = ""
            if args.on_existing != "skip":
                extra = f", ~{overwritten} overschreven"

            print(f"✅ {rel}: +{copied} gekopieerd, {skipped} overgeslagen (bestonden al){extra}")

    # Enable hooks
    hooks_dir = repo_root / args.hooks_path
    if hooks_dir.exists():
        ensure_exec_bits(hooks_dir)
        set_hooks_path(repo_root, args.hooks_path)
        print(f"🪝 Git hooks geactiveerd: core.hooksPath = {args.hooks_path}")
    else:
        print(f"⚠️  Hooks-map niet gevonden ({args.hooks_path}); core.hooksPath niet gezet.", file=sys.stderr)

    summaryExtra = ""
    if args.on_existing != "skip":
        summaryExtra = f", ~{total_overwritten} overschreven"

    print(f"🎉 Klaar. Totaal: +{total_copied} gekopieerd, {total_skipped} overgeslagen{summaryExtra}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
