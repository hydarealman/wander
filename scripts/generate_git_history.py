"""Generate per-file git commit history as Hugo data JSON.

Run before `hugo` build so that layouts can access .Site.Data.git_history.
Requires: git (available), Python 3 stdlib only.
"""
import json
import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
POSTS_DIR = REPO_ROOT / "content" / "posts"
OUTPUT_FILE = REPO_ROOT / "data" / "git_history.json"

ENCODING = "utf-8"


def git_log(relative_path: Path) -> list[dict]:
    """Return commit list for *relative_path* (newest first)."""
    cmd = [
        "git", "log", "--follow",
        "--format=%H|%an|%aI|%s",
        "--", str(relative_path),
    ]
    result = subprocess.run(
        cmd, capture_output=True, text=True, encoding=ENCODING,
        cwd=str(REPO_ROOT), errors="replace",
    )
    if result.returncode != 0:
        print(f"  WARN: git log failed for {relative_path}: {result.stderr.strip()}", file=sys.stderr)
        return []

    commits = []
    for line in result.stdout.strip().splitlines():
        line = line.strip()
        if not line:
            continue
        parts = line.split("|", 3)
        if len(parts) < 4:
            continue
        full_hash, author, date_str, subject = parts
        commits.append({
            "hash": full_hash,
            "shortHash": full_hash[:7],
            "author": author.strip(),
            "date": date_str.strip(),
            "subject": subject.strip(),
        })
    return commits


def main() -> int:
    if not POSTS_DIR.is_dir():
        print(f"ERROR: posts directory not found: {POSTS_DIR}", file=sys.stderr)
        return 1

    md_files = sorted(POSTS_DIR.glob("*.md"))
    if not md_files:
        print("No .md files found in content/posts/ — writing empty JSON.")
        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_FILE.write_text("{}", encoding=ENCODING)
        return 0

    data: dict[str, dict] = {}
    total_commits = 0

    for md_path in md_files:
        relative = md_path.relative_to(REPO_ROOT)  # e.g. content/posts/foo.md
        commits = git_log(relative)
        # Hugo .File.Path is relative to content/  → e.g. posts/foo.md
        hugo_key = str(md_path.relative_to(REPO_ROOT / "content")).replace("\\", "/")
        data[hugo_key] = {"commits": commits}
        total_commits += len(commits)
        if not commits:
            print(f"  WARN: no commits for {hugo_key}", file=sys.stderr)

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_FILE.write_text(
        json.dumps(data, ensure_ascii=False, indent=2) + "\n",
        encoding=ENCODING,
    )

    print(f"git_history: {len(data)} files, {total_commits} total commits → {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
