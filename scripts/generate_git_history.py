"""Generate per-file git commit history as Hugo data JSON.

Run before `hugo` build so that layouts can access hugo.Data.git_history.
Requires: git (available), Python 3 stdlib only.

Output format:
  {
    "_all": [  ← sorted, deduplicated global commit list (newest first)
      {"hash": "...", "shortHash": "...", "author": "...", "date": "...",
       "subject": "...", "files": ["posts/a.md", "posts/b.md"]}
    ],
    "posts/foo.md": {"commits": [...]},
    ...
  }
"""
import json
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
POSTS_DIR = REPO_ROOT / "content" / "posts"
OUTPUT_FILE = REPO_ROOT / "data" / "git_history.json"

ENCODING = "utf-8"


def _parse_log(stdout: str) -> list[dict]:
    """Parse git log output into commit list."""
    commits = []
    for line in stdout.strip().splitlines():
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
    return _parse_log(result.stdout)


def git_log_site() -> list[dict]:
    """Return ALL commits in the repo (newest first), regardless of paths touched."""
    cmd = [
        "git", "log",
        "--format=%H|%an|%aI|%s",
    ]
    result = subprocess.run(
        cmd, capture_output=True, text=True, encoding=ENCODING,
        cwd=str(REPO_ROOT), errors="replace",
    )
    if result.returncode != 0:
        print(f"  WARN: git log --all failed: {result.stderr.strip()}", file=sys.stderr)
        return []
    return _parse_log(result.stdout)


def build_global_index(file_commits: dict[str, list[dict]]) -> list[dict]:
    """Merge all per-file commits into a deduplicated, date-sorted global list."""
    # hash -> merged entry
    seen: dict[str, dict] = {}

    for hugo_key, commits in file_commits.items():
        for c in commits:
            h = c["hash"]
            if h in seen:
                seen[h]["files"].append(hugo_key)
            else:
                seen[h] = {
                    "hash": c["hash"],
                    "shortHash": c["shortHash"],
                    "author": c["author"],
                    "date": c["date"],
                    "subject": c["subject"],
                    "files": [hugo_key],
                }

    # Sort newest first by date
    merged = sorted(seen.values(), key=lambda c: c["date"], reverse=True)
    return merged


def main() -> int:
    if not POSTS_DIR.is_dir():
        print(f"ERROR: posts directory not found: {POSTS_DIR}", file=sys.stderr)
        return 1

    md_files = sorted(POSTS_DIR.glob("*.md"))
    if not md_files:
        print("No .md files found in content/posts/ — writing empty JSON.")
        OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
        OUTPUT_FILE.write_text('{"_all": []}', encoding=ENCODING)
        return 0

    file_data: dict[str, list[dict]] = {}
    total_commits = 0

    for md_path in md_files:
        relative = md_path.relative_to(REPO_ROOT)  # e.g. content/posts/foo.md
        commits = git_log(relative)
        hugo_key = str(md_path.relative_to(REPO_ROOT / "content")).replace("\\", "/")
        file_data[hugo_key] = commits
        total_commits += len(commits)
        if not commits:
            print(f"  WARN: no commits for {hugo_key}", file=sys.stderr)

    # Build output
    global_commits = build_global_index(file_data)

    # Pre-group by date for Hugo templates (which can't mutate variables)
    by_date: dict[str, list[dict]] = {}
    for commit in global_commits:
        day = commit["date"][:10]  # YYYY-MM-DD
        by_date.setdefault(day, []).append(commit)
    date_groups = [
        {"date": day, "commits": commits}
        for day, commits in by_date.items()
    ]

    output: dict = {
        "_all": global_commits,
        "_by_date": date_groups,
        "_site": git_log_site(),
    }
    for hugo_key, commits in file_data.items():
        output[hugo_key] = {"commits": commits}

    OUTPUT_FILE.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT_FILE.write_text(
        json.dumps(output, ensure_ascii=False, indent=2) + "\n",
        encoding=ENCODING,
    )

    print(f"git_history: {len(file_data)} files, {total_commits} total commits, "
          f"{len(global_commits)} unique, {len(date_groups)} days → {OUTPUT_FILE}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
