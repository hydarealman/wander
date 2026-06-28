"""Fix post titles: set each post's title to match its filename (without .md)."""
import re
from pathlib import Path

POSTS_DIR = Path("content/posts")

TITLE_RE = re.compile(r'^title:\s*(.+?)\s*$', re.M)

def fix_titles() -> None:
    fixed = 0
    for md_path in sorted(POSTS_DIR.glob("*.md")):
        text = md_path.read_text(encoding="utf-8")
        m = TITLE_RE.search(text)
        if not m:
            print(f"SKIP (no title): {md_path.name}")
            continue

        current = m.group(1).strip().strip('"').strip("'")
        expected = md_path.stem  # filename without .md

        if current == expected:
            continue

        # Replace the first title: line with the expected title
        new_text, count = TITLE_RE.subn(
            f'title: "{expected}"',
            text,
            count=1,
        )
        if count == 1:
            md_path.write_text(new_text, encoding="utf-8", newline="\n")
            fixed += 1
            print(f"[{fixed}] {md_path.name}")
            print(f"     OLD: {current}")
            print(f"     NEW: {expected}")

    print(f"\nTotal fixed: {fixed}")

if __name__ == "__main__":
    fix_titles()
