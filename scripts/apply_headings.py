# -*- coding: utf-8 -*-
"""Apply heading heuristic to existing posts that lack markdown headings.

Converts flat structural lines to ## markdown headings for posts
that currently have zero #-style headings.
"""
import re
from pathlib import Path

POSTS_DIR = Path("content/posts")

# Patterns that signal a likely section heading
HEADING_PATTERNS = [
    # "1.xxx" or "1,xxx" (Chinese comma)
    (re.compile(r"^(\d+)[.、．]\s*(\S.*)$", re.M), r"## \1. \2"),
    # Chinese numbered: "一、xxx" etc
    (
        re.compile(
            r"^([一二三四五六七八九十]+)[、．.]\s*(\S.*)$",
            re.M,
        ),
        r"## \1. \2",
    ),
    # "Chapter X xxx"
    (
        re.compile(
            r"^(第[一二三四五六七八九十\d]+[章节部分])\s*(\S.*)$",
            re.M,
        ),
        r"## \1 \2",
    ),
]

FRONT_MATTER_RE = re.compile(
    r"\A(?:---|\+\+\+)\s*\n.*?\n(?:---|\+\+\+)\s*\n?", re.S
)

# Characters that indicate a line is NOT a heading
SKIP_FIRST_CHARS = set(
    "!\"#$%&'()*+,-./:;<=>?@[\\]^_`{|}~"
    "、。，．：；！？“”‘’"
)


def has_markdown_headings(text: str) -> bool:
    body = FRONT_MATTER_RE.sub("", text, count=1)
    return bool(re.search(r"^#{1,6}\s+", body, re.M))


def looks_like_heading(line: str) -> bool:
    """Heuristic: a short line that isn't punctuation/bullet and ends without sentence-final punctuation."""
    stripped = line.strip()
    if not stripped or len(stripped) > 40:
        return False
    if stripped[0] in SKIP_FIRST_CHARS:
        return False
    if re.match(r"^\d+$", stripped):
        return False
    # Don't convert lines ending with sentence punctuation
    if re.search(r"[。．！？.!?]$", stripped):
        return False
    return True


def apply_heading_heuristic(body: str) -> str:
    """Convert flat structural lines to ## markdown headings."""
    if has_markdown_headings(body):
        return body

    # Phase 1: explicit numbered/chapter patterns
    for pattern, replacement in HEADING_PATTERNS:
        body = pattern.sub(replacement, body)

    if has_markdown_headings(body):
        return body

    # Phase 2: short standalone lines that look like sub-headings
    lines = body.split("\n")
    converted = set()
    for i, line in enumerate(lines):
        if i == 0:
            continue
        if i in converted:
            continue
        stripped = line.strip()
        if not looks_like_heading(stripped):
            continue

        # Preceded by blank line (or start of body)?
        prev_blank = i == 0 or not lines[i - 1].strip()
        if not prev_blank:
            continue

        # Followed by substantive text (longer line)?
        next_is_text = False
        for j in range(i + 1, min(i + 3, len(lines))):
            nxt = lines[j].strip()
            if nxt and len(nxt) > 15 and not nxt.startswith("#"):
                next_is_text = True
                break
            if nxt and len(nxt) <= 40:
                break  # another short line, don't convert
        if next_is_text:
            lines[i] = "### " + stripped
            converted.add(i)

    return "\n".join(lines)


def main() -> None:
    fixed = 0
    for md_path in sorted(POSTS_DIR.glob("*.md")):
        text = md_path.read_text(encoding="utf-8")
        if has_markdown_headings(text):
            continue
        new_text = apply_heading_heuristic(text)
        if new_text != text:
            md_path.write_text(new_text, encoding="utf-8", newline="\n")
            fixed += 1
            new_headings = len(re.findall(r"^#{2,3} ", new_text, re.M))
            print("[%d] %s -> %d headings" % (fixed, md_path.name, new_headings))

    print("\nFixed: %d posts" % fixed)


if __name__ == "__main__":
    main()
