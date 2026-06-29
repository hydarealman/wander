from pathlib import Path
from datetime import datetime
import re


SOURCE_DIR = Path(r"D:\notes")
DEST_DIR = Path(r"D:\wander\content\posts")
STATIC_MEDIA_DIR = Path(r"D:\wander\static\media")

FRONT_MATTER_RE = re.compile(r"\A(?:---|\+\+\+)\s*\n.*?\n(?:---|\+\+\+)\s*\n?", re.S)
HEADING_RE = re.compile(r"^\s*#\s+(.+?)\s*#*\s*$", re.M)
LEADING_HTML_COMMENT_RE = re.compile(r"\A\s*<!--.*?-->\s*", re.S)
MARKDOWN_IMAGE_RE = re.compile(r"!\[([^\]]*)\]\(([^)]+)\)")
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".gif", ".webp", ".svg", ".bmp"}

TAG_RULES = [
    ("AI", ["ai", "prompt", "openvino", "大模型", "claude"]),
    ("C/C++", ["c++", "c_c++", "cpp", "qt"]),
    ("ROS", ["ros1", "ros2", "slam"]),
    ("机器人", ["自瞄", "视觉", "stm32", "嵌入式", "导航", "相机", "三维", "点云", "机械臂", "jakac5", "吊车", "防碰撞", "vision", "3v3", "7v7", "滤波器", "传感器", "雷达"]),
    ("算法", ["leetcode", "蓝桥", "kalman", "mpc", "数学建模", "滤波"]),
    ("工具", ["git", "docker", "linux", "vscode", "tmux", "sql", "技术栈", "ssh", "labelme", "debug", "快捷键", "ch343", "驱动", "数据库", "前后端", "rust", "并发", "cmake", "makefile"]),
    ("读书", ["文献", "阅读", "深入理解计算机系统", "xv6", "操作系统", "实习报告", "培训", "lab_plant"]),
    ("复盘", ["复盘", "备赛", "赛季", "汇报", "总结"]),
    ("面经", ["牛客", "面经", "面试"]),
]

CATEGORY_RULES = [
    ("机器人视觉", ["自瞄", "视觉", "相机", "三维", "点云", "slam", "ros", "吊车", "防碰撞", "完整形态", "3v3", "7v7", "赛季", "机械臂", "jakac5", "导航", "滤波器"]),
    ("编程开发", ["c++", "git", "docker", "linux", "vscode", "sql", "rust", "前后端", "ssh", "tmux", "qt", "数据库", "并发", "claude", "debug", "快捷键"]),
    ("算法数学", ["leetcode", "蓝桥", "kalman", "mpc", "数学建模", "滤波"]),
    ("学习记录", ["文献", "阅读", "培训", "指南", "复盘", "计划", "汇报", "技术栈", "操作系统", "实习报告", "面经", "牛客"]),
    ("硬件嵌入式", ["stm32", "ch343", "嵌入式", "驱动", "物料", "单片机", "arm"]),
]


def read_text(path: Path) -> str:
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    return path.read_text(encoding="utf-8", errors="replace")


def strip_existing_front_matter(text: str) -> str:
    return FRONT_MATTER_RE.sub("", text, count=1)


def extract_title(path: Path, _text: str) -> str:
    # Use filename (without .md) as title — the document name IS the title.
    return clean_title(path.stem)


def clean_title(value: str) -> str:
    value = re.sub(r"[*_`~\[\]<>]", "", value)
    value = re.sub(r"\s+", " ", value).strip()
    return value or "Untitled"


# ── Heading heuristic ──────────────────────────
# Converts flat numbered lines (common in Feishu exports) to markdown ## headings.
# Only applied to posts that have zero existing #-style headings.

HEADING_CANDIDATE_RE = re.compile(
    r"^(\d+)[\.、．]\s*(.+)$|"             # "1.xxx" or "1、xxx"
    r"^(第[一二三四五六七八九十\d]+[章节部分])\s*(.+)$",  # "第一章 xxx"
    re.M,
)


def has_markdown_headings(text: str) -> bool:
    """Return True if text contains any #-style markdown heading."""
    return bool(re.search(r"^#{1,6}\s+", text, re.M))


def apply_heading_heuristic(body: str) -> str:
    """Convert numbered flat lines to ## headings, but only if there are no
    existing markdown headings (to avoid double-processing)."""
    if has_markdown_headings(body):
        return body

    def heading_replacer(m: re.Match) -> str:
        if m.group(1):  # "1.xxx" or "1、xxx"
            return f"## {m.group(1)}. {m.group(2)}"
        elif m.group(3):  # "第一章 xxx"
            return f"## {m.group(3)} {m.group(4)}"
        return m.group(0)

    return HEADING_CANDIDATE_RE.sub(heading_replacer, body)


def yaml_quote(value: str) -> str:
    escaped = value.replace("\\", "\\\\").replace('"', '\\"')
    escaped = escaped.replace("\r", "\\r").replace("\n", "\\n")
    return f'"{escaped}"'


def tags_from_relative_path(relative_path: Path) -> list[str]:
    tags = []
    for part in relative_path.parent.parts:
        tag = part.strip()
        if tag and tag not in tags:
            tags.append(tag)
    return tags


def keyword_text(relative_path: Path, title: str) -> str:
    return f"{relative_path.as_posix()} {title}".lower()


def infer_terms(relative_path: Path, title: str, rules: list[tuple[str, list[str]]]) -> list[str]:
    haystack = keyword_text(relative_path, title)
    terms = []
    for term, keywords in rules:
        if any(keyword.lower() in haystack for keyword in keywords):
            terms.append(term)
    return terms


def merge_unique(*groups: list[str]) -> list[str]:
    values = []
    for group in groups:
        for item in group:
            if item and item not in values:
                values.append(item)
    return values


def slug_from_relative_path(relative_path: Path) -> str:
    raw = "-".join(relative_path.with_suffix("").parts).lower()
    raw = raw.replace("\\", "-").replace("/", "-")
    raw = re.sub(r"[\\/:*?\"<>|#%{}^~\[\]`]+", "-", raw)
    raw = re.sub(r"[()（）]+", "-", raw)
    raw = re.sub(r"\s+", "-", raw)
    raw = re.sub(r"-{2,}", "-", raw)
    return raw.strip("-. ") or "post"


def is_remote_or_special_link(target: str) -> bool:
    lowered = target.lower()
    return (
        lowered.startswith("http://")
        or lowered.startswith("https://")
        or lowered.startswith("data:")
        or lowered.startswith("#")
        or lowered.startswith("/")
    )


def markdown_asset_target(raw_target: str) -> str:
    target = raw_target.strip()
    if target.startswith("<") and target.endswith(">"):
        target = target[1:-1].strip()
    return target


def copy_local_image(markdown_path: Path, relative_markdown_path: Path, raw_target: str) -> str | None:
    target = markdown_asset_target(raw_target)
    if is_remote_or_special_link(target):
        return None

    source_image = (markdown_path.parent / target).resolve()
    if not source_image.exists() or source_image.suffix.lower() not in IMAGE_EXTENSIONS:
        return None

    media_subdir = STATIC_MEDIA_DIR / relative_markdown_path.with_suffix("")
    media_subdir.mkdir(parents=True, exist_ok=True)

    destination = media_subdir / source_image.name
    destination.write_bytes(source_image.read_bytes())

    url_path = destination.relative_to(STATIC_MEDIA_DIR.parent).as_posix()
    return "/" + url_path


def rewrite_local_image_links(markdown_path: Path, relative_markdown_path: Path, text: str) -> tuple[str, int, int]:
    copied = 0
    remote = 0

    def replace(match: re.Match[str]) -> str:
        nonlocal copied, remote
        alt_text = match.group(1)
        raw_target = match.group(2)
        target = markdown_asset_target(raw_target)

        if target.lower().startswith(("http://", "https://")):
            remote += 1
            return match.group(0)

        new_url = copy_local_image(markdown_path, relative_markdown_path, raw_target)
        if not new_url:
            return match.group(0)

        copied += 1
        return f"![{alt_text}]({new_url})"

    return MARKDOWN_IMAGE_RE.sub(replace, text), copied, remote


def front_matter(path: Path, relative_path: Path, text: str) -> str:
    modified = datetime.fromtimestamp(path.stat().st_mtime).astimezone()
    # Fallback: if file timestamp is epoch-era (pre-2020), use ctime or current time
    if modified.year < 2020:
        ctime = datetime.fromtimestamp(path.stat().st_ctime).astimezone()
        if ctime.year >= 2020:
            modified = ctime
        else:
            modified = datetime.now().astimezone()
            print(f"  WARN: {relative_path.as_posix()} has broken timestamp, using current time")
    date = modified.isoformat(timespec="seconds")
    title = extract_title(path, text)
    tags = merge_unique(tags_from_relative_path(relative_path), infer_terms(relative_path, title, TAG_RULES))
    categories = infer_terms(relative_path, title, CATEGORY_RULES)
    source_size = path.stat().st_size
    source_lines = text.count("\n") + 1 if text else 0

    lines = [
        "---",
        f"title: {yaml_quote(title)}",
        f"slug: {yaml_quote(slug_from_relative_path(relative_path))}",
        f"date: {date}",
        "draft: false",
        f"source_file: {yaml_quote(relative_path.as_posix())}",
        f"source_size: {source_size}",
        f"source_lines: {source_lines}",
    ]

    if tags:
        lines.append("tags:")
        for tag in tags:
            lines.append(f"  - {yaml_quote(tag)}")
    else:
        lines.append("tags: []")

    if categories:
        lines.append("categories:")
        for category in categories:
            lines.append(f"  - {yaml_quote(category)}")
    else:
        lines.append("categories: []")

    lines.append("---")
    return "\n".join(lines) + "\n\n"


def import_file(path: Path) -> Path:
    relative_path = path.relative_to(SOURCE_DIR)
    destination = DEST_DIR / relative_path
    text = read_text(path)
    body = strip_existing_front_matter(text).lstrip("\ufeff")
    body, copied_images, remote_images = rewrite_local_image_links(path, relative_path, body)
    body = apply_heading_heuristic(body)

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(front_matter(path, relative_path, text) + body, encoding="utf-8", newline="\n")
    return destination, copied_images, remote_images


def main() -> None:
    if not SOURCE_DIR.exists():
        raise SystemExit(f"Source directory does not exist: {SOURCE_DIR}")

    DEST_DIR.mkdir(parents=True, exist_ok=True)
    markdown_files = sorted(SOURCE_DIR.rglob("*.md"))

    imported = 0
    copied_images = 0
    remote_images = 0
    for path in markdown_files:
        if path.is_file():
            _, file_copied_images, file_remote_images = import_file(path)
            imported += 1
            copied_images += file_copied_images
            remote_images += file_remote_images

    print(f"Imported {imported} Markdown files")
    print(f"Copied {copied_images} local images")
    print(f"Kept {remote_images} remote image links")
    print(f"Source: {SOURCE_DIR}")
    print(f"Destination: {DEST_DIR}")


if __name__ == "__main__":
    main()
