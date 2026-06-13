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


def read_text(path: Path) -> str:
    for encoding in ("utf-8-sig", "utf-8", "gb18030"):
        try:
            return path.read_text(encoding=encoding)
        except UnicodeDecodeError:
            continue
    return path.read_text(encoding="utf-8", errors="replace")


def strip_existing_front_matter(text: str) -> str:
    return FRONT_MATTER_RE.sub("", text, count=1)


def extract_title(path: Path, text: str) -> str:
    body = LEADING_HTML_COMMENT_RE.sub("", strip_existing_front_matter(text), count=1)
    match = HEADING_RE.search(body)
    if match:
        return clean_title(match.group(1))
    return clean_title(path.stem)


def clean_title(value: str) -> str:
    value = re.sub(r"[*_`~\[\]<>]", "", value)
    value = re.sub(r"\s+", " ", value).strip()
    return value or "Untitled"


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
    date = modified.isoformat(timespec="seconds")
    title = extract_title(path, text)
    tags = tags_from_relative_path(relative_path)

    lines = [
        "---",
        f"title: {yaml_quote(title)}",
        f"slug: {yaml_quote(slug_from_relative_path(relative_path))}",
        f"date: {date}",
        "draft: false",
    ]

    if tags:
        lines.append("tags:")
        for tag in tags:
            lines.append(f"  - {yaml_quote(tag)}")
    else:
        lines.append("tags: []")

    lines.append("---")
    return "\n".join(lines) + "\n\n"


def import_file(path: Path) -> Path:
    relative_path = path.relative_to(SOURCE_DIR)
    destination = DEST_DIR / relative_path
    text = read_text(path)
    body = strip_existing_front_matter(text).lstrip("\ufeff")
    body, copied_images, remote_images = rewrite_local_image_links(path, relative_path, body)

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
