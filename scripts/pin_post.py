from pathlib import Path
import sys
import tomllib


PINNED_POSTS_FILE = Path(r"D:\wander\data\pinned_posts.toml")


def load_posts() -> list[dict[str, object]]:
    if not PINNED_POSTS_FILE.exists():
        return []
    data = tomllib.loads(PINNED_POSTS_FILE.read_text(encoding="utf-8"))
    return list(data.get("posts", []))


def save_posts(posts: list[dict[str, object]]) -> None:
    PINNED_POSTS_FILE.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "# 置顶文章配置。weight 越小越靠前。",
        "# path 使用 D:\\notes 下的相对路径，和 content/posts 中的文件名通常一致。",
        "",
    ]
    for post in sorted(posts, key=lambda item: int(item["weight"])):
        lines.extend([
            "[[posts]]",
            f'path = "{str(post["path"]).replace(chr(92), "/")}"',
            f'weight = {int(post["weight"])}',
            "",
        ])
    PINNED_POSTS_FILE.write_text("\n".join(lines), encoding="utf-8", newline="\n")


def print_usage() -> None:
    print("Usage:")
    print(r'  python .\scripts\pin_post.py list')
    print(r'  python .\scripts\pin_post.py add "文章文件名.md" 1')
    print(r'  python .\scripts\pin_post.py remove "文章文件名.md"')
    print()
    print("Examples:")
    print(r'  python .\scripts\pin_post.py add "ROS2.md" 1')
    print(r'  python .\scripts\pin_post.py remove "ai使用指南.md"')


def main() -> None:
    posts = load_posts()
    if len(sys.argv) < 2:
        print_usage()
        return

    command = sys.argv[1].lower()

    if command == "list":
        if not posts:
            print("No pinned posts")
            return
        for post in sorted(posts, key=lambda item: int(item["weight"])):
            print(f'{post["weight"]}\t{post["path"]}')
        return

    if command == "add":
        if len(sys.argv) != 4:
            print_usage()
            raise SystemExit(1)
        path = sys.argv[2].replace("\\", "/")
        weight = int(sys.argv[3])
        posts = [post for post in posts if str(post.get("path", "")).replace("\\", "/") != path]
        posts.append({"path": path, "weight": weight})
        save_posts(posts)
        print(f"Pinned {path} with weight {weight}")
        return

    if command == "remove":
        if len(sys.argv) != 3:
            print_usage()
            raise SystemExit(1)
        path = sys.argv[2].replace("\\", "/")
        kept = [post for post in posts if str(post.get("path", "")).replace("\\", "/") != path]
        save_posts(kept)
        print(f"Removed {path} from pinned posts")
        return

    print_usage()
    raise SystemExit(1)


if __name__ == "__main__":
    main()
