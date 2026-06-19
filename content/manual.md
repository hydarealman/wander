---
title: "使用说明书"
summary: "博客维护、置顶、排序、预览和部署说明"
url: "/manual/"
searchHidden: true
---

## 1. 本地预览

在 PowerShell 中执行：

```powershell
cd D:\wander
powershell -ExecutionPolicy Bypass -File .\scripts\serve.ps1
```

然后打开：

```text
http://127.0.0.1:1313/
```

## 2. 导入笔记

当 `D:\notes` 里的 Markdown 有新增或修改时，执行：

```powershell
cd D:\wander
python .\scripts\import_notes.py
```

脚本会做这些事：

- 扫描 `D:\notes` 下所有 `.md` 文件。
- 导入到 `D:\wander\content\posts`。
- 自动生成 front matter。
- 保留目录结构。
- 自动生成标题、日期、标签、分类。
- 写入文件大小、行数等索引字段。
- 按 `data/pinned_posts.toml` 应用置顶规则。

## 3. 置顶文章

置顶配置文件：

```text
D:\wander\data\pinned_posts.toml
```

也可以用脚本操作：

```powershell
cd D:\wander
python .\scripts\pin_post.py list
python .\scripts\pin_post.py add "桂工自瞄文档.md" 1
python .\scripts\pin_post.py add "Git分布式版本控制工具.md" 2
python .\scripts\pin_post.py add "C_C++代码中积累的语法.md" 3
python .\scripts\import_notes.py
```

`weight` 越小越靠前。

取消置顶：

```powershell
python .\scripts\pin_post.py remove "文章文件名.md"
python .\scripts\import_notes.py
```

## 4. 文章索引和排序

打开：

```text
/library/
```

可以按这些属性排序：

- 置顶优先
- 时间新旧
- 标题
- 内容字数
- 阅读时间
- 文件大小
- 本机访问次数

还可以按关键词、分类、标签筛选。

注意：当前“访问次数”是浏览器本机统计，保存在 localStorage 中。GitHub Pages 是静态站点，没有数据库，所以它不是全网真实访问量。如果需要全站访问量，需要后续接入 Umami、GoatCounter、Google Analytics 或类似统计服务。

## 5. 点赞系统

每篇文章底部都有点赞按钮。默认模式是“本机点赞”，只保存在当前浏览器里。

如果要让所有访问者共享点赞数，需要配置 Supabase：

1. 新建 Supabase 项目。
2. 进入 SQL Editor，执行下面 SQL：

```sql
create table if not exists public.doc_likes (
  doc_id text primary key,
  title text,
  count integer not null default 0,
  updated_at timestamptz not null default now()
);

alter table public.doc_likes enable row level security;

drop policy if exists "Public read likes" on public.doc_likes;
create policy "Public read likes"
on public.doc_likes
for select
to anon
using (true);

create or replace function public.increment_doc_like(
  p_doc_id text,
  p_title text default null
)
returns integer
language plpgsql
security definer
set search_path = public
as $$
declare
  new_count integer;
begin
  insert into public.doc_likes(doc_id, title, count, updated_at)
  values (p_doc_id, p_title, 1, now())
  on conflict (doc_id)
  do update set
    count = public.doc_likes.count + 1,
    title = coalesce(excluded.title, public.doc_likes.title),
    updated_at = now()
  returning count into new_count;

  return new_count;
end;
$$;

grant execute on function public.increment_doc_like(text, text) to anon;
```

3. 在 Supabase 的 Project Settings -> API 中复制 Project URL 和 anon public key。
4. 修改 `D:\wander\hugo.toml`：

```toml
  [params.likeSystem]
    provider = "supabase"
    supabaseUrl = "https://你的项目.supabase.co"
    supabaseAnonKey = "你的 anon public key"
    table = "doc_likes"
```

5. 重新构建并推送。

注意：anon public key 可以放在前端，但不要放 service_role key。

## 6. 发布到 GitHub Pages

修改、导入、检查完成后执行：

```powershell
cd D:\wander
hugo --gc --minify --ignoreCache
git add .
git commit -m "Update blog"
git push
```

推送后 GitHub Actions 会自动部署。

线上地址：

```text
https://hydarealman.github.io/wander/
```

## 7. 常见问题

如果 `hugo` 命令找不到，使用：

```powershell
powershell -ExecutionPolicy Bypass -File .\scripts\serve.ps1
```

如果图片显示失败，多半是原 Markdown 里使用了飞书临时图片链接。需要从飞书重新导出“包含附件”的 Markdown 包，或者先忽略图片。

如果 GitHub Pages 404，检查：

- GitHub Actions 是否成功。
- Settings -> Pages -> Source 是否为 GitHub Actions。
- 访问地址是否为 `https://hydarealman.github.io/wander/`。
