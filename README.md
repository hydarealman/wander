# wander

这是一个 Hugo + PaperMod 搭建的个人博客项目，会把 `D:\notes` 下的 Markdown 笔记导入到 `content/posts`。

## 环境

```powershell
git --version
go version
hugo version
```

当前已验证：

- Git for Windows: 已安装
- Go: 已安装
- Hugo Extended: 已安装

如果换到新机器，可以用：

```powershell
winget install --id Git.Git --exact --source winget --accept-package-agreements --accept-source-agreements
winget install --id GoLang.Go --exact --source winget --accept-package-agreements --accept-source-agreements
winget install --id Hugo.Hugo.Extended --exact --source winget --accept-package-agreements --accept-source-agreements
```

## 导入笔记

```powershell
cd D:\wander
python .\scripts\import_notes.py
```

脚本会递归扫描 `D:\notes` 下的 `.md` 文件，保留目录结构，生成 YAML front matter，并写入 `D:\wander\content\posts`。

## 置顶文章

置顶规则放在 `data/pinned_posts.toml`，也可以用脚本管理：

```powershell
cd D:\wander
python .\scripts\pin_post.py list
python .\scripts\pin_post.py add "ROS2.md" 1
python .\scripts\pin_post.py remove "ai使用指南.md"
python .\scripts\import_notes.py
```

`weight` 越小越靠前。重新导入后，脚本会自动把 `pinned: true` 和 `weight` 写入文章 front matter。

## 文章索引与排序

站点提供 `/library/` 页面，可以按多个属性浏览文章：

- 置顶优先
- 时间
- 标题
- 内容字数
- 阅读时间
- 文件大小
- 本机访问次数
- 分类和标签筛选

访问次数使用浏览器 localStorage 统计，是“本机阅读次数”，不是全站真实访问量。GitHub Pages 是静态站点，如果需要全站访问统计，需要额外接入第三方统计服务。

## 使用说明书

站点提供 `/manual/` 页面，记录本地预览、导入、置顶、排序和部署流程。

## 本地预览

```powershell
cd D:\wander
powershell -ExecutionPolicy Bypass -File .\scripts\serve.ps1
```

打开：

```text
http://127.0.0.1:1313/
```

## 构建静态文件

```powershell
cd D:\wander
hugo --gc --minify
```

生成结果在 `D:\wander\public`，该目录已加入 `.gitignore`，不用提交。

## 部署到 GitHub Pages

1. GitHub 仓库：`https://github.com/hydarealman/wander`。
2. GitHub Pages 地址：`https://hydarealman.github.io/wander/`。
3. 在仓库 Settings -> Pages 中把 Source 设置为 GitHub Actions。
4. 推送源码：

```powershell
cd D:\wander
git config core.autocrlf false
git add .
git commit -m "Create Hugo blog"
git branch -M main
git remote add origin https://github.com/hydarealman/wander.git
git push -u origin main
```

之后每次 push 到 `main`，`.github/workflows/deploy.yml` 会自动构建并部署。


## 点赞系统

文章页底部有点赞按钮。默认是本机点赞；如需全网共享点赞数，在 Supabase 建表后把 `hugo.toml` 中 `[params.likeSystem]` 的 `provider` 改为 `supabase`，并填入 Project URL 与 anon public key。详细 SQL 和步骤见 `/manual/`。

## 页面背景特效

页面背景中的黑洞视觉效果（吸积盘辉光 + 光子环 + 引力透镜晕）的灵感来源于 [NPGS](https://github.com/baopinshui/NPGS) 项目中的 `BlackHole_common.glsl` 着色器。该着色器基于 Kerr 度规进行光线追踪，渲染了带有相对论多普勒效应、光子环和吸积盘物理的黑洞图像。

本项目的 CSS 实现仅取其视觉意象，使用径向渐变和 CSS 动画模拟黑洞的吸积盘辉光与光子环效果，未直接使用原始 GLSL 代码。

### 许可声明

NPGS 项目遵循 **GNU General Public License v3 (GPL v3)**：

> NPGS — Copyright (C) baopinshui
> 
> This program is free software: you can redistribute it and/or modify
> it under the terms of the GNU General Public License as published by
> the Free Software Foundation, either version 3 of the License, or
> (at your option) any later version.
> 
> This program is distributed in the hope that it will be useful,
> but WITHOUT ANY WARRANTY; without even the implied warranty of
> MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
> GNU General Public License for more details.
> 
> You should have received a copy of the GNU General Public License
> along with this program.  If not, see <https://www.gnu.org/licenses/>.

完整的 GPL v3 许可证文本见 <https://www.gnu.org/licenses/gpl-3.0.html>。
