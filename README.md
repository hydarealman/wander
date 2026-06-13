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
