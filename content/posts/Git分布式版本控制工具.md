---
title: "用于输出git提交日志"
slug: "git分布式版本控制工具"
date: 1970-01-21T22:38:48+08:00
draft: false
source_file: "feishu://git分布式版本控制工具"
source_size: 3549
source_lines: 144
tags:
  - "工具"
categories:
  - "编程开发"
---

Git分布式版本控制工具
1.基本命令行
ls/ll   查看当前目录

cat    查看文件内容

touch  创建文件

vi       vi编辑器

2.创建.bashrc文件
打开用户目录,创建.bashrc文件
在.bashrc文件中输入


# 用于输出git提交日志
alias git-log='git log --pretty=oneline --all --graph --abbrev-commit'
# 用于输出当前目录所有文件及其基本信息
alias ll='ls -al'


3.解决GitBash乱码问题
1.打开GitBash执行下面指令
Git config --global core.quotepath false

2.&{git_home}/etc/bash.bashrc 文件最后加下面两行

export LANG="zh_CN.UTF-8"
export LC_ALL="zh_CN.UTF-8"

4 获取本地仓库
1) 在电脑的任意位置创建一个空目录作为我们的本地Git仓库
2) 进入这个目录中,电机右键Git bash窗口
3) 执行命令git init
4) 如果创建成功后可在文件夹下看到隐藏的.git目录 

基础操作指令
Git工作目录下对文件的修改(增加,删除,更新)会存在几个状态,这些修改的状态会随着我们执行Git的命令而发生变化
image.png

1.git add         (工作区 -> 暂存区)
git add . 将所有修改加入暂存区
2.git commit -m "注释内容"    (暂存区 -> 本地仓库)

仓库(repositor)    暂存区(index)    工作区(workspace)
查看修改的状态(status)
作用: 查看的修改的状态 (残存区,工作区)
命令形式: git status

3.git log[options]             (查看日志)
作用:查看提交记录
--all 显示所有分支
--pretty=oneline 将所有信息显示为一行
--abbrev-commit 使得输出的commit更简短
--graph 以图的形式显示

4. 版本回退
git reset --hard commitID    (commitID可以使用git-log或git log指令查看)
git reflog                           (查看已经删除的记录)

5.添加文件至忽略列表
--pretty=oneline 

5.修改用户名和邮箱地址
修改用户名: 

git config --global user.name "hydarealman"
修改邮箱地址: 

git config --global user.email 2281306133@qq.com

6.如何查看git的邮箱地址和用户名是否配置成功
可以在这个路径下面找到: 
路径地址
C:\Users\dong

朋友给的笔记
git init //初始化仓库

git add .

git commit -m "message" //先保存本地的工作进度，避免被pull覆盖

git pull origin main //从远程仓库拉取代码到本地仓库分支，与远程仓库的新代码合并
        [主机名][分支名]
git push origin main



将我的代推送到我的仓库

# 第一次提交
git init
git branch -M main
git add .
git commit -m "..."
git remote add origin https://github.com/hydarealman/ws_glut_vision.git // 网址自定义
git push -u origin main




# 再添加
git add .
git commit -m "..."
git push




使用git diff查看两段代码的差异
使用git diff对比两个文件的差异
文件A为旧文件
文件B为新文件

code --diff 文件夹A/ 文件夹B/



虚拟机上使用SSH  git推送到github仓库

# 在 Ubuntu 虚拟机中执行

# 1. 清除 Git 代理
git config --global --unset http.proxy
git config --global --unset https.proxy

# 2. 生成 SSH 密钥（如果已有可以跳过）
ssh-keygen -t ed25519 -C "your_email@example.com"

# 3. 查看公钥并复制输出
cat ~/.ssh/id_ed25519.pub

# 4. 在浏览器中登录 GitHub，进入 Settings -> SSH and GPG keys -> New SSH key，粘贴并保存

# 5. 修改本地仓库的远程地址为 SSH 格式
git remote set-url origin git@github.com:hydarealman/AimScope.git

# 6. 推送
git push -u origin main


![图片 1](/images/feishu/git分布式版本控制工具/image_8630fa.png)
