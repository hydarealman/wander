---
title: "tmux简明速查手册\\(WSL/Linux通用\\)"
slug: "tmux简明速查手册-wsl_linux通用"
date: 2026-06-06T16:15:40+08:00
draft: false
tags: []
---

# tmux简明速查手册\(WSL/Linux通用\)

1\.安装

```C++
sudo apt update && sudo apt install tmux
```





2\.启动与退出

启动tmux

```C++
tmux
```

退出当前窗格

```C++
exit 或 Ctrl+d
```







3\.会话管理

```C++
分离会话（后台运行）     Ctrl+b 然后按 d
重新连接（回到上次现场） tmux attach
列出所有会话            tmux ls
连接到指定会话          tmux attach -t 会话名
新建命名会话            tmux new -s 会话名
```





4\.窗格管理 按键\(先按Ctrl\+b,再按下下一个键\)

垂直分割 %

水平分割 "

切换窗格 方向键（↑ ↓ ← →）

调整窗格大小 `Ctrl+b` 按住，然后按方向键

