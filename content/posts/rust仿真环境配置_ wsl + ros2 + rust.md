---
title: "rust仿真环境配置: wsl \\+ ros2 \\+ rust"
slug: "rust仿真环境配置_-wsl-+-ros2-+-rust"
date: 2026-06-06T16:15:30+08:00
draft: false
source_file: "rust仿真环境配置_ wsl + ros2 + rust.md"
source_size: 1194
source_lines: 123
tags:
  - "ROS"
categories:
  - "机器人视觉"
  - "编程开发"
---

# rust仿真环境配置: wsl \+ ros2 \+ rust

1\.以管理员身份打开PowerShell或CMD



2\.执行安装命令

```C++
wsl --install -d Ubuntu-24.04
```

3\.设置用户名和密码





列出所有的ubantu版本

```C++
wsl -l -v
```



进入想要的版本

```C++
wsl -d Ubuntu-24.04
```





复制文件到根目录

```C++
cp -r /mnt/c/Users/dong/Desktop/at_vision_simulator-master ~/
```





ROS2使用小鱼自动安装







更新系统并安装基础依赖

```C++
sudo apt update && sudo apt upgrade -y
sudo apt install -y \
    build-essential \
    pkg-config \
    libx11-dev \
    libasound2-dev \
    libudev-dev \
    libxkbcommon-x11-0 \
    libwayland-dev \
    libxkbcommon-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    curl \
    git \
    cmake
```







安装Rust

```C++
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

选择默认安装1

安装完成后,重新加载环境变量

```C++
source ~/.cargo/env
```

验证

```C++
cargo --version
```







更新系统并安装Vulkan驱动与工具

```C++
sudo apt update && sudo apt upgrade -y
sudo apt install -y mesa-vulkan-drivers vulkan-tools
```









