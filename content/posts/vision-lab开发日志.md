---
title: "1. 升级 pip, setuptools, wheel"
slug: "vision-lab开发日志"
date: 1970-01-21T23:10:24+08:00
draft: false
source_file: "feishu://vision-lab开发日志"
source_size: 2503
source_lines: 159
tags: []
categories: []
---

vision-lab开发日志
开发日志:
lab_huofu:
5月28日 & 5月29日 % 5月30日
完成霍夫圆识别水质圆圈的程序
使用pca 分析试剂的色值
并做线性回归


lab_water:
6月7日:
拷贝程序


6月8日:
测试水质分析仪的摄像头

6月13日:
浏览项目框架
阅读项目qt程序

好像没什么难度


lab_plant:
5月28日 ~ 6月6日：
购买材料:  
在自己电脑上测试代码

6月7日：
环境部署：
lubancat
烧录镜像 环境部署


6月8日 ~ 6月16日：
准备考试

6月17日:
继续部署环境 尝试运行代码


lab_water项目



lab_huofu项目




lab_plant项目



环境配置: 
更新系统底层软件源：

sudo apt update && sudo apt upgrade -y

将python升级到3.9及以上
安装rust编译工具

# 1. 升级 pip, setuptools, wheel
pip3 install --upgrade pip setuptools wheel

# 2. 安装 Rust 编译工具链
sudo apt update
sudo apt install cargo rustc -y


部署轻量化推理模型: 

sudo apt install libgl1-mesa-glx libglib2.0-0 -y



numpy

pip3 install numpy -i https://pypi.tuna.tsinghua.edu.cn/simple


pandas

pip3 install pandas -i https://pypi.tuna.tsinghua.edu.cn/simple

opencv-python-headless

pip3 install opencv-python-headless -i https://pypi.tuna.tsinghua.edu.cn/simple

ultralytics

pip3 install ultralytics -i https://pypi.tuna.tsinghua.edu.cn/simple
但是polars会报错: 绕过polars: 不一定使用

pip install ultralytics --no-deps && \
pip install numpy opencv-python torch torchvision matplotlib requests scipy pyyaml psutil ultralytics-thop




拷贝程序
将程序从u盘拷贝到主目录

sudo cp -r /media/usb1/lab_plant.zip /home/cat/

解压缩

unzip lab_plant.zip

删除压缩包

rm -rf lab_plant.zip




bug:
6月17日
千万不要运行

sudo ifconfig eth0 down


现在只能通过网线ssh远程连接树莓派,没办法使用显示屏和串口
由于ipv6走的也是这张网卡 所以eth0关掉后ssh也会断掉 









知识点

三维点云测量
概述
安装PCL
安装Open3D
安装CloudCompare


激光三角测距: 
线激光器向物体表面投射一条激光线,当物体表面有高低起伏时,激光线会发生弯曲和位移,相机(与激光器成固定角度)捕捉到这条变形激光线的二维图像,通过三角测量法计算出每个点亮度的中心点,最终拼接成完整的三维点云


RANSAC (Random Sample Consensus)：随机采样一致性算法
随机采样一致性算法，从包含大量离群点的数据中拟合数学模型








