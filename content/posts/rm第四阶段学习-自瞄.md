---
title: "rm第四阶段学习---自瞄"
slug: "rm第四阶段学习-自瞄"
date: 1970-01-21T22:38:48+08:00
draft: false
source_file: "feishu://rm第四阶段学习-自瞄"
source_size: 2091
source_lines: 66
tags:
  - "机器人"
categories:
  - "机器人视觉"
---

rm第四阶段学习---自瞄


image.png

image.png

image.png


opencv书

第八章检测兴趣点
这个概念的原理是，从图像中选取某些特征点并对 图像进行局部分析（即提取局部特征），而非观察整幅图像（即提取全局特征）。
视觉不变性 目前略

检测图像中的角点
检测角点的经典方法：Harris 特征检测
基本函数:cv::cornerHarris
image.png

THRESH_BINARY_INV用黑色表示被检测的角点
THRESH_BINARY用白色表示被检测的角点
image.png

检测Harris角点需要两个步骤：
1.计算每个像素的Harris值
2.然后用指定的阈值获得特征点
8.3 快速检测特征
这种算子专门用来快速检测兴趣点——只需对比几个像素，就可以判断它是 否为关键点。
略
8.4 尺度不变特征的检测
不仅在任何尺度下拍摄的物体都能检测到一致的关键点，而且每个被检测的特征点都对应一个尺 度因子。

8.5 多尺度FAST特征的检测
BRISK（Binary Robust Invariant Scalable Keypoints，二元稳健恒定可扩展关键点）检测法，它 基于上一节介绍的FAST特征检测法。本节还将讨论另一种检测方法ORB（

第九章描述和匹配兴趣点
鲁棒性（Robustness）是指算法在面对各种变化和干扰时仍能保持稳定性能的能力。这些变化可能包括光照条件的变化、视角的变化、噪声的干扰、目标的遮挡等

局部模板匹配
matchTemplate()
描述并匹配局部强度值模式

用二值描述子匹配关键点

第十章估算图像之间的投影关系

计算图像之间的基础矩阵

用RANSAC（随机抽样一致性）算法匹配图像






基地任务
基地第一次网课

![图片 1](/images/feishu/rm第四阶段学习-自瞄/image_8cac57.png)
![图片 2](/images/feishu/rm第四阶段学习-自瞄/image_a888f2.png)
![图片 3](/images/feishu/rm第四阶段学习-自瞄/image_104840.png)
![图片 4](/images/feishu/rm第四阶段学习-自瞄/image_a9437d.png)
![图片 5](/images/feishu/rm第四阶段学习-自瞄/image_26731f.png)
