---
title: "claude-code"
slug: "claude-code"
date: 2026-06-28T13:24:17+08:00
draft: false
source_file: "feishu://claude-code"
source_size: 1179
source_lines: 66
tags: []
categories: []
---

Claude Code
CLI:
cli是一种通过输入文本指令来与电脑操作系统或软件进行交互的方式及

斜杠命令: 
核心命令
/help
显示所有可用命令、快捷键和帮助信息，是入门和遗忘时的好帮手。

/init
在项目根目录生成 CLAUDE.md 文件。Claude 会在每次会话中读取此文件，用于存储项目规范、技术栈等持久化信息


/clear
硬重置,清空所有对话历史,开始一个全新的会话

/model
在会话中切换模型

开发辅助:
/plan
进入计划模式,Claude会先给出执行方案,经你确认后再动手,适合复杂任务

/btw
在主任务进行时,并行提出一个不相关的问题,不打断当前流程

/rewind
回退到之前的某个节点,可以同时回退代码和对话状态

/simplify
对代码进行三重审查,寻找可以简化的地方


监控与配置
/cost
查看当前会话已经消耗的Token费用


/context
查看当前已加载的上下文信息


/memory
直接编译CLAUDE.md文件


/doctor
诊断Claude Code的安装和配置问题

/config
打开全局配置

/login / logout
进行会话的身份验证或断开连接


键盘快捷键






Agent skills

