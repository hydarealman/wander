---
title: "Prompt Engineering 提示工程"
slug: "prompt-engineering-提示工程"
date: 1970-01-21T23:10:23+08:00
draft: false
source_file: "feishu://prompt-engineering-提示工程"
source_size: 4731
source_lines: 186
tags:
  - "AI"
categories: []
---

# Prompt Engineering 提示工程
# 1.什么是提示词工程

当前是AGI时代
AGI Artificial General Intelligence 通用人工智能

## 我们在提示工程上的优势
我们懂原理,所以知道
为什么有的指令有效,有的指令无效
为什么同样的指令有时有效,有时无效
怎么提升指令的有效的概率

我们懂编程:
知道哪些问题用提示词工程解决更高效,哪些用传统编程
能完成和业务系统的对接,把效能发挥到极致


## 使用Prompt的两种目的
1.获得具体问题的具体结果
2.固化一套Prompt到程序,成为系统功能的一部分

后者更难,掌握后能轻松搞定前者
后者是我们的独特优势

## Prompt调优
找到好的prompt是个持续迭代的过程,需要不断调优

高质量prompt核心要点: 
具体,丰富,少歧义



image.png






# 2.Prompt的典型构成
### 模板建议1 -> 来自吴恩达:
角色: 
给AI定义一个最匹配任务的角色, 比如: 你是一位软件工程师 你是一位小学老师

image.png

证明: 
image.png

放在开头的影响最大
放在结尾的影响也比较大
放在中间的影响最小

大模型对prompt开头和结尾的内容更敏感

先定义角色,其实就是在开头把问题收窄,减少二义性

指示: 
对任务进行描述 

上下文: 
给出与任务相关的其他背景信息(尤其在多轮交互中)

例子:
必要时给出举例,学术中称为 one shot learning , few-shot learnig context learning; 实践证明对输出正确性有很大帮助

输入: 
任务的输入信息;在提示词中明确地标识出输入

输出: 
输出的格式描述,以便后续模块自动解析模型的输出



### 模板建议2 -> 来自字节:
浅层的提示词工程
1.明确目标: 首先确定你希望大模型或者机器人为你做什么是写一个营销方案还是智能回答
2.优化提示: 我们可以给大模型更加具体的提示,让大模型知道自己是干啥的
3.评估并迭代: 通过不同的提示词来问同样的问题,看大模型是如何反馈的,如果不满意的话可以修改提示词,然后再次尝试,不要怕麻烦,直到它可以反馈出让我们满意的答案或者反馈出更适合应用场景的答案


深层的提示词工程 -> 属于开发层面



提示词的两个核心技术: 
N-gram
通过统计计算N个词共同出现的概率来预测下一个词

深度学习
深度学习模型是由多层神经网络组成的，可以自动从数据中去学习这些特征
让模型不断地自我学习不断进步不断成长



# 3.如何编写提示词



gemini自己生成的自己的Prompt模板: 

1. 当前 Lab 与任务： [例如：Lab 3 Page tables，任务 2：A kernel page table per process]
2. 我的目标： [例如：我想在 allocproc() 中为每个进程分配一个独立的内核页表，并拷贝全局内核页表的内容。]
3. 遇到的问题 (Expected vs. Actual)： [例如：编译通过了，但在运行 make qemu 时，系统在启动 init 进程时发生了 Panic。]
4. 报错日志 / 终端输出：
Plaintext
[粘贴你的 QEMU panic 信息、usertrap/kerneltrap 报错、或者 make grade 的失败提示。包含 scause, sepc, stval 等寄存器信息非常关键！]
5. 相关的代码片段：
C
// [在这里贴上你修改过的代码，最好带上函数名和一点上下文]struct trapframe *trapframe = p->trapframe;
// ...你的代码...
6. 我的思考与尝试 (非常重要)： [例如：我怀疑是因为在 scheduler() 切换页表时，satp 寄存器没有正确刷新，但我加了 sfence.vma 还是不行。]
7. 我的诉求： [例如：请帮我指出代码逻辑的漏洞 / 请给我一个 debug 的思路 / 请解释一下 walk() 函数的第三个参数是什么意思。]










# 4.进阶技巧
## 思维链
image.png




## 自洽性 (Self-Consistency)
image.png





## 思维树(Tree-of-thought,ToT)
image.png















image.png




image.png


核心思路: 
把输入的自然语言对话,转成结构化的表示
从结构化的表示,生成策略
把策略转化成自然语言输出











![图片 1](/images/feishu/prompt-engineering-提示工程/image_bdebef.png)
![图片 2](/images/feishu/prompt-engineering-提示工程/image_f5576c.png)
![图片 3](/images/feishu/prompt-engineering-提示工程/image_4b3b55.png)
![图片 4](/images/feishu/prompt-engineering-提示工程/image_aac566.png)
![图片 5](/images/feishu/prompt-engineering-提示工程/image_29d992.png)
![图片 6](/images/feishu/prompt-engineering-提示工程/image_1ba50e.png)
![图片 7](/images/feishu/prompt-engineering-提示工程/image_72f362.png)
![图片 8](/images/feishu/prompt-engineering-提示工程/image_36d313.png)
