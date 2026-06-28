---
title: "OpenVINO"
slug: "openvino"
date: 1970-01-21T22:38:48+08:00
draft: false
source_file: "feishu://openvino"
source_size: 1817
source_lines: 50
tags:
  - "AI"
categories: []
---

OpenVINO
OpenVINO 是做什么的？
OpenVINO 是英特尔（Intel）开源的一套工具包，专门用于加速 AI 模型的推理（Inference）。
它的核心职责： 它不负责训练模型（那是 PyTorch 或 TensorFlow 的工作），它只负责使用模型。它能把你训练好的模型进行底层指令集的优化，让它在 Intel 的硬件（比如 CPU、集成显卡 iGPU、或者最新的 NPU）上跑得飞快。
为什么用 C++： 虽然 Python 开发快，但在工业落地（如自动驾驶、安防监控、医疗设备）中，通常要求极低的延迟和极高的性能，这时候 C++ + OpenVINO 就是黄金搭档。



头文件

ov::Core core; 

1.ov::Core - 初始化核心
这是程序的起点,用于管理当前设备上的所有硬件资源

// 初始化核心对象 
ov::Core core; 

2.read_model() - 读取模型
把你的 AI 模型（通常是 .xml 格式，也原生支持 .onnx 格式）加载到内存中。

// 返回的是一个指向模型图的智能指针
std::shared_ptr<ov::Model> model = core.read_model("your_model.xml");


3.compile_model() - 编译模型
将模型针对特定硬件（如 "CPU" 或 "GPU"）进行底层优化和编译。

// 将模型编译到 CPU 上
ov::CompiledModel compiled_model = core.compile_model(model, "CPU");


4.create_infer_request - 创建推理请求

ov::InferRequest infer_request = compiled_model.create_infer_request();


5.infer() - 处理数据并执行

// 1. 获取输入 Tensor (张量)
ov::Tensor input_tensor = infer_request.get_input_tensor();
// ... 在这里用你的图像数据填充 input_tensor ...

// 2. 执行推理 (同步调用)
infer_request.infer();

// 3. 获取输出结果
ov::Tensor output_tensor = infer_request.get_output_tensor();
// ... 获取 output_tensor 的数据并进行后处理 ...
