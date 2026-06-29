---
title: "Vscode命令行"
slug: "vscode命令行"
date: 1970-01-21T23:10:23+08:00
draft: false
source_file: "feishu://vscode命令行"
source_size: 6217
source_lines: 192
tags:
  - "工具"
categories:
  - "编程开发"
---

# Vscode命令行
### 单文件编译
g++ -o 输出文件名 源文件.cpp
指定输出可执行文件名为...: 要编译的源文件名
或者 g++ 源文件.cpp -o 输出文件名
将要编译的源文件: 指定输出可执行文件名
### 调试支持
g++ -g -o hello hello.cpp  # 便于后续使用GDB调试[4,9](@ref)

### 运行命令
Windows:
.\输出文件名.exe  # 例如 .\hello.exe[1,2](@ref)
Linux/macOS系统​：
./输出文件名  # 例如 ./hello[1,5](@ref)

### 编译与运行组合命令
g++ -o hello hello.cpp && .\hello.exe  # Windows
g++ -o hello hello.cpp && ./hello     # Linux/macOS[8](@ref)

### 多文件编译
同时编译多个源文件
g++ -o program main.cpp utils.cpp helper.cpp  # 将多个.cpp文件编译为单一可执行文件[8](@ref)
分布编译(适合大型项目)
g++ -c utils.cpp             # 生成 utils.o
g++ -c helper.cpp            # 生成 helper.o
g++ -o program main.cpp utils.o helper.o  # 链接所有对象文件[6](@ref)




### 调试:
#### 断点调试
单步跳过: 执行当前行代码.如果该行包含函数调用,不进入函数内部,直接得到结果并跳到下一行
F10
单步进入: 执行当前行代码,如果该行包含函数调用,会进入该函数内部,并暂停在函数的第一行
F11
单步跳出: 立即执行完当前函数体内剩余的所有代码,并跳出到该函数的下一行语句暂停处
Shift + F11

这两步一般环境配置好过后直接略过就好了

方式一: 使用vscode进行图形化调试
1.安装插件​：确保已安装 C/C++ 和 ROS (可选，但方便) 插件
2.配置调试环境 (launch.json)​：
在VSCode中打开你的ROS工作空间。
切换到“运行和调试”视图，点击“创建一个launch.json文件”。
选择 C++ (GDB/LLDB)。
替换或修改配置为如下示例（​重点修改 program路径​）：
3.设置断点
4.启动调试：
确保roscore已运行: 在一个终端中手动执行roscore
在vscode中选择你刚配置好的调试配置,按F5启动

方式二: 使用GDB命令行进行调试(更灵活底层)
1.直接启动调试
直接通过gdb启动节点
gdb --args /path/to/your/ros/node
2.附加到已运行的进程(非常适合调试已崩溃或卡死的节点)

// 首先,找到你要调试的节点的进程ID(PID)
ps aux | grep your_node_name
// 然后,使用gdb附加到该进程
sudo gdb -p <PID>[5](@ref) # 可能需要sudo权限
# 附加后,程序会暂停,你可以设置断点然后输入continue让程序继续
确保程序编译时包含调试信息
3.在.launch文件,在<node>标签中添加launch-prefix属性,这样每次通过roslaunch启动都会自动进入调试模式

# 在CMakeLists.txt中确保有
set(CMAKE_BUILD_TYPE Debug)
# 或者
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")

# 重新编译
cd /home/rm/ws_glut_vison
catkin_make -DCMAKE_BUILD_TYPE=Debug

三: 常用调试命令(GDB)
命令1.: break <location>  
简写: b 
用途: 设置断点(如b main.cpp: 25 myFunction) 
VSCode中的等效操作:  在行号前点击
示例： 

# 启动gdb后，设置关键断点
(gdb) break OutpostObserver::update
(gdb) break OutpostObserver::predict
(gdb) break OutpostObserver::getPredictiveMeasurement
(gdb) break OutpostObserver::getMeasurementPD

# 运行程序
(gdb) run

命令2: run 
简写: r  
用途: 启动或重新启动程序 
VSCode中的等效操作： F5继续

命令3: next
简写: n
用途: 执行下一行(不进入函数内部) 
VSCode中的等效操作：F10

命令4: step
简写: s
用途: 执行下一行(进入函数内部) 
VSCode中的等效操作: F11

命令5: print <variable>
简写: p
用途: 打印变量的值
VScode: 在调试窗口查看或悬停

命令6: backtrace
简写: bt
用途: 显示当前的函数调用栈
VSCode: 查看"调用堆栈"窗口
示例: 

# 查看崩溃时的调用栈
(gdb) bt
(gdb) bt full  # 显示完整栈帧和局部变量

# 查看具体的崩溃位置
(gdb) frame 0  # 查看最顶层的帧

# 打印相关变量的值
(gdb) print X_.rows()
(gdb) print X_.cols() 
(gdb) print P_.rows()
(gdb) print P_.cols()
(gdb) print H.rows()
(gdb) print H.cols()

命令7: info breakpoints
简写: i b 
用途: 列出所有断点
VSCode: 在”断点“窗口查看


命令8: quit
简写: q
用途: 退出GDB
VSCode: 停止调试按钮

#### 调试流程
1.配置编译任务
按ctrl+shift+p,输入"Tasks: Configure Task" , 选择"Create tasks.json file from templates",选择"Others"或"C/C++ g++ buikd active file" ,这回生成一个tasks.json,然后问ai生成一个调试文件

2.配置调试设置(launch.json)
按F5，选择(GDB/LDB),然后选择"g++ build and debug activate file"

3.开始断点调试
1）.设置断点: 在代码行号左侧单击,出现红点即为断点
2）.启动调试: 按F5
3）.调试操作
4）.查看状态

4.高级调试技巧
条件断点

# 在可能越界的矩阵访问处设置条件断点
(gdb) break OutpostObserver.cpp:123 if row >= rows() || col >= cols()
日志断点
函数断点


#### 常见调试方法
1.打印语句 (快速查看变量值,执行流程) -> 简单逻辑排查,快速确认执行到哪,变量是什么值

std::cout << "x = " << x << std::endl;
2.断言 (检查假设条件是否成立,捕捉非法状态) -> 检查函数参数有效性,数组越界,不可能出现的状态,调试版本常用

assert(ptr != nullptr)
3.断点 暂停程序执行,检查现场 -> 任何需要详细检查上下文,调用栈,不可能的状态等,调试版本常用

在 IDE 中点击行号左侧；GDB: break mainbreak filename:lineno
4.单步调试: 精细控制执行过程,观察每一步变化 -> 逻辑复杂,需要逐行分析;理解代码执行流程;跟踪进入函数内部

GDB: next(步过), step(步入); IDE: F10, F11
5.查看变量/内存: 检查变量值,指针指向,内存数据 -> 验证数据是否正确,排查内存错误(如越界,坏指针)


GDB: print variable, print *ptr@10; IDE: 悬停观察，监视(Watch)窗口
6.日志调试: 记录程序运行信息,追踪线上问题 -> 复杂逻辑跟踪,循环体内,无法直接调试的环境,嵌入式

7.内存检查工具 -> 检查内存泄漏,越界访问等内存错误 -> 程序运行崩溃,怀疑有内存相关错误时


### make编译


