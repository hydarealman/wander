---
title: "ROS1"
slug: "ros1"
date: 1970-01-21T23:10:23+08:00
draft: false
source_file: "feishu://ros1"
source_size: 26450
source_lines: 789
tags:
  - "ROS"
categories:
  - "机器人视觉"
---

# ROS1

学习网址
https://bluesnie.github.io/Learning-notes/ROS2/%E6%9C%BA%E5%99%A8%E4%BA%BA%E5%AD%A6%E7%AF%87/%E7%AC%AC7%E7%AB%A0-ROS2%E8%BF%90%E5%8A%A8%E5%AD%A6/001-TF2%E4%BB%8B%E7%BB%8D%E5%8F%8ARVIZ-TF%E7%BB%84%E4%BB%B6.html
## 创建工作空间功能包流程
遇到的bug: 
最开始改CMakeLists.txt文件时改错文件了,而且居然改的时候有使用超级用户权限,应该改的文件是功能包的CMakeLists.txt文件，而不是工作空间的CMakeLists.txt文件,这个文件是自动生成的,后来意识到这个问题,但是改的时候一直没有意识到权限不够,我以为我改了实际上我没有更改成功,最后知道新开了一个工作空间重新编译才发现这个报了一摸一样的错误,才发现,之前一直以为是路径错误.

1.创建工作空间目录结构

mkdir -p ~/catkin_ws/src
2.初始化工作空间

cd ~/catkin_ws/src
catkin_init_workspace
3.编译工作空间

cd ~/catkin_ws
catkin_make
4.创建功能包
（1.）进入src目录: 

cd ~/catkin_ws/src
(2.) 创建功能包

catkin_create_pkg my_first_pkg roscpp rospy std_msgs

5.编写C++节点代码

cd ~/catkin_ws/src/my_first_pkg
touch src/hello_ros.cpp
6.示例代码

#include "ros/ros.h" // ROS 核心头文件

int main(int argc, char **argv) {
    // 初始化 ROS 节点，节点名必须唯一
    ros::init(argc, argv, "hello_ros_node");
    // 创建节点句柄，用于管理节点资源
    ros::NodeHandle nh;
    // 在控制台输出信息
    ROS_INFO("Hello, ROS World!");
    return 0;
}
7.配置编译规则
1.找到add_executable部分,添加你的可执行文件生成规则

add_executable(hello_ros_node src/hello_ros.cpp)
hello_ros_node: 可执行文件的目标名称(在CMake内部使用) -> 不是源文件名(这个名字是自己定义的)
src/hello_ros.cpp: 源文件路径(相对于CMakeLists.txt)

2.找到target_link_libraries部分,为你的可执行文件链接必要的catkin库: 

target_link_libraries(hello_ros_node ${catkin_LIBRARIES})
hello_ros_node: 之前add_executable定义的目标名称
&{catkin_LIBRARIES}: CMake变量,包含所有需要的ROS库

8.编译并运行节点
(1)回到工作空间根目录重新编译

cd ~/catkin_ws
catkin_make
(2)在运行节点前,必须启动ROS核心,打开一个新的终端标签页或窗口

roscore
(3)配置当前终端的环境变量,使其能够找到你工作空间编译好的功能包和节点

source ~/catkin_ws/devel/setup.bash
(4)运行节点

rosrun my_first_pkg hello_ros_node

9.补充,设置环境变量(持久化)

echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc  # 使更改立即生效

2.初始化
创建功能包: 
1.进入工作空间的src目录: 

cd ~/catkin_ws/src
2.使用 catkin_create_pkg 命令创建功能包

catkin_create_pkg <your_package_name> [depend1] [depend2] [depend3] ...
3.整理功能包结构:

cd my_robot
mkdir scripts src launch msg srv
4.回到工作空间根目录编译功能包

cd ~/catkin_ws
catkin_make
刷新当前终端的环境变量

source devel/setup.bash
5.验证功能包创建成功
可以使用 rospack 命令来查找或查看你的包信息：

rospack find my_robot  # 查找功能包路径
roscd my_robot         # 切换到功能包目录
rosls my_robot         # 列出功能包内容

## ros::spin()和ros::spinOnce()的区别
ros::spin() 
阻塞行为: 阻塞,调用后不返回,持续处理回调
循环机制: 内部自带无限循环,直到节点关闭
使用场景: 节点仅需要处理回调函数,无其他周期性任务
使用场景: 节点仅需要处理回调函数,无其他周期性任务
控制灵活性: 低,无法在循环内添加其他任务
后续代码: 其后的代码不会被执行(除非节点关闭)

ros::spinceOnce()
阻塞行为：非阻塞,调用后立即返回,只处理一次当前回调队列
循环机制: 需外部循环(while(ros::ok()))配合
使用场景: 节点需同时处理回调函数和其他周期性任务
控制灵活性: 高,可在调用前后执行自定义代码
后续代码: 其后的代码会继续执行


## roscore的作用
roscore是ROS系统的总指挥部和信息中心,它提供了节点之间通信所必须的基础设施
ROS Master: 
作用: 管理所有节点的注册,发现和连接,充当节点通信的"名称服务"和"协调员"
Parameter Server:
一个全局的键值存储服务器,用于节点间共享配置参数和初始设置
rosout节点: 
收集所有节点的日志输出,并提供统一的日志记录和查看接口

roscore为ROS的分布式计算提供了核心的通信基础设施.在ROS1中,任何节点在启动时都需要向ROS Master注册,并通过它来发现其他节点,从而建立点对点的直接通信,没有roscore,节点就像失去了通讯录和电话号码,节点就像失去了通讯录和电话号码的员工，无法找到彼此并进行协作

## ros::spinceOnce()和ros::spin()的区别

ros::spin()
阻塞式: 进入一个无限循环,持续处理ROS回调
不会返回: 一旦调用,程序会已知停留在这个函数中,知道节点被关闭
适用于: 只需要处理回调而不需要执行其他任务的简单节点

ros::spinceOnce()
非阻塞式: 处理当前时刻所有挂起的回调,然后立即返回
继续执行：调用后程序会继续执行后面的代码
适用于: 需要在循环中同时处理回到和执行其他任务的节点

## ROS的并发模型
ros::spin()不会阻塞其他节点: 
每个节点是独立进程： ROS节点是独立的进程,每个节点有自己的执行线程
ros的通信是异步的: 节点间的消息传递通过ROS Master和话题/服务实现,不依赖对方节点的执行装填

ros::spin()只阻塞当前节点: 它只影响调用它的节点,不会影响系统中其他节点的运行
## ROS工具类:
### 1.ros::Rate
ros::Rate是ROS中一个非常重要的实用工具类,用于控制循环(尤其是主循环)的执行频率(速率). 它的目的是让循环以尽可能接近的固定频率运行

没有速率控制: 
1.循环会尽可能快的运行: 这可能导致CPU占用率过高,浪费计算资源
2.执行时间不稳定: 每次循环执行的任务耗时可能不同,导致循环的实际时间波动很大
3.难以与其他节点同步: 如果你的节点需要以特定频率发布数据或执行控制,没有速率控制就无法保证这个频率


ros::Rate loop_rate(10); // 期望以10Hz(每秒10次)运行

while(ros::ok()) {
loop_rate.sleep(); // 关键! 在这里睡眠以控制速率

### 2.时间相关
ros::Time： 表示一个时间点，它由秒和纳秒两部分组成,通常用于时间戳

ros::Duration: 表示一个时间段或事件间隔, 它也由秒和纳秒组成

ros::Timer: 用于创建周期性的定时回调，类似于ros::Rate但更面向事件，你不需要自己写while循环,只需设置一个回调函数和周期,ROS就会定期调用它
与ros:: Rate的区别: Timer在后台线程中触发回调,不会阻塞你的主线程.而ros::Rate通常用于阻塞主循环以控制其频率 

void timerCallback(const ros::TimerEvent& event) {
  // 这个函数会被定期调用
  ROS_INFO("Timer called. Expected period: %.4f, Actual last period: %.4f",
           event.current_expected.toSec(),
           event.last_duration.toSec());
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "timer_node");
  ros::NodeHandle nh;

  // 创建一个定时器，每 1.0 秒调用一次 timerCallback 函数
  ros::Timer timer = nh.createTimer(ros::Duration(1.0), timerCallback);

  // 进入自旋，等待回调
  ros::spin();
  return 0;
}

### 3.tf
tf2_ros::TransformListener       用于侦听和缓存由 TransformBroadcaster发布的坐标变换(tf)数据

tf2_ros::TransformBroadcaster  用于发布坐标变换关系

tf2::doTransform()和tfBuffer.transform() 用于执行实际的坐标变换,将一个点或姿态从一个坐标系转换到另一个坐标系

### 4.日志与诊断
ROS_INFO(),ROS_WARN(),ROS_ERROR(),ROS_FATAL()（通常用于导致节点无法继续运行的致命错误）
这些是宏,而不是类,但极其常用,它们提供了不同级别的日志输出功能，替代了std::cout
输出会带有时间戳,节点名,消息级别等信息,非常利于调试

### 5.节点句柄与参数服务
ros::NodeHandle
这是你与ROS系统交互的主要入口,几乎所有操作(创建发布者/订阅者,获取/设置参数,创建定时器等)都需要通过它
它提供了访问参数服务器的方法

### 6.多线程处理(Spinners)
用于处理回调函数,特别是在有多个订阅或服务时
ros::MultiThreadedSpinner spinner(4);
使用一个线程池来处理回调,可以并行处理多个回调。如果一个回调函数执行时间很长,它不会阻塞其他回调的执行

ros::AsyncSpinner
类似于MultiThreadedSpinner ，但更灵活,可以随时启动和通知

ros::NodeHandle nh;
ros::NodeHandle private_nh("~"); // 私有节点句柄，用于访问私有参数

std::string default_name = "robot";
// 从参数服务器获取参数，如果不存在则使用默认值
nh.param<std::string>("robot_name", robot_name, default_name);
// 设置参数
nh.setParam("control_frequency", 30.0);

## 录制与回放数据步骤

1.启动节点并列出话题


//1.启动核心
roscore
//2.启动各种节点
...
//3.source工作空间
source catkin_ws/devel/setup.bash
//4.列出话题
rostopic list -v
//或者 rostopic list
2.创建文件并开始录制


//1.创建文件
mkdir bagfiles
//2.进入文件路径
cd bagfiles
//3.录制当前发布的所有话题数据
rosbag record -a
3.检查并回放bag文件


//在bag包所在目录下执行命令
//查看bag包
rosbag info <your bagfile>
//回放bag文件以再现系统运行过程
rosbag play <your bagfile>
4.录制数据子集


rosbag record -o subset /turtle/command_velocity /turtlel/pose

5.录制指定话题 

rosbag record <话题1> <话题2> ... <话题N>

# 录制单个话题
rosbag record /turtle1/cmd_vel

# 录制多个话题
rosbag record /turtle1/cmd_vel /turtle1/pose /odom

# 录制不同类型的话题
rosbag record /scan /tf /camera/rgb/image_raw


rosbag的局限性
在前述部分中你可能已经注意到了turtle的路径可能并没有完全映射到原先通过键盘控制时产生的路径----整体形状应该是差不多的,但没有完全一样,造成该问题的原因是turtlesim的移动路径对系统定时精度的变化非常敏感。rosbag受制于其本身的性能无法完全复制录制时的系统运行行为,rosplay也一样,对于turtlesim这样的节点,当 处理消息的过程中系统定时发生极小变化时也会使其行为发生微妙变化,用户不应该期望能够完美的模仿系统行为



## 超级终端
安装: 

sudo apt install terminator

快捷键
0ad8c37504e42b9e10a9eab9b806f3b.jpg


快捷键冲突bug解决
ibus-setup
把占用的快捷键给去除掉


## ROSTF广播
一,核心概念：
1.坐标系(Frame):
.以字符串
.构成树状结构
2.变换:
.包含平移(x,y,z) 和旋转(四元数qx,qy,qz,qw)
.描述子坐标系相对于父坐标系的位置姿态

二,广播变换:
1.静态变换广播

#include <tf2_ros/static_transform_broadcaster.h>

geometry_msgs::TransformStamped static_transform;
static_transform.header.stamp = ros::Time::now();
static_transform.header.frame_id = "parent_frame";
static_transform.child_frame_id = "child_frame";
static_transform.transform.translation.x = 0.5;
static_transform.transform.translation.y = 0.0;
static_transform.transform.translation.z = 0.2;
static_transform.transform.rotation = tf::createQuaternionMsgFromYaw(M_PI/4); // 45度

tf2_ros::StaticTransformBroadcaster broadcaster;
broadcaster.sendTransform(static_transform);
动态变换广播

#include <tf2_ros/transform_broadcaster.h>

tf2_ros::TransformBroadcaster broadcaster;
geometry_msgs::TransformStamped transform;

void publishTransform() {
  transform.header.stamp = ros::Time::now();
  transform.header.frame_id = "odom";
  transform.child_frame_id = "base_link";
  transform.transform.translation.x = x_position;
  transform.transform.rotation =          tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
  broadcaster.sendTransform(transform);
}

三.监听变换
1.查询最新变换

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped>

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

geometry_msgs::TransformStamped transform;
try {
  transform = tfBuffer.lookupTransform("target_frame", "source_frame", ros::Time(0)); // 获取最新可用变换
  double x = transform.transform.translation.x;
  // 使用变换数据...
} catch (tf2::TransformException &ex) {
  ROS_ERROR("%s", ex.what());
}
2.指定时间戳查询

// 查询特定时刻的变换（需保证时间戳在tf树范围内）
transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(ros::Time::now() - ros::Duration(1.0)));

3.等待可用变换

// 阻塞等待直到变换可用（超时2秒）
tfBuffer.canTransform("map", "base_link", ros::Time::now(), ros::Duration(2.0));

四,常用工具函数
1.欧拉角 <-> 四元数转换

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// 欧拉角转四元数
tf2::Quaternion quat;
quat.setRPY(roll, pitch, yaw);
geometry_msgs::Quaternion quat_msg = tf2::toMsg(quat);

// 四元数转欧拉角
tf2::fromMsg(quat_msg, quat);
tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
2.坐标点变换

geometry_msgs::PointStamped point_in, point_out;
point_in.header.frame_id = "camera";
point_in.point.x = 1.0;

tfBuffer.transform(point_in, point_out, "map"); // 将点从camera系转换到map系

五,调试命令
1.查看坐标系树

rosrun tf2_tools view_frames.py  # 生成frames.pdf
2.检查特定变换

rosrun tf tf_echo source_frame target_frame
3.可视化坐标系(RViz)

六,常见问题
1.时间戳不匹配
错误: Lookup would require extrapolation into the past
解决: 确保广播时间戳 >= 监听器查询的时间
2.坐标系未连接
错误: Could not find a connection between 'map' and 'base_link'
3.使用tf2替代旧版tf


## TF时间戳
一，时间戳的核心作用:
1.时空一致性
每个变换,必须携带时间戳,表示该位姿数据的有效时刻
2.避免时间外推错误

二，时间戳的四种使用场景
查询最新变换 (零延迟)

// 优先使用：获取最新发布的变换（即使时间戳稍旧）
transform = tfBuffer.lookupTransform("target", "source", ros::Time(0));

指定历史时刻变换

// 查询特定时刻的位姿（如匹配传感器数据时间戳）
ros::Time sensor_stamp = scan_msg->header.stamp;
transform = tfBuffer.lookupTransform("map", "base_link", sensor_stamp);

3.时间旅行查询

// 组合：查询从source_time到target_time的完整变换链
transform = tfBuffer.lookupTransform("target_frame", ros::Time::now(),
                                     "source_frame", sensor_stamp,
                                     "fixed_frame"); // 固定参考系

4.等待未来变换

// 阻塞等待直到指定变换可用（超时5秒）
if (tfBuffer.canTransform("map", "base_link", ros::Time::now(), ros::Duration(5.0))) {
  transform = tfBuffer.lookupTransform("map", "base_link", ros::Time::now());
}






## 清理ROS日志
1.检查当前日志大小:

rosclean check

2.清理所有ROS日志:

rosclean purge
3.执行后系统会提醒你确认是否删除,输入y并按回车即可


4.ros日志文件的作用: 
记录节点运行状态
辅助调试俄故障排除

可以使用less,cat,tail-f 或grep 等linux命令直接查看或过滤日志文件

查看实时日志流:

rostopic echo /rosout

检查ROS日志配置
rosrun rqt_console rqt_console

## ROS常用组件
演示小乌龟
roslaunch turtle_tf2 turtle_tf2_demo_cpp.launch
或者(上面的是cpp节点写的,下面的是python节点写的)
roslaunch turtle_tf2 turtle_tf2_demo.launch
### TF坐标变换(TransForm Frame)
### 概述
TF坐标变换: 实现不同类别的坐标系之间的转换
(因为不可以将物体相对于该传感器的方位信息,等价于机器人系统或机器人其他组件的方位信息) -> 所以需要坐标系之间的变换

概念: 
tf: TransForm Frame
坐标系: ROS中是通过坐标系开标定物体的,确切的将是通过右手坐标系来标定的
作用: 
在ROS中用于实现不同坐标系之间的点或向量的转换

说明: 
在ROS中坐标变换最初对应的是tf,不过在hyfro版本开始,tf被废弃,迁移到tf2,后者更为简洁高效,tf2对应的常用功能包有: 
tf2_geometry_msgs 可以将ROS消息转换为tf2消息
tf2: 封装了坐标系变换常用消息    四元数 <-> 欧拉角
tf2_ros: 为tf2提供了rscpp和rospy绑定,封装了坐标变换常用的API

### 坐标msg消息
在坐标转换实现中常用的msg:
geometry_msgs/TransformStamped 和 geometry_msgs/PointStamped
前者用于传输坐标系相关位置信息,后者用于传输某个坐标系内坐标点的信息,在坐标变换中,频繁的需要使用坐标系的相对位置以及坐标点的信息 

1.geometry_msgs/TransformStamped 
命令行键入: rosmsg info geometry_msgs/TransformStamped

std_msgs/Header header                 # 头信息
    uint32 seq                         #|-- 序列号
    time stamp                         #|-- 时间戳
    string frame_id                    #|-- 坐标 ID
string child_frame_id                  #子坐标系的id
geometry_msgs/Transform transform      #坐标信息
  geometry_msgs/Vector3 translation    #偏移量
    float64 x                          #|--x 方向的偏移量
    float64 y                          #|--y 方向的偏移量
    float64 z                          #|--z 方向的偏移量
  geometry_msgs/Quaternion rotation    # 四元数
    float64 x
    float64 y
    float64 z
    float64 w
四元数用于表示坐标的相对姿态
2.geometry_msgs/PointStamped
命令行键入: rosmsg info geometry_msgs/PointStamped

std_msgs/Header header                # 头信息
  uint32 seq                          #| -- 序列号
  time stamp                          #| -- 时间戳
  string frame_id                     #| -- 所属坐标系的id
geometry_msgs/Point point             # 点坐标 (x,y,z坐标)
  float64 x                          
  float64 y                       
  float64 z

### 静态坐标变换
指两个坐标系之间的相对位置是固定的
实现分析: 
1.坐标系相对关系,可以通过发布方发布
2.订阅方,订阅到发布的坐标系相对关系,再传入坐标点信息(可以写死),然后借助于tf实现坐标变换,并将结果输出

实现流程: 
1.新建功能包,添加依赖
(创建项目功能包依赖于 tf2 tf2_ros tf2_geometry_msgs roscpp rospy std_msgs geometry_msgs)
2.编写发布方实现
3.编写订阅方实现
4.执行并查看结果

发布方: 

/*
静态坐标变换发布方: 
    发布关于laser 坐标系的位置信息
    
实现流程: 
     1.包含头文件
     2.初始化ROS节点
     3.创建静态坐标转换广播器
     4.创建坐标信息
     5.广播器发布坐标信息
     6.spin()
*/

#include "ros/ros.h" // ros核心功能
#include "tf2_ros/static_transform_broadcaster.h" // 静态变换广播器
#include "geometry_msgs/TransformStamped.h" // 变换消息结构
#include "tf2/LinearMath/Quaternion.h"// 四元数操作

int main(int argc, char *argv[]) {
    setlocale(LC_ALL,""); // 支持中文字符
    ros::init(argc,argv,"static_pub"); // 初始化节点,名为"static_pub"
    ros::NodeHandle nh; // 创建节点句柄

    tf2_ros::StaticTransformBroadcaster pub; // 创建静态变换广播器
 
    // 设置变换消息
    geometry_msgs::TransformStamped tfs;
    tfs.header.stamp = ros::Time::now();  // 当前时间戳
    tfs.header.frame_id = "base_link"; // 父坐标系
    tfs.child_frame_id = "laser"; // 子坐标系
    tfs.transform.translation.x = 0.2; // x轴偏移0.2米
    tfs.transform.translation.y = 0.0; // y轴无偏移
    tfs.transform.translation.z = 0.5; // z轴偏移0.5米

    // 设置旋转
    tf2::Quaternion qtn;
    qtn.setRPY(0,0,0); 

    tfs.transform.rotation.x = qtn.getX();
    tfs.transform.rotation.y = qtn.getY();
    tfs.transform.rotation.z = qtn.getZ();
    tfs.transform.rotation.w = qtn.getW();

    pub.sendTransform(tfs); // 发布静态变换

    ros::spin(); // 保持节点运行

    return 0;
}

订阅方

#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/PointStamped.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

int main(int argc, char* argv[]) {

    setlocale(LC_ALL,"");
    ros::init(argc,argv,"static_sub");
    ros::NodeHandle nh;

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);
    geometry_msgs::PointStamped ps;
    ps.header.frame_id = "laser";
    ps.header.stamp = ros::Time::now();
    ps.point.x = 2.0;
    ps.point.y = 3.0;
    ps.point.z = 5.0;

    ros::Duration(2).sleep(); // 方案1: 在调用转换函数前,执行休眠
    ros::Rate rate(10);
    while (ros::ok) {

        geometry_msgs::PointStamped ps_out;
        /*
            调用了该 buffer 的转换函数 transform
            参数1: 被转换的坐标点
            参数2: 目标坐标系
            返回值；输出的坐标点
        
            PS1: 调用时必须包含头文件 tf2_geometry_msgs/tf2_geometry_msgs.h
            PS2: 运行时存在的问题: 抛出异常 base_link 不存在
                原因: 订阅数据是一个耗时操作,可能再调用 transform 转换函数,坐标系的
                    相对关系还没有订阅到,因此出现异常
                解决: 
                    方案1: 在调用转换函数前,执行休眠
                    方案2: 进行异常处理(放一个异常捕获)，捕获后随便做一些处理再让他进入循环 -> 直                     到不抛出异常了就正常完成操作
        */
        try // 方案2: 执行异常捕获
        {
            ps_out = buffer.transform(ps,"base_link");


            ROS_INFO("转换后的坐标值:(%.2f,%.2f,%.2f),参考的坐标系:%s",
                        ps_out.point.x,
                        ps_out.point.y,
                        ps_out.point.z,
                        ps_out.header.frame_id.c_str()
                        );
        }
        catch (const std::exception& e)
        {
            // std::cerr << e.what(ps.what() << '\n');
            ROS_INFO("异常消息: %s",e.what());
        }
        
        rate.sleep(); 
        ros::spinOnce();
    }

    return 0;
}

package.xml - 功能包清单文件
作用: 
1.定义功能包名称,版本,描述等元数据
2.声明依赖关系
3.指定维护者信息和许可证
创建方式: catkin_create_pkg命令时会自动生成模板 ， 
开发者需要手动编辑填充具体内容， 
位于功能包的根目录

CMakeLists.txt - 构建系统文件(功能包目录下)
作用: 
定义如何编译源代码
指定可执行文件和库的生成规则

创建功能包模板命令行
创建一个新功能包
catkin_create_pkg tf01_static roscpp tf2 tf2_ros

运行节点命令行: 

rosrun tf01_static demo01_static_pub
rosrun: ROS的核心命令,用于运行节点
tf01_static: 功能包名
demo01_static_pub: 节点名
定义位置: 在代码中通过ros::init()函数设置
注意: 实际运行的是编译后的可执行文件名,与代码中的节点名可以不同

核心概念区分: 
可执行文件名 - 操作系统层面
定义在CMakeLists.txt中
是磁盘的二进制文件名
通过rosrun <包名> <可执行文件名> 执行
节点名 - ROS系统层面: 
定义在代码中ros::init()函数
是ROS图中的标识符
通过rosnode list查看
这里面还有很深的门道:    
略


补充1: 
当坐标系之间的相对位置固定时,那么所需参数也是固定的,父坐标系名称,子坐标系名称,x偏移量,y偏移量,x翻滚角度,y俯仰角度,z偏航角度,实现逻辑相同,参数不同,ROS系统已经封装好了专门的节点: 

命令行
// rosrun tf2_ros static_transform_publisher x偏移量 y偏移量 z偏移量 z偏航角度 y俯仰角度 x翻滚角度 父级坐标系 子级坐标系

rosrun tf2_ros static_trasform_publisher 0.2 0 05 0 0 0 /baselink/laser

补充2: 
可以借助于rviz显示坐标系关系,具体操作
新建窗口输入命令: rviz
在启动的rviz中设置Fixed Frame 为base_link
点击左下的add按钮,在弹出的窗口中选择TF组件,即可显示坐标关系


配置安装规则: 
安装后: 
编辑: package.xml

<name>tf01_static</name>
<version>0.0.0</version>
<description>Static TF broadcaster package</description>
<maintainer email="you@example.com">Your Name</maintainer>
<license>BSD</license>
编辑: 

<name>tf01_static</name>
<version>0.0.0</version>
<description>Static TF broadcaster package</description>
<maintainer email="you@example.com">Your Name</maintainer>
<license>BSD</license>




代码编写： 
需求: 发布两个坐标系的相对关系
流程: 
1.包含头文件;
2.设置编码 节点初始化 NodeHandle;
 3. 创建发布对象;
 4.组织被发布的消息
 5.发布数据;
 6.spin();










### 动态坐标变换
实现流程: 
1.新建功能包,添加依赖
2.创建坐标相对关系发布方(同时需要订阅乌龟位姿信息)
3.创建坐标相对关系订阅方
4.执行









                                                                                                                              C++ 实现
1.创建功能包
创建项目功能包依赖于: tf2, tf2_ros, tf2_geometry_msgs, roscpp rospy std_msgs geometry_msgs

2.发布方
                                   


## rviz

tf:
x轴: 红色 前方
y轴: 绿色 左方
z轴：蓝色 上方


查看特定话题
rqt_plot /debugpub/data
rqt_plot /debugpub1/data

rqt_plot 话题名称

同时查看多个特定话题
rqt_plot /debugpub/data/data /debugpub1/data/data                                                                                                                                                                                                                                                                                                                                                                                               

![图片 1](/images/feishu/ros1/0ad8c37504e42b9e10a9eab9b806f3b_eb7712.jpg)
