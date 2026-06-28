---
title: "ROS2"
slug: "ros2"
date: 1970-01-21T22:38:48+08:00
draft: false
source_file: "feishu://ros2"
source_size: 23057
source_lines: 782
tags:
  - "ROS"
categories:
  - "机器人视觉"
---

ROS2
第一章ROS2介绍
暂无
第二章准备环境与安装ROS2
暂无
第三章动手学ROS2基础
工作空间
工作空间是一个存放项目开发类相关文件的文件夹，是开发过程的大本营

src         代码空间
Install     安装空间
build       编译空间
Log         日志空间

创建工作空间：
 mkdir -p ~/dev-ws/src




节点：机器人的工作细胞
执行具体任务的进程
独立运行的可执行文件
可使用不同的编程语言
可分布式运行在不同主机
通过节点名称进行管理
节点操作
ros2 node list   //当前正在运行的节点信息
ros2 node info  //查看该节点信息
ros2 topic       //话题
ros2 bag         //录制


ros2基本使用
1.Terminal 
ctrl+alt+t
2.当前终端所在位置
pwd
3.当前路径下所在的文件或文件夹
Ls
4.隐藏文件
ls-A
5.创建文件
Mkdir + 文件名
6.进入路径
Cd
7.创建文件
Touch
8.删除文件
Rm
9.退回上一级目录
cd...
10.递归删除（删除文件夹）
rm-R
11.安装功能包
Sudo apt install//提升使用当前用户权限为管理员权限//应用//安装


ros2功能包的相关使用
1.创建功能包

2.列出可执行文件
ros2 pkg executables
列出某个功能包
ros2 pkg executables

3.列出所有功能包
ros2 pkg list

ros2 pkg list | grep vill//过滤
列出vill开头的功能包

4.列出某个包所在的路径前缀

列出包的清单描述文件
每一个功能包都有一个标配的manifest.xml文件,用于记录这个包的名字,构建工具,编译信息,拥有者，干啥用的信息
通过这个信息，就可以自动为该功能包安装依赖，构建时确立编译顺序
ros2 pkg xml 

自动安装依赖：

编译工作空间：
Colon build    //在工作空间的根目录下进行编译
设置环境变量：


安装获取
Sudo apt install ros-<version>-package_name
功能包相关命令行ros2 pkg
Create   executables    lists    prefix    xml

ros2构建工具colcon
作用：功能包构建工具    编译代码
1.安装colcon
Sudo apt -get install python3-colcon-common-extensions
2.编译工程
Colcon build
3.运行一个自己编的节点
(1)打开一个终端使用cd colcon_test进入我们刚刚创建的工作空间,先source一下资源
source install/setup.bash
(2)运行一个订杂志节点,你将看不到任何打印功能,因为没有发布者
ros2 run examples_rclcpp_minimal_subscriber subscriber_member_function
(3)打开一个新的终端,先source,再运行一个发行杂志节点
source install/setup.bash
ros2 run examples_rclcpp_minimal_publisher publisher_member_function

colcon常用指令
1.只编译一个包
colcon test --packages-select YOUR_PKG_NAME
2.不编译测试单元
colcon test --packages-select YOUR_PKG_NAME  --cmake-args -DBUILD_TESTING=8
3.运行编译的包的测试
colcon test 
4.允许通过更改src下的部分文件来改变install(重要)
colcon build --symlink-install

第四章ROS2通信机制-话题与服务
ROS2话题介绍
1.话题的发布订阅模型(Topic通信模型)
    发布             订阅
Node1-------->话题---------->Node2
2.话题通信有哪些需要注意的规则
规则：
话题名字是关键,发布订阅接口类型要相同,发布的是字符串,接受也要用字符串来接收
同一个节点可以订阅多个话题,同时也可以发布多个话题,就像是一本书的作者也可以是另外一本书的读者
同一个话题可以有多个发布者
可以1对n，n对1，n对n
3.相关工具
3.1 RQT工具之rqt_graph
ROS2作为一个强大的工具，在运行过程中，我们是通过命令来看到节点和节点之间的数据关系的
运行第二章你说我听小demo。依次打开三个终端,分别输入下面三个命令
ros2 run demo_nodes_py listener
ros2 run demo_nodes_cpp talker
rqt_graph
3.2 ROS2话题相关命令行界面(CLI)工具
ros2 topic -h
3.2.1 ros2 topic list返回系统中当前活动的所有主题的列表
ros2 topic list
3.2.2 ros2 topic list -t 增加消息类型
3.2.3 ros2 topic echo 打印实时话题内容
ros2 topic echo /chatter
3.2.4 ros2 topic info 查看主题信息
ros2 topic info /chatter 
3.2.5 ros2 interface show 查看消息类型
ros2 interface show std_msgs/msg/String
3.2.6 ros2 topic pub arg手动发布命令
ros2 topic pub /chatter std_msgs/msg/String 'data: "123"' 
编写话题发布者:见实战案例
编写话题订阅者:见实战案例
4.接口介绍与自定义接口
4.1ROS2通信接口介绍
接口:interface
1.什么是接口
接口其实是一种规范
字符串:std_msgs/msg/String
32位二进制的整型数据:std_msgs/msg/String

使用接口的好处
不同语言对字符串的定义是不同的，接口可以抹平这种语言差异
方便程序的适配

2.ROS2接口介绍
使用ros2 interface package sensor_msgs 命令可以查看某一个接口包下所有的接口
比如传感器类的消息包:sensor_msgs
3.ROS2自定义接口
ROS2的四种通信方式
话题-Topics
服务-Services
动作-Actions
参数-Parameters
除了参数之外,话题,服务和动作(Action)都支持自定义接口,每一种通信方式所适用的场景各不相同，所定义的接口也被分为话题接口,服务接口,动作接口三种

话题接口格式：xxx.msg
int64 num//发布的话题组合

服务接口格式：xxx.srv
int64 a
int64 b//请求
---
int64 sum//发布

动作接口格式：xxx.action
int32 order//目标
---
int32[] sequence//反馈
---
int32[] partial_sequence//结果


转换过程
msg,srv,action ---------> ROS2-IDL转换器 ----------------> python的py,C++的.h头文件


4.ROS2接口常用CLI命令
4.1查看接口列表
ros2 interface list
4.2查看所有接口包
ros2 interface packages
4.3查看某一个包下的所有接口
ros2 interface package std_msgs
4.4查看某一个接口详细的内容
ros2 interface show std_msgs/msg/String
4.5输出某一个接口的所有属性
ros2 interface proto sensor_msgs/msg/Image


4.2 ROS2自定义话题接口
话题是一种单向通信的接口,同一个话题只能由发布者将数据传递给订阅者，所以定义话题接口也只需要定义发布者所要发布的类型即可.在实际的工程中为了减少功能包之间的互相依赖,通常会将接口定义在一个独立的功能包中
有了功能包之后,我们就可以新建话题接口了,新建方法如下:
新建msg文件夹,并在文件夹下新建xxx.msg（大写字母开头）
在xxx.msg下编写消息内容并保存
在CmakeList.txt添加依赖和msg文件目录
在package.xml中添加xxx.msg所需要的依赖
编译功能包即可生成python与C++头文件
2.1新建功能空间
ros2 pkg create village_interfaces --build-type ament_cmake
2.2新建msg文件夹和Novel.msg(小说类型)
cd village_interface
mkdir msg
touch Novel.msg 
2.3编写Novel.msg内容
我们的目的是给李四的小说的每一章增加一张图片,原来李四写小说是对外发布一个 std_msgs/msg/String字符串类型的数据.而发布图片的格式,我们需要采用ros自带的传感器消息接口中的图片 sensor_msgs/msg/Image 数据类型,所以我们新的消息文件的内容就是将两者合并,在ROS2中可以写做这样：

方法一
在msg文件中可以使用#号添加注释

#标准消息接口std_msgs下的String类型
std_msgs/String content
#图像消息,调用sensor_msgs下的Image类型
sensor_msgs/Image image

方法二
#直接使用ROS2原始的数据类型
String content
#图像消息,调用sensor_msgs下的Image类型
sensor_msgs/Image image

说明
通过下面的指令查看std_msgs/String是由基础数据类型string组成的
ros2 interface show std_msgs/msg/String

ROS2中的原始数据类型
bool 
byte字节类型
char字符类型
float32,float64
int8,uint16
int32,uint32
int64,uint64
string
2.4修改CMakeList.txt
完成代码编写还不够,我们还需要在CmakeLists.txt中告诉编译器,你要给我把Novel.msg转换成python库和C++库

#添加对sensor_msg的
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
#添加消息文件和依赖
rosidl_generate_interfaces(&{PROJECT_NAME}
"msg/Novel.msg"
DEPENDENCIES sensor_msgs
)

find_package用于查找rosidl_default_generators位置,下面rosidl_generate_interfaces就是声明msg文件所属的工程名字,文件位置以及依赖的DEPENDENCIES 
踩坑报告:重点强调一下依赖部分DEPENDENCIES ,我们消息中用到的依赖这里必须写上,即使不写编译器也不会报错,知道运行的时候才会报错



4.4ROS2服务介绍（序号乱了不知道为什么就这样没有改）
服务通信介绍与体验
1.启动服务端
这个命令用于运行一个服务节点，这个服务的功能是将两个数字相加,给定a,b两个数返回sum就是ab之和
ros2 run examples_rclpy_minimal_service service
2.使用命令查看列表
ros2 service list
3.3手动调用服务
再启动一个终端，输入下面的命令
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{a: 5,b: 10}"

服务相关CLI工具
1.查看服务列表
ros2 service list
2.手动调用服务
ros2 service call /add_two_ints  example_interfaces/srv/AddTwoInts "{a: 1,b: 5}"
3.查看服务接口类型
ros2 service type /add_two_inits
4.查找使用某一接口的服务
ros2 service find example_interfaces/srv/AddTwoInts

4.5自定义服务接口
话题是发布订阅模型，主要是单向传输数据,只能由发布者发布，接收者接收（同一话题，发布者接收者都可以有多个）
服务是客户服务端（请求响应）模型 由客户端发送请求，服务端处理请求，然后返回处理结果（同一服务，客户端可以有多个，服务端只能有一个）

如何创建自己的服务接口
新建srv文件夹，并在文件夹下新建xxx.srv
在xxx.srv下编写服务接口内容并保存
在CmakeLists.txt添加依赖和srv文件目录
在package.xml中添加xxx.srv所需依赖
编译功能包即可生成python与C++头文件

4.5.2python服务通信实现(李三借钱)
服务端:
1.导入服务接口
2.创建服务端回调函数
3.声明并创建服务端
4.编写回调函数逻辑处理请求

4.6.3创建客户端李三节点
1.导入服务接口
2.创建请求结果接收回调函数
3.声明并 创建客户端
4.编写结果接收逻辑
5.调用客户端发送请求

4.6话题服务对比
1.话题
1.话题是单向的,而且不需要等待服务端上线，直接发就行,数据的实时性比较高
频率高,实时性强的传感器数据的传递一般使用话题实现

2.服务
服务是双向的,客户端发送请求后,服务端有 响应,可以得知服务端的处理结果
频率较低,强调服务特性和反馈的场景一般使用服务实现







Git
安装git
Sudo apt install git
//下载别人的代码
Git clone 地址

第五章
2.参数是节点的一个配置,你可以任务参数是一个节点的设置
3.论参数组成成分
参数是由键值对组成,键值对指的就是名字和数值.

1.ros2查看节点有哪些参数(设置)
ros2 param list

2.详细查看一个参数的信息
ros2 param describe <node_name> <param_name>

3.查看参数值
例子:
ros2 param get /turtlesim background_b

4.设置参数
例子:
ros2 param set /turtlesim background_b 83

5.保存参数值
ros2 param dump /turtlesim

6.启动节点时加载参数快照
ros2 run <package_name> <executable_name> --ros-args --param-file <file_name>


ros2相关代码
函数名称                                     描述
declare_parameter                        声明和初始化一个参数
declare_parameters                       声明和初始化一堆参数
get_parameter                              通过参数名字获取一个参数
get_parameter                              通过多个参数名字获取多个参数
set_parameters                             设置一组参数的值            


编写CPP参数
声明参数
获取并设置参数                 

Action通信介绍
话题适用于节点间单向的频繁的数据传输,
服务则适用于节点间双向的数据传输,
而参数则适用于动态调整节点的设置

Action的组成部分
目标:Action客户端告诉服务端要做什么,服务端针对该目标要有响应,解决了不能确认服务端接收并处理目标的问题
反馈:Action服务端告诉客户端此时做的进度如何(类似工作汇报).解决了执行过程中没有反馈问题
结果:Action服务端最终告诉客户端执行结果,结果最后返回,用于表示任务最终执行情况

参数是由服务构建出来了,而Action是由话题和服务共同构建出来的(一个Action = 三个服务+两个话题)

三个服务分别是:
1.目标传递服务
2.结果传递服务
3.取消执行服务

两个话题:
1.反馈话题(服务发布,客户端订阅)
2.状态话题(服务端发布,客户端订阅)

Action的CLI工具

action list
获取目前系统中的action列表

action info
查看action信息

action send_goal
发送请求到服务端

ros2通信机制的总结
1.话题
话题是单向的而且不需要等待服务器上线,直接发就行,数据的实时性比较高.
频率高,实时性强的传感器数据的传递一般使用话题实现

2.服务
服务是双向的,客户端发送请求后,服务端有响应,可以得知服务端的处理结果,频率较低,强调服务特性和反馈的场景一般使用服务实现

3.参数
参数是节点的设置,用于配置节点,原理基于服务

4.动作
动作适用于实时反馈的场景，原理基于服务


第六章-ROS2工具介绍

1.launch
1.ros2节点管理之launch文件
为什么需要launch文件
1.1需要启动的节点太多
1.2节点之间有依赖关系
launch文件类似于一个脚本文件来管理节点的启动
launch文件允许我们同时启动和配置多个包含ros2节点的可执行文件

2.编写ros2的launch文件
ros2中可以使用python文件来编写launch文件
1.导入头文件
2.定义函数
3.创建节点函数
4.launch文件描述


3.测试launch文件
ros2 launch 节点(启动launch文件)

python略

cmake编译类型功能包的launch文件安装
install(DIRECTORY launch
  DESTIMATION share/${PROJECT_NAME}
  )


4.通过launch修改参数
例子:
parameters=[{"writer_timer_period" : 1}]


2.rosbag2

rosbag2介绍与安装
CLI工具:命令行接口工具
ros2中常用的一个CLI工具--rosbag2,这个工具用于记录话题的数据
(我们做一个真实机器人的时候非常有用,比如我们可以录制一段机器人发生问题的话题数据,录制完成后可以多次发布出来进行测试和实验,也可以将话题数据分享给别人用于验证算法)

常用指令
1.记录一个话题
ros2 bag record /sexy_girl

2.记录多个话题的数据
ros2 bag record topic-name1 topic-name2

3.记录所有话题
ros2 bag record -a

4.其他选项
-o name 自定义输出文件的名字
ros2 bag record -o fille-name topic-name
-s存储格式
目前仅支持sqlite3，其他还带扩展

查看录制出话题的信息
(比如话题记录的时间,大小,类型,数量)
ros2 bag info bag-file


播放话题数据
ros2 bag play xxx.db3

3.RQT工具
RQT是一个GUI框架,通过插件的方式实现了各种各样的界面工具

命令行:
rqt


4.数据可视化工具RVIZ2

数据:各种调试机器人时常用的数据,比如:图像数据,三维点云数据,地图数据,TF数据,机器人模型数据
可视化:可视化就是让你直观的看到数据,比如说一个三维的点(100,100,100)，通过RVIZ可以将其显示在空间中

注意:RVIZ强调将数据可视化出来,是已有数据的情况下，把数据显示出来而已,而后面讲的gazebo仿真软件是通过真实环境产生数据,两者用途并不一样


Gazebo集成ROS2
Gazebo是一个独立的应用程序,可以独立于ros2或ros使用
Gazebo与ros版本的集成通过一组叫做gazebo_ros_pkgs的包完成的,gazebo_ros_pkgs将Gazebo和ROS2连接起来

gazebo_dev:开发Gazebo插件可以用的API
gazebo_msgs:定义的ROS2和Gazebo之间的接口（Topic/Service/Action）
gazebo_ros:提供方便的C++类和函数,可供其他插件使用,例如转换和测试使用程序.他还提供一些通常有用的插件
gazebo_plugins:一系列Gazebo插件,将传感器和其他功能暴露给ROS2

gazebo_ros_camera 发布ROS2图像
gazebo_ros_diff_drive 通过ROS2控制和获取两轮驱动机器人的接口





5.ros2命令行工具总结

第七章

1.miniconda
退出conda    conda deactivate

2.jupyter安装
pip3 install jupyter -i https://pypi.tuna.tsinghua.edu.cn/simple

激活ros_venv环境
source ros_venv/bin/activate

然后在这里面:
启动jupyter
jupyter-notebook

3.Numpy
Numpy是一个功能强大的python库,主要用于对多维数组执行计算。

使用numpy定义矩阵
1.创建单位矩阵
np.identity(3)

2.创建零矩阵
np.zeros([3,3])

3.创建随机矩阵
np.random.rand(3,5)

4.从已有的数组创建矩阵
np.asarray([1,2,3,4]).reshape(2,2)

5.判断两个矩阵是否相等
numpy的allclose方法，比较两个array是不是每一个元素都相等,默认在1e-05的误差范围内

numpy进行矩阵运算

矩阵加法/减法
加法使用np.add
减法使用np.subtract

矩阵乘法
np.dot

矩阵求逆
np.linalg.inv

矩阵转置
矩阵转置在矩阵后使用.T即可 

4.空间位置姿态
描述三维空间中的姿态------旋转矩阵
旋转矩阵与位置矢量概念------略

1.平移矩阵----略

2.旋转矩阵----坐标变换
image.png


3.平移旋转复合矩阵
拆分
例如:可以将坐标变换拆分成先绕参考坐标系旋转,再绕参考坐标系平移,这样就得到了复合变换方程


左右手坐标系的区别

5.使用numpy表示位置和姿态
3*3单位矩阵表示没有姿态变换（注意不是零矩阵）



常见问题即解决方案
死锁

1.检查进程状态
如果进程存在且正常运行,等待它完成(尤其是系统更新时)
如果进程无响应或已结束但仍占用锁继续下一步

2.终止占用锁的进程(谨慎操作)
强制终止进程:

sudo kill -9 4800
使用-9强制终止卡死进程
注意:确保没有关键系统进程被误杀

3.检查并删除锁文件
查找并删除软件包管理锁文件:
sudo rm/var/lib/dpkg/lock
sudo rm/var/lib/apt/lists/lock
sudo rm/var/cache/apt/archives//lock
删除前建议检查是否有进程占用锁

4.修复软件包管理状态
//修复中断的dpkg操作
Sudo dpkg --configure -a
//修复依赖关系
sudo apt-get install -f
//清理并更新缓存
sudo apt-get clean
sudo apt-get update

5.重新运行原命令
附加说明
预防措施:避免同时运行多个包管理命令
系统日志:若问题反复出现,检查日志
//实时查看dpkg日志
Tail -f/var/log/dpkg.log


linux大小写敏感

全屏虚拟机
Ctrl + alt + enter

vscode切分终端
Ctrl + shift + 5

综合案例
1.手撸一个节点
python版
1.创建工作空间
mkdir -p town_ws/src
cd town_ws/src
code ./ 当前目录下打开vscode
2.创建功能包
创建一个名字叫village_li pythpn版本的功能包
ros2 pkg create village_li --build-type ament_python --dependencies rclpy
pkg create 是创建包的意思
--build-type 用来指定该包的编译类型, 一共三个可选项 ament_python,ament_cmake,cmake
--dependencies 指的是这个包的依赖,这里小鱼给一个ros2的python客户端接口rclpy
build-type什么都不写,ros2会默认为ament_cmake
3.创建节点文件
在__init__.py同级别目录下创建一个叫做li4.py的文件(在vscode中右击新建就行)
编写ROS2节点的一般步骤
1.导入库文件
2.初始化客户端库
3.新建节点
4.spin循环节点
5.关闭客户端库
import rclpy
from rclpy.node import Node

源代码见vscode
配置
在setup.py console_scripts里面添加
"li4_node=village_li.li4:main"

在工作空间下编译：   colcon build
为了让系统找到功能包和节点：source install/setup.bash
最后在工作空间下运行节点： ros2 run village_li li4_node 

ros2 node list
ros2 node info /li4

import rclpy
from rclpy .node import Node
from std_msgs.msg import String

"""
导入消息类型

声明并创建发布者

编写发布逻辑发布数据
"""

class WriterNode(Node):
    def __init(self,name):
        super().__init__(name)
        self.get_logger.info("大家好,我是%s,我是一名作家!"%name)
        self.pub_novel = self.create_publisher(String,"sexy_girl",10)

        self.count = 0
        self.timer_period = 5
        self.timer = self.create_timer(self.timer_period,self.timer_callback)


    def timer_callback(self):
        msg = String()
        msg.data = "第%d回连掩胡 %d 次偶遇呼延娘" % (self.count,self.count)
        self.pub_novel.publish(msg) #让发布者发布消息
        self.get_logger().info("发布了一个章节的小说内容是%s" % msg.data)
        self.count += 1
    


def main(args=None):
    """
    入口函数
    1.ros2运行该节点的入口函数
    2.编写ROS2的一般步骤
    3,新建节点对象
    4.spin循环节点
    5.关闭客户端库
    """
    rclpy.init(args=args)                              #初始化rclpy
    li4_node = WriterNode("li4")                           #新建一个节点
    rclpy.spin(li4_node)                               #保持节点运行,检测是否收到退出指令
    rclpy.shutdown()                                   #关闭客户端库


C++版
1.创建王家村功能包
王二居住在王家村，王家村和李家村不一样，是使用ament_cmake作为编译类型
所以王家村建立指令像下面这样,依赖变成rclcpp
新建工作空间（新建功能包）
ros2 pkg create village_wang --build-type ament_cmake --dependencies rclcpp
2.创建节点
在village_wang/src下创建一个wang2.cpp文件

配置rclcpp/rcpcpp.hpp路径

源代码见vscode

修改CMakeList.txt文件
在CMakeList.txt最后一行加入下面两行代码
add_executable(wang2_node src/wang2.cpp)
ament_target_dependencies(wang2_node rclcpp)

添加两行代码的目的是让编译器编译wang2.cpp这个文件,不然不会主动编译.接着在上面两行代码下面添加下面代码
install(TARGETS
wang2_node
DESTINATION lib/${PROJECT_NAME}
)
这个是C++比python更麻烦的地方,需要手动将编译好的文件安装到install/village_wang/lib/village_wang

编译运行节点
在工作空间下
colcon build --packages-select village_wang

ros2 pkg list | grep vill

source install/setup.bash

ros2 run village_wang wang2_node

2.编写python话题发布者
1.导入消息类型
2.声明并创建发布者
3.编写发布逻辑发布数据
编写cpp话题发布者
3.编写python话题订阅者
1.导入订阅的话题接口类型
2.创建订阅回调函数
3.声明并创建回调者
4.编写订阅回调处理逻辑

4.编写cpp话题发布者
1.创建一个话题订阅者的能力,用于拿到艳娘传奇的数据
2.创建一个话题发布者的能力,用于给李四送稿费
3.获取日志打印器的能力
5.编写cpp话题订阅者










![图片 1](/images/feishu/ros2/image_3062ac.png)
