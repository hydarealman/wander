---
title: "双机械臂防碰撞 Demo 调试总结"
slug: "jakac5机械臂日志"
date: 1970-01-21T23:10:24+08:00
draft: false
source_file: "feishu://jakac5机械臂日志"
source_size: 14949
source_lines: 396
tags: []
categories: []
---

jakac5机械臂日志
开发日志:
6月9日
配置moveit_resources-ros2的官方例程
跑通单臂的rviz

6月10日
跑通双臂的rviz

6月11日 & 6月12日
准备考试 暂时搁置

6月13日
熟悉整个代码框架


6月14日 & 6月15日
复习数电

6月16日
实现jaka c5 双臂防碰撞轨迹规划执行

遇到的bug: 

# 双机械臂防碰撞 Demo 调试总结

> 本文档记录了搭建 JAKA C5 双机械臂防碰撞演示项目过程中遇到的关键 Bug 及其解决方案。

---

## Bug 1：轨迹时间戳全为零，Plan 成功但 Execute 失败

**现象：**
- RViz 中 Plan 能找到无碰撞路径（OMPL 规划成功）
- Plan & Execute 时机械臂不动
- move_group 日志：`Time between points 0 and 1 is not strictly increasing, it is 0.000000`
- 手动发送带时间戳的轨迹到 controller 可以正常执行

**根因：**
`AddTimeOptimalParameterization`（时间最优轨迹时间参数化）响应适配器没有被正确加载。

在 ROS2 Humble 的 MoveIt2 中，该类注册在：
```
default_planner_request_adapters/AddTimeOptimalParameterization
```

但我们的 `ompl_planning.yaml` 中把它放在了 `response_adapters`，且用了**错误的命名空间**：
```yaml
# ❌ 错误配置（双机械臂套件常见错误）
response_adapters: "default_planner_response_adapters/AddTimeOptimalParameterization ..."
```

`default_planner_response_adapters/` 这个命名空间下没有注册任何插件，导致 `AddTimeOptimalParameterization` 加载失败，轨迹的时间戳全为零。

对比单机械臂 panda_moveit_config（可正常工作）：
```yaml
# ✅ 正确配置
request_adapters: "... default_planner_request_adapters/AddTimeOptimalParameterization"
```

**解决：**
将 `AddTimeOptimalParameterization` 移到 `request_adapters` 链中，使用正确的命名空间 `default_planner_request_adapters/`。

**教训：**
- 不要混用 `default_planner_request_adapters/` 和 `default_planner_response_adapters/`
- 所有 MoveIt2 Humble 的 motion planning adapter 都注册在 `default_planner_request_adapters/` 下
- 用 `cat /opt/ros/humble/share/moveit_ros_planning/planning_request_adapters_plugin_description.xml` 可查看所有已注册的适配器

---

## Bug 2：RViz 拖动交互标记只能旋转不能平移

**现象：**
- RViz 中拖动机械臂末端的交互标记时，只能改变末端姿态（旋转），不能改变位置（平移）
- 之前可以正常拖动，切换为 OMPL 后出现问题

**根因：**
在 SRDF 中定义了末端执行器（end effector）组，但配置不完整——定义了 `parent` 和 `group` 但不匹配，导致 MoveIt 的逆运动学（IK）求解器无法正确计算末端位置对应的关节角。

**解决：**
删除了不完整的末端执行器定义，回归之前的简洁配置。在没有末端执行器的情况下，RViz 使用规划组（left_arm / right_arm）的 tip link 作为交互标记的参考点，IK 求解正常工作。

---

## Bug 3：CHOMP 规划器被误选为默认规划器

**现象：**
- move_group 日志显示使用了 `chomp_interface/CHOMPPlanner`
- CHOMP 规划失败或行为不符合预期
- 明确配置了 `planning_plugin: "ompl_interface/OMPLPlanner"` 但无效

**根因：**
MoveIt2 的规划器选择逻辑：当系统中安装了多个规划器插件时（CHOMP、OMPL、STOMP 等），MoveIt 可能选择非预期的默认规划器。`planning_plugin` 参数在某些配置路径下被忽略。

**解决：**
卸载 CHOMP 规划器包，确保 OMPL 是唯一可用的规划器：
```bash
sudo apt remove ros-humble-moveit-planners-chomp
```

---

## Bug 4：move_group 启动崩溃 — YAML 参数格式错误

**现象：**
```
[FATAL] Cannot have a value before ros__parameters
```
move_group 进程启动后立即崩溃。

**根因：**
`move_group_params.yaml`（或类似命名的 ROS 参数文件）格式不正确。ROS2 Humble 的 `rclcpp` 要求参数文件必须以 `/**` 节点名开头，然后是 `ros__parameters` 键：

```yaml
# ❌ 错误格式
planning_plugin: "ompl_interface/OMPLPlanner"

# ✅ 正确格式
/**:
  ros__parameters:
    planning_plugin: "ompl_interface/OMPLPlanner"
```

**解决：**
最终放弃了独立的参数 YAML 文件，改为在 launch 文件中通过 Python 字典传递参数（`moveit_config.to_dict()`），这由 `MoveItConfigsBuilder` 自动处理。

---

## Bug 5：Ros2ControlManager 崩溃 — 插件未找到

**现象：**
```
[FATAL] [moveit_ros_control_interface]: The 'moveit_ros_control_interface/Ros2ControlManager' plugin failed to load
```
move_group 启动后崩溃。

**根因：**
`Ros2ControlManager` 需要特定的插件库，在当前 ROS2 Humble apt 安装的版本中不可用或未正确注册。

**解决：**
切换为 `MoveItSimpleControllerManager`，配置 `FollowJointTrajectory` 动作接口：

```yaml
moveit_controller_manager: moveit_simple_controller_manager/MoveItSimpleControllerManager

moveit_simple_controller_manager:
  controller_names:
    - left_arm_controller
    - right_arm_controller
  left_arm_controller:
    type: FollowJointTrajectory
    action_ns: follow_joint_trajectory
    joints: [left_joint_1, ..., left_joint_6]
```

---

## Bug 6：OMPL 配置 "Could not find the planner configuration 'RRTConnect'"

**现象：**
move_group 日志报错找不到 `RRTConnect` 规划器配置。

**根因：**
`ompl_planning.yaml` 中设置了一个指向不存在配置名称的 `default_planner_config`：
```yaml
# ❌ 错误
default_planner_config: RRTConnect
```
实际的配置名是 `RRTConnectkConfigDefault`（带 `kConfigDefault` 后缀）。

**解决：**
删除 `default_planner_config` 和 `longest_valid_segment_fraction` 键（这些是 MoveIt1 的配置项，MoveIt2 中不再支持或移动到其他位置）。

---

## 总结

| Bug | 类别 | 严重程度 | 修复方式 |
|---|---|---|---|
| AddTimeOptimalParameterization 命名空间错误 | 配置 | 🔴 阻塞 | 移到 request_adapters + 正确命名空间 |
| 交互标记无法平移 | 配置 | 🟡 中等 | 删除不完整的 end effector 定义 |
| CHOMP 被误选 | 依赖 | 🟡 中等 | 卸载 CHOMP 包 |
| YAML 参数格式错误 | 配置 | 🔴 崩溃 | 改用 Python 字典传参 |
| Ros2ControlManager 加载失败 | 插件 | 🔴 崩溃 | 改用 MoveItSimpleControllerManager |
| OMPL 配置名无效 | 配置 | 🟡 中等 | 删除无效的 default_planner_config |

**核心教训：** 参考官方单机械臂配置（panda_moveit_config）是验证双机械臂配置正确性的最可靠方法。当双机械臂配置出现问题时，逐项对比与单机械臂配置的差异是最有效的调试策略。


知识点:
碰撞检测
MoveIt 在做运动规划时，要保证机械臂在运动中不撞到任何东西。为此它需要检查：
机械臂自己的连杆之间是否会自碰撞（self-collision）
机械臂是否碰到了环境中的物体（障碍物）

MoveIt 假设任何两个连杆都可能碰撞，会对所有连杆对做检测。但这样做非常消耗计算资源。实际上很多连杆对是永远不可能碰撞的——disable_collisions 就是用来告诉 MoveIt："这对不用查了"，从而提高规划效率。

reason的三种取值



URDF 只定义机器人的几何、运动学、惯性、碰撞模型（连杆、关节、形状等）。
SRDF 补充高层语义：哪些关节可以一起运动（规划组）、预设姿态、哪些连杆之间永远不检查碰撞、末端执行器定义、虚拟关节等

<group> – 规划组
定义一组关节/连杆，用于运动规划。

<group_state> – 预设关节状态
某个组定义一组关节的命名目标值，便于快速调用（如回家、伸展、闭合手爪）


<virtual_joint> – 虚拟关节
将机器人连接到外部坐标系（如世界坐标系、基座固定点）。
type="floating" 表示机器人可以在世界坐标系中自由移动（一般用于移动基座或仿真中的浮动基座）。对于固定基座机械臂，这里只是建模方便，实际运动中不会产生位移。

<disable_collisions> – 禁用碰撞检测
指定两个连杆之间永远不需要检查碰撞（提高规划效率）

<end_effector> – 末端执行器定义
将某个组标记为末端执行器，并关联到父组（手臂）。

passive_joint> – 被动关节
在 <group> 内部标记一个非驱动的从动关节（如耦合手指的第二关节）。该关节随主动关节运动，不需要额外控制。

OMPL（Open Motion Planning Library）
OMPL中的算法都属于基于采样的运动规划,核心思路是在机器人的关键空间
(Configuration Space)中随机采样,然后尝试把这些随机点连接成一条从起点到终点的无碰撞路径

参数调优的本质，是在探索速度(快速找到一条可行路径)和路径质量
(路径短不短,平滑不平滑)之间做权衡

range: 它代表每次扩展时,从当前节点向外探索的最大步长
较大 : 树生长快,探索范围广，规划速度快        适用 空旷环境,快速 找路
较小 : 探索更精细,路径更平滑,但规划变慢       适用 狭窄通道,高精度需求
0.0   : OMPL会根据状态空间自动计算合适的值 适用 大多数情况下的安全选择



单树采样: 
代表算法: 
RRT, RRTstar, TRRT, STRIDE, KPIECE, EST, SBL, PDST, ProjEST, LBTRRT
特点: 
从起点开始生长一棵树，适合低维或中等维度空间

双树采样: 
代表算法:
RRTConnect, BiTRRT, BiEST
特点: 
同时从起点和终点生长两棵树，收敛快，常用于高维机械臂

多查询/概率图
代表算法: 
PRM, PRMstar, LazyPRM, LazyPRMstar, SPARS, SPARStwo
特点: 
先随机采样构建路图，再查询路径，适合多次规划

最优路径: 
代表算法: 
RRTstar, PRMstar, BFMT, FMT, TRRT
特点: 
渐进地优化路径长度（或其它代价）

任意时间优化
代表算法: 
APS (Anytime Path Shortening)

特点: 
先快速找到可行路径，然后在剩余时间内不断缩短

轨迹规划: 
代表算法: 
TrajOpt（基于梯度的局部优化）

特点: 
从初始猜测出发，通过非线性优化得到光滑无碰撞轨迹



世界坐标系(根链接)
有且仅有一个根节点: 
URDF（统一机器人描述格式）在解析时强制要求整个模型呈树状结构
(Tree Structure),不能有环,也不能有多个根,如果没有word这个链接,左臂和右臂各自是一个独立的树,无法合并成一个合法的机器人模型文件

word充当了左右两个独立机械臂的公共父级,通过它,两个原本独立的运动学
子树被焊接到了同一个坐标系下,满足了ROS对机器人模型的底层数据结构要求

定义安装基准: 在word坐标系下,你通过你通过 <origin xyz="0 -0.25 0" /> 和 xyz="0 0.25 0" 定义了双臂基座的安装位置。这意味着：
左臂基座中心点位于 world 的 Y 轴负方向 0.25m 处。
右臂基座中心点位于 world 的 Y 轴正方向 0.25m 处。


在TF(坐标变换)树中的特殊地位
所有变换的源头
静态变换发布: 在此URDF被加载到robot_state_publisher后,它会自动发布从
wordl到左右臂基座连杆的静态变换,这些变换时永恒不变的,为Moveit运动规划提供了绝对参考系


物理仿真: 
默认无质量刚体: 
在物理引擎中,<link name="world"/>如果没有定义<inertial>（惯性矩阵）和<collision>（碰撞体）,它会被引擎自动视为固定不动的"大地",质量视为无穷大,位置绝对锁定

千万不能加惯性参数: 
这个"世界链接"会受重力影响直接向下坠落,导致整个双臂模型瞬间他先散架,因此根链接通过保持空链接,只作为纯数学参考点



URDF(统一机器人描述格式): 
全称：Unified Robot Description Format
职责：描述机器人的物理与几何属性，是 ROS 中最底层的模型文件

内容: 
连杆: Visual Collision Inertial
关节: origin parent/child axis limit

谁在使用: 
Rviz Gazebo/lgnition/Moveit

SRDF(语义机器人描述格式) : Moveit的"策略配置文件"
全称：Semantic Robot Description Format
职责: 基于URDF提供额外的"规划层信息"，专门服务于运动规划框架Moveit

规划组(Group)
意义: MoveIt 不用关心底座（world）和基座（Link_00）的固定关节，只需要知道控制这 6 个旋转轴就能移动末端。

预设姿态(Group State)
意义: 一键让手臂跳到指定角度，省去手动拖动滑块的麻烦，也是程序启动时的安全初始化姿势。

碰撞禁用矩阵(Disable Collisions)
意义: 极大降低碰撞检测的 CPU 计算量，让规划速度翻倍。

虚拟关节(Virtual Joints)
用于把机器人固定在浮动基座上（比如移动底盘）

笛卡尔空间:
xyz三维直角坐标系
rpy角: roll横滚角 pitch俯仰角 yaw偏航角

描述对象: 末端执行器(手抓 吸盘 焊枪) 在物理位置的具体位置和朝向


关节空间: 
机械臂所有关节变量的集合,对于旋转关节,它是角度值(θ1, θ2 ... θn)
对于平移关节,它是位移值(d1, d2 ... dn)

描述对象: 机械臂本身的整体构型



正逆运动学
正运动学（Forward Kinematics, FK）
知道关节角度,求解末端位置
计算方向: 关节空间 -> 笛卡尔空间
齐次变换矩阵的顺序连乘
唯一解


逆运动学（Inverse Kinematics, IK）
知道末端位置,求关节角度
计算方向: 笛卡尔空间 -> 关节空间
求解非线性超越方程组
零解 单解 多解 无穷解


IK求解器
KDL MoveIt默认的IK求解器。它是一个基于牛顿-拉夫森迭代法的数值求解器

arm_group:
  kinematics_solver: kdl_kinematics_plugin/KDLKinematicsPlugin
  kinematics_solver_timeout: 0.005
  kinematics_solver_attempts: 3



TRAC-IK (更可靠的升级版)：一个为替代KDL而生的数值求解器。它通过并发运行两种算法（改进的KDL算法和SQP非线性优化）来提高求解成功率

arm_group:
  kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
  kinematics_solver_timeout: 0.005 # 超时时间（秒）
  solve_type: Speed # 可选: Speed, Distance, Manip1, Manip2
  # kinematics_solver_attempts: 3 # TRAC-IK 不需要此参数


IKFast (追求极致性能)：一个解析求解器，通过编译器生成针对特定机器人的C++代码，速度极快。
优点：速度极快（可达微秒级），能找到所有数学解。
缺点：配置极其复杂，依赖OpenRAVE环境。
配置方法：需要安装OpenRAVE，将URDF转换为DAE格式，再用IKFast生成代码，最后编译成MoveIt插件。
pick_ik (新一代选择)：一个较新的IK求解器，结合了全局优化（进化算法）和局部优化（梯度下降）。
优点：鲁棒且可定制，能更好地找到全局最优解。
配置示例:

arm_group:
  kinematics_solver: pick_ik/PickIkPlugin
  kinematics_solver_timeout: 0.05
  mode: global
  position_threshold: 0.001
