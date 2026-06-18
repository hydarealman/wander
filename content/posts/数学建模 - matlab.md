---
title: "数学建模 \\- matlab"
slug: "数学建模-matlab"
date: 2026-06-06T16:05:42+08:00
draft: false
source_file: "数学建模 - matlab.md"
source_size: 6466
source_lines: 254
tags:
  - "算法"
categories:
  - "算法数学"
---

# 数学建模 \- matlab

# matlab

## 1\.matlab界面及基本操作

clc \-\-\- 清空命令行

clear \-\-\- 清空工作区

按上方向键 \-\-\- 调用历史命令

% 后面写的都是注释

命令行敲回车执行 

脚本文件 函数文件结尾尾缀\.m

实时脚本文件尾缀\.mlx

分节符

## 2\.matlab中两种引号

如果字符串本身有双引号,用双重双引号,让matlab识别双引号

如果字符串本身有单引号,用双重单引号,让matlab识别单引号

双引号得到的是1个string变量,单引号得到的是多个char变量

所以单引号可以用\(\)访问第几个字符

## 3\.matlab矩阵运算

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Mzg4OTIzY2M0Y2ViNjJmOWNkYTA2NzJiYzlhZWNjZTZfMGJhNjljZWZhNzBhMGM4ZTkzZmIzNWE2MmJmZTUzOTZfSUQ6NzQ4OTM1MzI1Njk4MDg3MzIzNV8xNzgwNzMzMTM4OjE3ODA4MTk1MzhfVjM)

1\.plot\(b\) plot函数作图以索引为横坐标\.索引就是该数字在矩阵里是"第几个"

2\.grid on添加灰色网格线

3\.多维矩阵:以空格或逗号分隔同一行元素,分号分隔各行

4\.常见运算：转置A'，取逆inv\(），求特征值和特征向量eig\(\)

5\.矩阵乘法\*,和矩阵点乘\.\*

6\.矩阵方程求解

A\*x = b

X = A\\b    %表示A的逆矩阵乘以矩阵b\(无论是斜杠还是反斜杠谁在相对下面的位置谁就取逆矩阵\)

7\.如果一个操作数是标量,而另一个数不是标量,则matlab会将该标量隐式扩展为与另一个操作数具有相同的大小

隐式扩展

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODJkMjk1MTFjNDRhNTQxNGM4NmUyYTA1N2Q1YzJhNThfYmE0OWJmZGE2Y2FjMWYxZDI3ZGM2ZThjZDdmY2U0NjhfSUQ6NzQ4OTM1NjAxODYxMzM4NzI2Nl8xNzgwNzMzMTM4OjE3ODA4MTk1MzhfVjM)

## 4\.matlab的4种二维图

1\.线图

plot函数代码示例

```MATLAB
x = 0:0.05:30;   %从0到30,每隔0.05取一次值
y = sin(x)
y2 = cos(x)
%plot(x,y,'r','LineWidth',2) 
plot(x,y,'r',x,y2,'g','LineWidth',2)
xlabel("横轴标题")
ylabel("纵轴标题")
grid on     %显示网格
axis([0 20 -1.5 1.5])   %设置横纵坐标范围
```

2\.条形图

bar函数创建条形图

barh函数用来创建水平条形图

```MATLAB
t = -3:0.5:3;
p = exp(-t.*t)  %e的-t的平方
bar(t,p)
barh(t,p)
```

3\.极坐标图

polarplot函数用来绘制极坐标图

```MATLAB
theta = 0:0.01:2*pi;    %弧度
%abs求绝对值或复数的模
radi = abs(sin(2*theta).*cos(2*theta));    %半径(注意是点乘)
polarplot(theta,radi)   %括号内是弧度和半径
```

4\.散点图

scatter函数用来绘制x和y值的散点图

## 5\.matlab三维图和内嵌子图

1\.三维曲面图

surf函数可以用来做三维曲面图。一般是展示函数z = z\(x,y\)的图像\.

首先需要用meshgrid创建好空间上\(x,y\)点

```MATLAB
[X,Y] = meshgrid(-2:0.05:2);
%Z = X.^2 + Y.^2
Z = X.*exp(-X.^2-Y.^2);
surf(X,Y,Z);
colormap hsv    % colormap设置颜色,可跟winter,summer等
colorbar
```

2\.子图

使用subplot函数可以在同一窗口的不同子区域显示多个绘图

```matlab
theta = 0:0.01:2*pi;    %弧度
%abs求绝对值或复数的模
radi = abs(sin(2*theta).*cos(2*theta));    %半径(注意是点乘)
Height = randn(1000,1)  %生成1000行1列的符合正太分布的随机数
Weight = randn(1000,1)

subplot(2,2,1); surf(X.^2); title('1st');
subplot(2,2,2); surf(Y.^3); title('2nd');
subplot(2,2,3); polarplot(theta,radi); title('2nd');
subplot(2,2,4); scatter(Height,Weight); title('4th');
```

## 6\.matlab导入数据

### 导入的范围

导入的数据的范围默认是从第二行开始的，第一行一般是标题行

如果不想导入所有数据，可以按住ctrl键，选择想导入的内容，例如某行，某列

变量名称行也就是导入之后，matlab里表讴歌最上方会显示变量，一般默认选择原文件第一行。但是只能识别英文\.如果是汉字则变成VerName

### 导入类型

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGYwYWM3Y2FhZDU1MTc2OTczYTBlMjQ3ZmM2MmMyZGZfOTBkYzc3MjM1NGE2ZDFmYTA4ZDJhMmY1Mjg4MzI5Y2FfSUQ6NzQ4OTQxNjM1OTM0MDk5ODY2MF8xNzgwNzMzMTM4OjE3ODA4MTk1MzhfVjM)

### 处理无法导入的数据

选择替换,则所有字符串都变成NaN

选择排除行，那么某一行只要有字符串，这一行数据都不会被导入

选择排除列，同上

## 7\.matlab处理缺失值和异常值

# 算法

## 线性规划

### 概念

线性规划就是再一组线性约束条件下,求线性目标函数的最大或最小值\.

''线性"就是所有变量都是一次方

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODFmNWQ1Mzg5NjYzZWJkYjU0NTQ0MDFmMTRjNzkxNzZfMDRkYTJiMTdlMzZhMmYzMTYyZGY1ZjIyMzAxOWMyY2ZfSUQ6NzQ4OTc4Nzk2Nzc0MDI0ODA2OF8xNzgwNzMzMTM4OjE3ODA4MTk1MzhfVjM)

关键词:怎样安排/分配，尽量多少，利润最大，最合理

### 线性规划\-代码实现

模型化为matlab标准型:目标函数最小值,约束条件小于等于号或等号

求y的最大值等价于求\-y的最小值

#### linprog

1\.求解线性规划问题

```MATLAB
x = linprog(f, A, b, Aeq, beq, lb, ub)
```

- `f`：目标函数的系数向量，表示目标函数 min**f***T***x**。

- `A` 和 `b`：不等式约束 *A***x**≤**b**。

- `Aeq` 和 `beq`：等式约束 *Aeq***x**=**b***eq*。

- `lb` 和 `ub`：变量的上下界，分别表示 **lb**≤**x**≤**ub**。

#### `intlinprog`

2\.求解整数线性规划问题

```MATLAB
x = intlinprog(f, intcon, A, b, Aeq, beq, lb, ub)
```

#### `linprog` 的其他选项

3. `options`：可以通过 `optimoptions` 设置优化选项，例如算法选择（`'dual-simplex'`、`'interior-point'` 等）。

```MATLAB
options = optimoptions('linprog', 'Algorithm', 'dual-simplex');
x = linprog(f, A, b, Aeq, beq, lb, ub, options);
```

#### `optimproblem`

4\.用于定义优化问题，包括线性规划问题。

```MATLAB
prob = optimproblem;
prob.Objective = f' * x;
prob.Constraints.cons1 = A * x <= b;
prob.Constraints.cons2 = Aeq * x == beq;
[sol, fval] = solve(prob);
```

#### `optimoptions`

5\.用于设置优化选项，例如算法选择、容差、迭代次数等。

```MATLAB
options = optimoptions('linprog', 'Algorithm', 'interior-point', 'Display', 'iter');
```



可以转化为线性规划的问题







#### 线性规划模型建模实战与代码

没搞太懂



## 整数规划

