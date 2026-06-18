---
title: "stm32"
slug: "stm32"
date: 2026-06-06T16:07:25+08:00
draft: false
tags:
  - "机器人"
categories:
  - "机器人视觉"
---

# stm32

## stm32基础知识

stm32开发方式

基于寄存器的方式

基于标准库

和基于HAL库的方式



## 新建工程

\.建立工程文件夹,Keil中新建工程,选择型号

\.工程文件夹中建立Start，Library,User等文件夹，复制固件库里面的文件到工程文件夹

\.工程里对应建立Start,Library,User等同名称的分组,然后文件夹内的文件添加到工程分组里

\.工程选项,C/C\+\+,Include Paths内声明所有包含头文件的文件夹

\.工程选项,C/C\+\+,Define内定义USE\_STDPERIPH\_DRIVER

\.工程选项,Debug,下拉列表选择对应调试器,Settings,Flash,Download里勾选Reset and Run



## GPIO输出

\.GPIO通用输入输出口

\.可配置为8种输入输出模式

\.引脚电平:0V\~3\.3V,部分引脚可容忍5V

\.输出模式下可控制端口输出高低电平,用以驱动LED，控制蜂鸣器

模拟通信协议输出时序

\.输入模式下可读取端口的高低电平或电压,用于读取按键输入,外接模块电平信号输入,ADC电压采集,模拟通信协议接收数据等





八种输入输出模式

浮空输入\-\-\-\-数字输入\-\-\-\-可读取引脚电平,若引脚悬空,则电平不确定

上拉输入\-\-\-\-\-数字输入\-\-\-\-可读取引脚电平,内部连接上拉电阻,悬空时默认高电平

下拉输入\-\-\-\-\-数字输入\-\-\-\-\-可读取引脚电平,内部连接下拉电阻,悬空时默认低电平

模拟输入\-\-\-\-\-\-模拟输入\-\-\-\-\-GPIO无效,引脚直接接入内部ADC

开漏输出\-\-\-\-\-数字输出\-\-\-\-\-可输出引脚电平,高电平直接接VDD，低电平接VSS

推挽输出\-\-\-\-\-数字输出\-\-\-\-\-可输出引脚电平，高电平接VDD，低电平接VSS

复用开漏输入\-\-\-\-\-数字输出\-\-\-\-\-由片上外设控制m,高电平为高组态,低电平接VSS

复用推挽输入\-\-\-\-\-数字输出,由片上外设控制\.高电平接VDD，低电平接VSS





## GPIO输出



1。按键：常见的输入设备，按下导通,松手断开

按键抖动:由于按键内部使用的是机械式弹簧片来进行通断,所以在按下和松手的瞬间会伴随一连串的抖动



2\.传感器:传感器元件\(光敏电阻/热敏电阻/红外接受管等\)的电阻会随外界模拟量的变化而变化,通过与定值电阻分压即可得到模拟电压输出,再通过电压比较器进行二值化即可得到数字电压输出





## 调试方式

串口调试: 通过串口通信,将调试信息发送 到电脑端,电脑使用串口助手显示调试信息



显示屏调试: 直接将显示屏连接到单片机,将 调试 信息打印在显示屏上 



Keil调试模式: 借助Keil软件的调试模式 可使用 单步运行,设置断点,查看寄存器及变量等功能



对照法



逐行注释发



点灯法



测试程序的基本思想: 缩小范围,控制变量,对比测试





## OLED简介

OLED：有机发光二极管

OLED显示屏: 性能优异的新型显示屏,具有功耗低,响应速度快,宽视角,轻薄柔韧等特点

0\.96寸OLED模块: 小巧玲珑，占用接口少,简单易用,是电子设计中非常常见的显示屏模块

供电: 3\~3,5V, 通信协议 : I2C/SPI, 分辨率: 128\*64





## 中断系统

中断: 在主程序运行过程中,出现了特定的中断触发条件\(中断源\), 使得CPU暂停正在运行的程序,转而去处理中断程序,处理完成后又返回原来被暂停的位置继续运行



中断优先级: 当有多个中断源同时申请中断时,CPU会根据中断源的轻重缓急进行裁决,优先响应更加紧急的中断源



中断嵌套: 当一个中断程序正在运行时, 又有新的更高优先级的中断源申请中断,CPU再次暂停当前中断程序,转而去处理新的中断程序,处理完成后依次进行返回





## EXTI简介

EXTI外部中断

EXTI可以检测指定GPIO口的电平信号,当其指定的GPIO口产生电平变化时,EXTI将立即向NVIC发出中断申请,经过NVIC裁决后即可中断CPU主程序,使CPU执行对应的中断程序。

支持的触发方式: 上升沿/下降沿/双边沿/软件触发

支持的GPIO口: 所有GPIO口,但相同的Pin不能同时触发中断

通道数: 16个GPIO\_Pin，外加PVD输出,RTC闹钟,USB唤醒,以太网唤醒

触发响应方式: 中断响应/事件响应





## NVIC优先级分组

NVIC的中断优先级由寄存器的4位\(0\~15\)决定,这4位可以进行切分,分为高n位的抢占优先级和低4\-n位的响应优先级

抢占优先级高的可以中断嵌套,响应优先级高的可以优先排队 ,抢占优先级和响应优先级均相同的按中断号排队





## AFIO复用IO口

AFIO主要用于引脚复用功能的选择和重定义

在STM32中,AFIO主要完成两个任务 : 复用功能引脚重映射,中断引脚选择

## 

## 旋转编码器介绍

旋转编码器: 用来测量位置,速度或旋转方向的装置,当其旋转轴旋转时,其输出端可以输出与旋转速度和方向对应的方波信号,读取方波信号的频率和相位信息即可得知旋转轴的速度和方向

类型: 机械触点式/霍尔传感器式/光栅式





## TIM定时器

定时器可以对输入的时钟进行计数,并在计数值达到设定值时触发中断

16位计数器,预分频器,自动重装寄存器的时基单元,在72MHz计数时钟下可以实现最大59\.65s的定时

不仅具备基本的定时中断功能,而且还包含外时钟选择,输入捕获,输出比较,编码器接口,主从触发模式等功能

根据复杂度和应用场景分为了高级定时器,通用定时器,基本定时器三种类型





## 定时器类型

高级定时器

编号: TIM1,TIMB   总线: APB2 

功能: 拥有通用定时器全部功能,并额外具有重复计数器,死区生成,互补输出,刹车输入等功能



通用定时器

编号: TIM2,TIM3,TIM4,TIM5    总线: APB1

功能: 拥有基本定时器全部功能,并额外具有内外时钟源选择,输入捕获,输出比较,编码器接口,主从触发模式等功能



基本定时器

编号: TIM6,TIM7   总线: APB1

功能: 拥有定时中断,主模式触发DAC的功能



补充: STM32F103C8T6定时器资源: TIM1,TIM2,TIM3,TIM4





定时中断基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDMwYmMzZjdiYTU3ZmQyZmZlM2MxMzUzZGIyOTIzOTFfMmMzZTQ0ZDIzZTgxOTlkM2E3ZDlhMzM3NjNjYWMxOGJfSUQ6NzUzNTMwNDUzMzUyNTg1NjI1OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 输出比较简介

OC输出比较

输出比较可以通过比较CNT与CCR寄存器值的关系,来对输出电平进行置0,置1或翻转的操作,用于输出一营频率和占空比的PWM波形

每个高级定时器和通用定时器都拥有4个输出比较通道 

高级定时器的前3个通道额外拥有死区生成和互补输出的功能



PWM简介

PWM脉冲宽度调制

在具有惯性的系统中,可以通过一系列的宽度进行调制,来等效地获得所需要地模拟参量,常用于电机控速等领域

PWM参数:

频率 = 1 / Ts      占空比 = Ton / Ts    分辨率 =   占空比变化步距





输出比较模式

模式                                                描述

冻结                                                CNT=CCR，REF保持为原状态

匹配时置有效电平                              CNT=CCR，REF置有效电平

匹配时置无效电平                              CNT=CCR,REF置无效电平

匹配时电平翻转                                 CNT=CCR，REF电平翻转

强制为无效电平                                 CNT与CCR无效,REF强制为无效电平

强制为有效电平                                 CNT与CCR无效,REF强制为有效电平

PWM模式1             向上计数: CNT \< CCR, REF置有效电平, CNT \>= CCR ,REF置无效电平

向下计数: CNR \> CCR，REF置无效电平，CNT\<=CCR,REF置有效电平

PWM模式2                                          

向上计数: CNT\<CCR,REF置无效电平，CNT\>=CCR,REF置有效电平

向下计数：CNT\>CCRREF置无效电平，CNT\<=CCR,REF置无效电平

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODQ3ZGQwYjI2YzVkNWYwMmNmZjhjNWRiY2VmNzBjMWJfYTEyYTViYzBjMDE2NGZjMjgzYjEwNDI5YjA3OGYwNDRfSUQ6NzUzNTY0NDk2MzE2NDM4OTM3N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

参数计算

PWM频率:     Freq = CK\_PSC/\(PSC \+ 1\) / \(ARR \+ 1\)

PWM占空比：Duty = CCR / \(ARR \+ 1\)

PWM分辨率:  Reso = 1 / \(ARR \+ 1\)



## 舵机简介:

舵机是一种根据输入PWM信号占空比来控制输出角度的装置

输入PWM信号要求: 周期为20ms,高电平宽度为0,5ms\~2\.5ms



## 直流电机及驱动简介

直流电机是一种将电能转换为机械能的装置,有两个电极，当电极正接时,电机正转,当电极反接时,电机反转

直流电机输入大功率器件，GPIO口无法直接驱动,需要配合电机驱动电路来操作

TB6612是一款双路H桥型的直流电机驱动芯片,可以驱动两个直流电机并且控制其转速和方向



//这里其实是把PWM波形当成一个通信协议来做的

输入信号脉冲宽度                       舵机输出轴转角

0\.5ms                                        \-90°

1ms                                           \-45°

1\.5ms                                         \-0°

2ms                                            45°

2\.5ms                                          90°

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTI2NjJlOTI1ZTk5ZTUwZTM5NzUzNTExNTM3NGEzOTBfZjc2ZmFmMTEzODRjNDUyODJiMTViYzJjOTljMzNlNGZfSUQ6NzUzNTY1MzI3NzUzNjgyOTQ0MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 输入捕获简介

IC输入捕获

输入捕获模式下，当通道输入引脚出现指定电平跳变,当前CNT的值将被锁存到CCR中,可用于测量PWM波形的频率,占空比,脉冲间隔,电平持续时间等参数

每个高级定时器和 通用定时器都拥有4个输入捕获通道

可配置为PWMI模式,同时测量频率和占空比

可配合主从触发模式,实现硬件全自动测量



## 频率测量

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzMxZGFiYWE5ZjhkYmI1MWUyZmE2ZDQ0ODNkNjdkZjlfZTE2OTY1ODcyYmRhZjgyZDlkYmZkNzcyMDM3YTI4YTFfSUQ6NzUzNjQxMDU1NDU5NzQ1NzkyM18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 输入捕获通道

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGZjNjgzMjY1MzBjYjJiMmY1NjBlNmUyMDM4YmZhZGFfMGIwN2IxNTA0MGI1MTA2ZWFlNGNkNjhjYzc1OThmNzdfSUQ6NzUzNjQxNTAxMjc5NDMxODg3Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 主从触发模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Mjg3YTQ0NDY5ZDJiNzA0YWMzNTVmNTA5OWRhYjI1OTdfNmRkZjViYWMyODQ4ZWJjOWEwZGIzNDkwZTM0NjAyYjZfSUQ6NzUzNjQxNjEzNzEzNjA2MjQ4M18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 输入捕获基本模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGM2ZGFiZGIxMDkxNjIzNGU0OTQ0MGI1NDJlNDllM2JfODA1NjE3YjI4NjZhZWQ2NWE2NDAzZTYzYWYzYmU2ZGVfSUQ6NzUzNjQxNzIwODU3OTk5NzY5N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## PWMI基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzVjYjI3YTFjYjY3YWEyNmM4ZjE4YWVjMGYxYWY2OGFfNTJmMjVlMThlZTczMTMyYTU5MTQ3MzE2ODJiZjc1MjBfSUQ6NzUzNjQxODM3OTU3OTA5NzA5Ml8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 编码器接口简介

Encoder Intefface编码器接口

编码器接口可接收增量\(正交\)编码器的信号,根据编码器旋转产生的正交信号脉冲,自动控制CNT自增或自减从而知识编码器的位置,旋转方向和旋转速度

每个高级定时器和通用定时器都拥有1个编码器接口

两个输入引脚借用了输入捕获的通道1和通道2



正交编码器

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjVkOWU1NWQwNDVlYTQyYTlhNmQ1YWJkMWY1MDRhOGFfMjIyYzY1NDY3YWMxODY0N2Y3MmI3YTI5M2NiNmU5Y2RfSUQ6NzUzODQxNTM4OTAyMzU2Nzg3NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





编码器接口基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2Y3MDY5ZjE5ZmJlMTQwMjhjNDA3ZjhkZTIzOThhYTNfOGE3MWQzNzI2ZDI1Yzc4Zjk3NzQwYTgzNGEyMzFmYmRfSUQ6NzUzODQxODczOTIyNDU0MzI1MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 工作模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzBmOTY2NGU3OThhNzg4Nzg5YWY2NGRjNzc5MTE1NGZfNWU3ZmE3ZjE4NDRjYjgyZWI1NmQ0NWM5OTVhZGIwOTdfSUQ6NzUzODQxOTE3MzAxNDMwNjgyMF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

## ADC简介

ADC模拟\-数字转换器

ADC可以将引脚上连续变化的模拟电压转换为内存中存储的数字量,建立模拟电路到数字电路的桥梁

12位逐次逼近型ADC,1us转换时间

输入电压范围:0\~3\.3V，转换结果范围:0\~4095

18个输入通道,可测量16个外部和2个内部信号源

规则组和注入组两个转换单元

模拟看门狗自动 检测输入电压范围

STMF103C8T6 AD 资源: ADC1,ADC2,10个外部输入通道



## 逐次逼近型ADC

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmM3MTU1MjQ5MmRkMTljNGI1NDhlOWQ5NzBiMDQ1MjVfMTc5YmVjNjgxMjBkZGYxMmU5NjI0NWMyNzkwOWRlYjlfSUQ6NzUzODk4NTgwNzcxOTMwMTEyMl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## ADC基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Mzk4ZmNmODExNGI0MDRhZjIyYmExMGY1MDA5YTJjYjlfNWVkZTlkOGY1Nzk2ZDJiMzg2OWJkMGYwYzRlMmZhZDZfSUQ6NzUzODk4OTc4MTQ2MTM2ODg2MF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 触发控制

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2Q5MmNhMWM3NWIzNmRjM2ZhNjY5ZTlkZTlmODY3MmVfYjk1M2MzMzhjZWI0NGY3YWE1NzkzYmQ0YWM2Y2NhYmZfSUQ6NzUzODk5NDMyMDg4OTA4NTk1Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 数据对齐

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTdiNDZjYmI0MTg4Y2U5OGM0MzVkY2I1MWY4NjA0NGZfMTFjYTVjODI3ZDU2NmUyZjBlNGJjOWNlZmNlNTNkYmJfSUQ6NzUzODk5NDYyNDkwOTMxMjAyOF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 转换时间

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTExMjZjMDNiNmY3MjBhZDY1ZGRlZjdmYzYxMDkwZWRfN2FjNTMzZWIwZDBhYzExYmRlNjM2MmU2YTVhNjZjYTRfSUQ6NzUzODk5NTIxNDg3OTU4ODM3MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 校准

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjliZWQwZjIwMmIyMzUxYWRlYzcwMTYyYzhmZWVlODdfMmRjYmU1MjViNjJlOGI0NjRhMWQwM2ExM2M3ZWMzYzdfSUQ6NzUzODk5NjMyMTk5Mjk4MjU0N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## DMA\(Direct Memory Access\)直接存储器存取

DMA可以提供外设和存储器或者存储器和存储器之间的高速数据传输,无须COU干预,节省了CPU资源

12个独立可配置的通道:DMA1\(7个通道\)，DMA2\(5个通道\)

每个通道都支持软件触发和特定的硬件触发

每个通道都支持软件触发和特定的硬件触发

STM32F103C8T6 DMA资源:DMA1\(7个通道\)





## 存储器映像

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2M4MTMyMWVlNThhNjQ4N2UzODMxMDdmNzU2N2Y1Y2FfNGI5NTMwODExN2M0OTg2NTE3OTlkY2RmNWZkOTBkZDlfSUQ6NzUzOTIyMDg3MDI4NDQ3NjQzNV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## DMA基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWY3MWJlNThlNGM4Zjk4YTc4N2M0NWQyMGE3ZWMzNDlfNWVkMzVlNjQ5NDU2NjI0ZWFkZDE3YzU4ODRhOGU5MDRfSUQ6NzUzOTIyNDc1Nzk4ODkwMDg2NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



DMA转运的3个条件

1\.传输计数器大于0

2\.触发源有触发信号

3\.DMA使能





## 通信接口

通信的目的: 将一个设备的数据传送到另一个设备,扩展硬件系统

通信协议: 指定通信的规则,通信双方按照协议规则进行数据收发

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjNhZmM3ZTQyMTMxMjk4MWJjMDA3ZmNkNzAwYWRhYWNfNTdkODFlZGVlZTEyM2JmMzFhNzIyNTRlM2M2ZDE4NmNfSUQ6NzU0MDkyODc2NDMzNjA5NTI1MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 串口通信

串口是一种应用十分广泛的通讯接口,串口成本低,容易使用,通信线路简单,可实现两个设备的互相通信

单片机的串口可以使单片机与单片机,单片机与电脑,单片机与各式各样的模块互相通信,极大地扩展了单片机的应用范围,增强了单片机系统的硬件实力



## 串口时序

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2Q0MjM4MDhlNGJjMGM5ZjNiZTA2MTliZDE5NDYzODNfNjZjNzlkMTY5M2Q5YTFjNTk1OGRhM2U4OTdhMTgzN2NfSUQ6NzU0MTA1MTY0MDY3NDUzMzM3N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## USART简介

USART 通用同步/异步收发器

USART是STM32内部集成的硬件外设,可根据数据 寄存器的一个字节数据 自动生成数据帧时序,从TX引脚发送出去,也可自动接收RX引脚的数据帧时序,拼接为一个字节数据,存放再数据寄存器里

自带波特率发生器,最高达4\.5Mbits/s

可配置数据位长度\(8/9\)，停止位长度\(0\.5/1/1\.5/2\)

可选校验位\(无校验\.奇校验/偶校验\)

支持同步模式,硬件流控制,DMA，智能卡,IrDA,LIN

STM32F103C8T6 USART资源 : USART1,USART2,USART3



## 串口硬件电路

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MjE3Y2JjNGU5ZWViNTQyMjVkMzUxMzJkMDY0NGNiOWZfOWJhOWEwZDhmZTI2NGY2NGJhOTdmNGQ3NTYwNzZlZGJfSUQ6NzU0MTA1NDA2Nzg4NDQ1Nzk4OF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

简单双向串口通信有两根通信线\(发送端TX和接收端RX\)

TX与RX交叉连接

当只需要单向的数据传输时,可以只接一根通信线

当电平标准不一致时,需要加电平转换芯片



## 电平标准

电平标准是数据1和数据0的表达方式,是传输线缆中人为规定的电压与数据的对应关系，串口常用的电平标准有如下三种:

TTL电平: \+3\.3V或\+5V表示1，0V表示0

RS232电平: \-3\~\-15V表示1， \+3\~\+15V表示0

RS485电平: 两线压差\+2\~\+6V 表示1，\-2\~\-6V表示0\(差分信号\)



## 串口参数及时序

波特率: 串口通信的速率

起始位: 标志一个数据帧的开始,固定为低电平

数据为: 数据帧的有效载荷,1为高电平,0为低电平,低位先行

校验位: 用于数据验证,根据数据位计算得来

停止位: 用于数据帧间隔,固定位高电平

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTBjZWQ2MDJmODdjNDllYWUyZTkwZjk3YmIzNzk5ZGZfNzkwOTAzNDQwZGZmNDY3NmQwMmIwMTQ0NWIyOWFmNWNfSUQ6NzU0MTA1NzQzMDUwODE1ODk3OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## USART简介

USART\(Universal Synchronous/Asynchronous Receiver/Transmitter\) 通用同步/异步收发器

USART是stm32内部集成的硬件外设,可根据数据寄存器的一个字节数据自动生成数据帧时序,从TX引脚发送出去,也可自动接收RX引脚的数据帧时序,拼接为一个字节数据,存放在数据寄存器里

自带波特率发生器,最高达4\.5Mbits/s

可配置数据位长度\(8/9\),停止位长度\(0\.5/1/1\.5/2\)

可选校验位\(无校验位/奇校验位/偶校验\)

支持同步模式,硬件流控制,DMA，智能卡,IrDA,LIN

STM32F103C8T6 USART资源： USART1,USART2,USART3



## USART基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTAyMGQwOGIwNDk0ODkyMjVlNzdkMWMwZjc3Y2Y2OWVfYWQ3YzE2ZmVhZDg3NjhiZDk3ZWE2ZTAxMjY4YWRmZDBfSUQ6NzU0MjcxMjMwNjUzNjM4MjQ2NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 数据帧

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MGFlZGM2NjBiNDllMzJjMmYwZmYyNmZkMDFiY2ZkMTlfMWFlZTYxNmJmODQ0ZTQxY2Y1ZGQxNGU2ZTdlZWM3MTdfSUQ6NzU0MjcxMjg3NjAzODYwMjc1NF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 波特率发生器

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjljZDUyNDcxZjZkZTk3M2M4ZmZjMjg3YTYzNTk1ODNfMjE2NmQxNGQwZjNlMWM1NWUzMWE1ODUxMzgyM2RkYmZfSUQ6NzU0MjcxNTUyNTE5ODQ3OTM2NF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 数据模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTUzNjNmZWJkYTM2NTFmZmNlMjc5ODdjMjMwMmUwNDdfMTEyMjhjZmFmNjVhNTg2OGViZmQ3OTVlYzIyYWQ0YWZfSUQ6NzU0MzQ0MzExODQzMzU1MDM1NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## C语言整数类型详解

固定宽度类型

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTNhYTA2YzJjMDI0MDI5OTFmMzIwMjVlMjU3YjM0ZTRfZjVkYmYwMWU0YTI3NmI2NzdlNzBkMWViOWVjOGJmMTlfSUQ6NzU0MzQ0ODU5NTk2NDY0MTI5OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



传统C语言类型

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDc0ZjRhNzg3NTlmNzVjZTczNWUyNmJlYzk1Njc1YjVfOTdiNGNmZGQ3ZjkwOWYwZGNiZGIxNjE4NDc1N2E3M2VfSUQ6NzU0MzQ0ODg1MDQ0OTIwMzIwMV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 转义字符

\\r回车键

\\n换行键

stm32串口打印换行这两个转义字符都需要



## C语言可变参数

允许函数接受不定数量的参数,最典型的例子就是printf\(\)和scanf\(\)函数

基本原理:

va\_list\-用于声明一个变量,该变量将引用参数列表

va\_start\-初始化va\_list变量,使其指向可变参数列表的第一个参数

va\_arg\-访问参数列表哦中的下一个参数

va\_end\(\)\-清理va\_list变量



可变参数函数的声明需要在参数列表中使用省略号\(\.\.\.\)



优点:提供了极大的灵活性,可以创建接受不定数量参数的函数

缺点:缺乏类型安全检查,容易导致运行时错误





## HEX数据包

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2VkZmYzNWY3ZWQ4MmYzNTc2NjNjNDQxNzE4NTFjMGZfMzZhMmQ1OGE4NGZiNGQyZjA1MGFlMmRmZDRiMTVmN2VfSUQ6NzU0Mzg3MTAwOTE2NjgwMjk0NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

问题一:包头包尾和数据载荷重复

解决方法:1\.限制载荷数据的范围

2\.如果无法限制包头包尾重复,就尽量 使用固定长度 的数据包

3\.增加包头包尾的数量,并且尽量呈现出载荷数据出现不了的状态

问题二:包头包尾并不是 全部都需要的，

1\.例如可以只需要包头,受够四个标志位结束,但是这样载荷和包头重复的问题会更           严重一些

问题三:固定包长和可变包长的选择问题

1\.如果你的载荷会出现和包头包尾重复的情况,那就最好选择固定包长,这样可以避               免接收错误

2\.如果载荷不会和包头包尾重复可以选择可变包长

问题四:各种数据转换位字节流的问题

1\.用一个uint8\_t的指针指向它，把他们当作一个字节数组发送就行了





## 文本数据包接收

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTIyODQ4NTk5NDk0ZGFmMzFjNmIzYWVlYzExZjQ1N2VfYWQ0MzE4NDAyZTA1MWM3MDNmZWIwZDY1ZDFhZWRmZjVfSUQ6NzU0Mzg3MjIzNTg3NDI1NDg2N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

数据通过编码和译码的形式进行传输,本质还是字节传输

纯文本数据包通常不担心包头,包尾与数据内容重复的问题,因为他们适应分隔符来标记字段和记录 边界,比如空格，逗号,换行符





## HEX数据包接收

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmZlYTk1ZmQ0NWIyYzRmMWFjYWNkNmIwMzZiYjgzOGFfZWFlNTcwYThkNTI4ZmExZDI4MzkxY2E5ZTBlN2JlMzlfSUQ6NzU0Mzg3MjM5MDk2MTU5NDM4N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## HEX数据包和文本数据包的优缺点

HEX数据包:

优点:

1\.极高的传输效率

2\.极快的解析速度

3\.结构紧凑且明确

缺点:

1\.极度不直观

2\.缺乏灵活性

3\.存在平台兼容性问题

4\.需要复杂的状态机处理



文本数据包:

优点:

1\.人类刻度,极易调试

2\.天然跨平台兼容

3\.非常灵活

4\.实现简单

缺点:

1\.传输效率极低

2\.解析开销大,速度慢

3\.数据精度可能丢失

4\.仍需处理边界问题







## 上位机和下位机

下位机就像​“四肢”和“感官”​​：它直接接触物理世界，负责执行具体的动作（如控制电机转动、点亮LED灯）和采集数据（如读取温度、检测按键）



上位机就像​“大脑”​​：它不直接干活，而是负责指挥、决策和显示。它接收下位机传来的数据，进行分析、计算、存储，并以图形化界面（GUI）等方式展示给用户，同时用户也可以通过它向下位机发送控制命令。



## 状态机

一个能记住不同状态的机制,在不同状态执行不同的操作,同时还要进行状态的合理转移

1\.先根据项目要求定义状态,画几个圈

2\.然后考虑好各个状态在什么情况下会进行转移,如何转移，画好线和转移条件

3\.最后根据画好的 图来进行编程



## 使用杜邦线代替按键

按键的本质是控制单片机GPIO引脚在高电平和低电平之间切换\.我们用杜邦线手动练剑不同的线路,就可以模拟这个动作



关键:用一根杜邦线,将已经配置为上拉输入模式的GPIO引脚瞬间与GND\(低电平\)短接





## 同步和异步

同步:

操作按顺序执行,必须等待前一个操作完成后才能开始下一个操作

异步:

操作发起后立即返回,不等待结果,通过回调/事件/中断等方式在完成后通知



## 中断和阻塞的区别

中断:

中断是一种由硬件（或特定软件指令）触发的信号，它迫使CPU暂停当前正在执行的指令序列，转而去执行一个特定的、被称为中断处理程序的函数，处理完毕后再返回原来的地方继续执行。

来源:    由硬件设备产生

异步性: 中断的发生是不可预测的,它可以在指令执行的任何时刻发生,与CPU当前正在执行               的代码无关

目的:     为了响应外部事件,提高CPU和硬件的利用率

上下文:  中断处理程序运行在中断上下文中\.这是一个非常特殊的环境,不能休眠,不能调用可              能引起的阻塞的函数\(因为它是打断正常流程的,没有进程概念\)

层级:      属于底层机制,是操作系统得以运行和设备管理的基石



阻塞:

阻塞是指一个进程或线程因为等待某个事件（如资源可用、I/O操作完成）而主动停止执行，并让出CPU的行为。操作系统会将其状态置为“阻塞态”，并调度其他就绪的进程来运行。

来源:      由正在运行的进程/线程自己通过系统调用发起的

同步性:   可预见的,主动的

目的:      为了高效的等待\.与其进程占着CPU空转,不如让它睡觉,把CPU让给其他需要的进程

上下文:   阻塞发生在进程上下文中\.这是程序正常的执行环境

层级:      高层抽象,是应用程序开发中常见的概念,用于多任务和资源共享 







## 进程和线程的区别

进程:

基本定义:    资源分配的基本单元

资源拥有:    拥有独立的地址空间和系统资源

独立性:       独立性高\.一个进程崩溃后,在保护模式下不会影响其他进程

开销:           创建,销毁,切换开销大\.需要为它分配独立的系统资源

通信机制:     复杂,需要IPV机制:如管道,消息队列,共享内存,套接字

包含关系:     一个进程可以包含多个线程



线程:

基本定义:    CPU调度的基本单元

资源拥有:    共享其所属进程的地址空间和资源

独立性:       独立性低,一个线程崩溃会导致整个进程崩溃,从而影响同进程下的所有其他线程

开销:          创建,销毁,切换开销小,因为它们共享资源,只需保存少量寄存器装填和栈空间

通信机制:    进程间通信非常简单,因为它们共享全局变量,静态变量等内存空间,但需要同步                    机制\(互斥锁,信号量\)来避免冲突

性能影响:    成本低,创建速度块,资源占用小,能极大提高程序的并发性能,但需要谨慎处理同                  步问题,否则易产生bug\(如死锁\)

包含关系:    线程必须依赖于进程而存在,它是进程的一部分



## 启动配置

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTBiMGYxZjIxNzY4YTljYjQ4YmYyNWY3OWViZWJkM2RfZTdjYzA0OWJlOTUzYjZjMWI0NzE1NGI3YmVkMjM5MjdfSUQ6NzU0NDY3MTMzNjIxNzEwMDI4OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 串口下载程序

1\.硬件连接

2\.配置Boot模式

要让芯片上电后不运行用户程序,而是直接进入内置的Bootloader,需要通过Boot引脚\(BOOT0和BOOT1\)来设置启动模式

\(BOOT0引脚接高电平,BOOT1引脚接低电平\)

3\.进入BootLoader模式

4\.使用软件烧录

5\.恢复正常启动模式

\(烧录完成后,先点击Disconnect断开连接,给目标板断电,将BOOT0引脚改回接低电平,重新上电,此时STN32将从主闪存Flash启动,运行你刚刚 烧录进去的新程序\)



STM32一键下载电路

核心思想:利用USB转串口芯片的两个硬件流控制信号:

RTS和DTS

通过软件控制这两个信号的电平变化,来模拟我们手动操作复位\(RESET\)和BooT模式\(BOOT0\)的动作





## 复位

内容目前略

本质:强制将芯片内部几乎所有的重要寄存器和状态恢复到芯片设计时规定的默认值





## ROM和RAM

ROM\(只读存储器\)

非易失性:断电后,所有存储的内容都不会丢失

通常用于存储:需要永久或长期保存的数据

RAM\(随机存取存储器\)

易失性:断电后,里面的数据会立即丢失

通常用于存储:需要高速访问的临时数据





## 选项字节

物理位置:它是芯片内部主Flash存储器中的一个特殊部分

内容:它由多对\(互补的\)16位字组成,用于提高可靠性\.每个配置项都有对应的互补项,如两者不匹配,则意味着选项字节错误或未编程

特性:非易失性\.一旦通过特殊工具烧写进去,断电后配置也不会丢失,下次上电自动生效



配置内容:

1\.读写保护

2\.写保护

3\.复位模式

4\.看门狗

5\.启动配置





## I2C通信

I2C总线\(Inter IC BUS\)是由Philips公司开发的一种通用数据总线

两根通信线:SCL\(Serial Clock\),SDA\(Serial Data\)

同步,半双工

带数据应答

支持总线挂载多设备\(一主多从,多主多从\)



串口通信的硬件电路: 

简单双向串口通信有两个通信线\(发送端TX和接收端RX\)

TX和RX要交叉连接

当只需要单向的数据传输时,可以只接一根通信线

当电平标准不一致时,需要加电平转换芯片

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTJlYzE5ZjllYTA1YWM3ZDRjMzg5MDJkNGFmYTIxNjZfMDc0NWE1MjVjZDNkNzdkMGEwN2FiMTRiODc5ODI2ZDNfSUQ6NzU3ODc5MTkwNjE2NTk1MTQ0Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



实现了读写寄存器就实现了对外挂设备的完全控制



要求1: 删掉一根通信线,只能在同一根线上进行发送和接收,全双工 \-\> 半双工

要求2: 增加应答机制

要求3: 一根线上能同时接多个模块

要求4: 异步 \-\> 同步, 加一条时钟线



硬件电路: 

所有I2C设备的SCL连在一起,SDA连在一起

设备的SCL和SDA均配置成开漏输出模式

SCL和SDA各添加一个上拉电阻,阻值一般为4\.7k欧姆左右



I2C时序基本单元

起始条件: SCL高电平期间,SDA从高电平切换到低电平

终止条件: SCL高电平期间，SDA从低电平切换到高电平

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzBjMWNmNjQzMDMzMzZiNDRkYjhmMWQyMzJhNTc1YWJfNmRhMmZmYjkwNDU1ZjQ4ODFiOGY4NWZmN2FkM2RmMDdfSUQ6NzU3ODc5ODc0NzkyOTgwODA4OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



发送一个字节: SCL低电平期间,主机将数据位依次放到SDA线上\(高位先行\)\.然后释放SCL,从机将在SCL高电平期间读取数据位,所以SCL高电平期间SDA不允许有数据变化,依次循环上述过程8次,即可发送一个字节

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjhhZGIxZGYwZWIzMzA4NjYwNGYyYjI2MzIxYmIwODFfNjVlNGZhYzAyNDY4OGVhMGU3ODUxOGRlMTk4NmU4OTRfSUQ6NzU3ODgwMDcwNjkwNDMyOTE3MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



接收一个字节: SCL低电平期间,从机将数据位依次放到SDA线上\(高位先行\),然后释放SCL，主机将在SCL高电平期间读取数据位,所以SCL高电平期间SDA不允许有数据变化,依次循环上述过程8次,即可接收一个字节\(主机在接收之前,需要释放SDA\)



发送应答: 主机在接收完一个字节之后,在下一个时钟发送一位数据,数据0表示应答,数据1表示非应答

接收应答: 主机在发送完一个字节之后,在下一个时钟接收一位数据,判断从机是否应答,数据0表示应答,数据1表示非应答\(主机在接收之前,需要释放SDA\)



指定地址写

对于指定设备\(Slave Address\), 在指定地址\(Reg Address\)下,写入指定数据\(Data\)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmE2MmE2ODdmYTQyM2RlZWQ2MDQyODc1ZjQ0NmMzNTlfZGQyYTY1NDY2Nzc4YmNjMjBmZmEwZjdiNWQ0MWMzMGVfSUQ6NzU3ODgwNzM3Njk3NTg1ODg5MF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



当前地址读: 

对于指定设备\(Slave Address\),在当前地址指针指示的地址下,读取从机数据\(Data\)



指定地址读: 

对于指定设备\(Slave Address\)，在指定地址\(Reg Address\)下,读取从机数据\(Data\)



## MPU6050简介

MPU6050是一个6轴姿态传感器,可以测量芯片自身X,Y,Z轴的加速度,角速度,通过数据融合,可进一步得到姿态角,常应用与平衡车,飞行器等需要自身姿态的场景

3轴加速度计\(Accelerometer\): 测量X,Y,Z轴的加速度

3轴陀螺仪传感器\(Gyroscope\): 测量X,Y,Z轴的角速度

补充: 

3轴的磁场传感器

1轴的气压传感器

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTYxMzAxNTRjZjhhN2ZiY2E3NDQ2NjY0ZGRmYmNlMTFfNjhkMzY4NzU5MTcyMmRkMjhjYWM5N2YzMTRiMjVmYzRfSUQ6NzU3ODgxMTE2NjY4MjE3MjYxNF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

加速度计具有静态稳定性,不具有动态稳定性

角速度积分 \-\> 角度\(但是不会受物体运动的影响\) \-\> 动态稳定,静态不稳定

加速度积分 \-\> 速度 \-\> 静态稳定,动态不稳定



零漂: 物体静止时,角速度值会因为噪声无法完全归零,经过积分的不断累积,这个小噪声就会导致计算出来的角度产生缓慢的漂移\(角速度积分得发哦的角度经不起时间的考验\) 



角速度积分和加速度积分取长补短进行互补滤波，就能融合得到静态和动态都稳定的姿态角



MPU6050参数

16位ADC采集传感器的模拟信号,量化范围: \-32768\~32767

加速度计满量程选择: \+\-2, \+\-4, \+\-8, \+\-16\(g\)

陀螺仪满量程选择: \+\- 250 , \+\- 500, \+\-1000, \+\-2000 \(度/sec\)

可配置的数字低通滤波器

可配置的时钟源

可配置的采样频率



I2C从机地址: 1101000 \(AD0=0\)

1101001 \(AD0=1\)



硬件电路

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MmRhMTNjZTVmNDhjOTQxNjExMmEzNjY0MGJiN2RmYzlfNjkxM2I2MGFjZmUyN2VlMzcxYTI5Y2IyMzdmMzFkNzBfSUQ6NzU4MTM5MDk5OTA4NTgxMjk0OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 复位

复位\(Reset\)是将STM32微控制器的内部状态强制恢复到一个已知的,确定的初始状态的过程,就像电脑的重启,但更彻底\-\-\-\-\-不仅软件重启,硬件状态也重置



复位时到底发生了什么? 

1\.时钟系统重置: 

HSI（内部高速时钟）称为默认系统时钟

PLL,HSE（外部高速时钟）等时钟源被禁用

系统时钟\(SYSCLK\)切换到HSI

AHB，APB总线时钟分频器重置位默认值



2\.CPU核心重置

\(1\)程序计数器\(PC\)被设置为复位向量地址

\(2\)CPU寄存器

\(3\)中断系统重置



3\.外设重置

\(1\)大多数外设寄存器恢复默认值

\(2\)DMA控制器停止,通道禁用

\(3\)所有挂起的终端标志被清除



```Java
/* 硬件自动完成 */
1. 从复位向量获取栈指针（SP） → 设置MSP
2. 从复位向量+4获取复位处理函数地址 → 设置PC

/* 启动文件（startup_stm32fxxx.s） */
3. 执行Reset_Handler:
   a. 调用SystemInit()          // 时钟、Flash等待周期等初始化
   b. 复制.data段到RAM          // 初始化全局变量
   c. 清零.bss段                // 清零未初始化全局变量
   d. 设置堆栈边界（可选）
   e. 调用__libc_init_array()   // C++全局对象构造函数
   f. 调用main()               // 用户程序入口

/* 用户程序 */
4. main()函数开始执行
   a. HAL_Init()              // HAL库初始化
   b. SystemClock_Config()    // 配置系统时钟
   c. 外设初始化（GPIO、USART等）
   d. while(1)主循环
```



## I2C外设简介

STM32内部集成了硬件I2C收发电路,可以由硬件自动执行时钟生成,起始终止条件生成,应答位收发,数据收发等功能,减轻CPU的负担

支持多主机模型

支持7位/10位地址模式

支持不同的通讯速度,标准速度\(高达100kHz\),快速\(高达400kHz\)

支持DMA

兼容SMBus协议

STM32F103C8T6硬件I2Cz资源: I2C1,I2C2





## STM32寄存器: CR,DR,SR寄存器

1. CR寄存器 \- 控制器\(Control Register\)

用于配置和控制外设哦的工作模式,参数和行为



常见CR寄存位含义: 

EN 外设使能

TE 发送使能

RE 接收使能

MODE 工作模式

PS/PH 分频系数 



2. DR寄存器 \- 数据寄存器\(Data Register\)

用于存放要发送或接收的数据



3. SR寄存器 \- 状态寄存器\(Status Register\)

反映外设的当前工作状态和事件标志      何时置1

TXE 发送缓冲区空                               发送缓冲区可以接收新数据时

TC 发送完成                                       最后一个数据发送完成时

RXNE 接收缓冲区非空                          接收到新数据时

ORE 过载错误                                     新数据到来时旧数据未读取

FE 帧错误                                           接收到的帧格式错误 

RE 奇偶校验错误                                  奇偶校验失败





## I2C框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTQ1NDIwYTQ0ODIxM2I2YWJjZjc2NDBjZGY1ZWQ0MGVfYzM0MTM4ZTNjZTEwMzk4YjIzMmFlNWQ3Zjg4M2M1OWFfSUQ6NzU5NzYyNTUxNTg4NTUxMzY4OF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## I2C基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGZkMWU4NWQ5ZTFlODI2OGI0MzE5NGQ3MzNiYTk3ZjBfOGM3NTY2ZDM3YTljZTMxYzhkNjhhYjRhMzVjZjBjYTdfSUQ6NzU5NzYyNzExMzA1NTQ4ODk3OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 主机发送

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzY3YjViOGIwMWI4MmVmNzQ5ZWQ4ZDc5Y2IzYWI2OGFfZWI5MThlMTE1ODE3ZDcyMjAyOTAyYWJkMmRjODFkOWVfSUQ6NzU5NzYyOTk5NTExNjk3MzI1OF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 主机接收

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWY4OWZkZWYwOWM0MzgyOTdkMjI1ZTQ3ODZlZDM3M2RfOWFjZTAwOTRhYWU5ZGNmYzg2YWQ2MDQ5NjU5YTI3MGVfSUQ6NzU5NzYzMDEzMjM5MTk1NTQxNl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## SPI通信

SPI\(Serial Peripheral Interface\) 是由Motorola公司开发的一种通用数据总线

四根通信线: SCK\(Serial Clock\) , MOSI\(Master Output Slave Input\) , MISO\(Master Input Slave Output\) , SS\(Slave Select\)

同步,全双工

支持 总线挂在多设备\(一主多从\)



## SPI硬件电路

所有的SPI设备的SCK,MOSI,MISO分别连在一起

主机另外引出多条SS控制线,分别连接到各从机的SS引脚

输出引脚配置为推挽输出,输入引脚配置为浮空或上拉输入

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjM0MTAzZDk2NDQxY2MzNjk3NzgyMzRhZGNmODMyNTZfYTFhMTRkZmJlZGVlMzlkMGMxNGZhYTE4NTM1NmVhNTJfSUQ6NzU5Nzc0NzgwMDc2MzY4MTczN18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## 移位示意图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YTFlOGYwYzE4YzY4ZmNlMmNmZWI3MzIzMjNjZWNiNmJfNGI1YjRiYzI1ZjMwZTQxNTM5NGY3ZWM0MjgwOTNmMjhfSUQ6NzU5ODAxMDg5OTE1Mjk0ODE2M18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## SPI时序基本单元

起始条件: SS从高电平切换到低电平

终止条件: SS从低电平切换到高电平

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGMyOTNhZWJkOGY2OWZkMGNjMTIzNjRjYmUyOTlmMGVfMTg5MmNmNzI0NTkwY2MwNGJlYzBhMzYxYjlmMTE3ZGVfSUQ6NzU5ODAxMzI4MzU2ODgzMTQzM18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 模式0

CPOL：时钟极性

表示时钟信号在空闲状态的电平



CPHA：时钟相位

表示数据采样的时钟沿



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmZlNjM0YzI3ZDU2YWRjMTM1ZmRjYTg3ZDQ3MmE4YThfNWZkNzhlN2RhZThhYWRlYzcyNWViOTJiMDkyNTg0YjBfSUQ6NzU5ODAxNzAzNjE1MzIxMTg1OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## W25Q64简介

W25Qxx系列是一种低成本,小型化,使用简单的非易失性存储器,常应用于数据存储,字库存储,固件程序存储等场景

存储介质: Nor Flash\(闪存\)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2EzNTUyZDZhZWYxMDkxYTlmMTUzYWQxMTEyMzNjNjJfZGIwMDE5YjRlMDYwNmM1NDAyZGM2OTZiMjE0YjBiNjlfSUQ6NzU5ODAyNzM1MzgyMjM1MDU2MF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## Flash操作注意事项

写入操作时: 

写入操作前,必须先进行写使能

每个数据位只用由1改写为0，不能由0改写为1

写入数据前必须先擦除,擦除后,所有数据位变为1

擦除必须按最小擦除单元进行

连续写入多字节时,最多写入一页的数据,超过页尾位置的数据,会回到页首覆盖写入

写入操作结束后,芯片进入忙状态,不响应新的读写操作

读取操作时: 

直接调用读取时序,无需使能,无需额外操作,没有页限制

读取操作结束后不会进入忙状态,但不能在忙状态时读取







## SPI外设简介

STM32内部集成了硬件SPI收发电路,可以由硬件自动执行时钟生成,数据收发等功能,减轻CPU的负担

可配置8位/16位数据帧,高位先行/低位先行

时钟频率: fpclk\(2,4,8,16,32,64,128\)

支持多主机模型,主或从操作

可精简为半双工/单工通信

支持DMA

兼容I2S协议



STM32F103C8T6硬件SPI资源: SPI1,SPI2





## SPI框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGE0NWMxZjAzNGMzNWZmNzA2Mzg5NGU0MzJjNDkxZmFfOGU1YWM1YjRiZjMzZDNkYzE4NDYzM2M5Nzk5NzliMmNfSUQ6NzU5ODQ3NDM5NTU5ODE3OTU1M18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## SPI框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTUwODU3ZjQxYzFiMjVhNzE0MTczMzgwNzM3MmNhZWJfYTk4NTc4ZDlhNjUxMzcwOWE3MmYyY2EzNzRlMTU1MGJfSUQ6NzU5ODQ3NDU2MzU4ODUwODYzNl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



### 非连续传输

每个数据传输单元之间都有间隔

片选信号在每个字节传输后拉高再拉低

时钟信号在每个字节传输后停止



速度: 较慢

功耗: 较高

时序要求: 宽松

抗干扰: 较好



### 连续传输

多个数据传输单元连续发送,无间隔

片选信号在整个传输期间保持有效

时钟信号连续产生,数据流不间断



速度: 快

功耗: 较低

时序要求: 严格

抗干扰: 较差







## Unix时间戳

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzE5OGIxNmQ3NmY4MzQ2OGI4MTA4MjAyMjY4ZWY2MTdfMTc4ZWZlNDg0ODNjNDc5MjA3MmZkODhiYTc2ZWU3ZWVfSUQ6NzU5ODc1MzQ1NTE0MjU3MDk1N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## UTC/GMT

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NjYxODIxYmU2MmNkNGEyM2I5YTY1MTFlZWViZGQzNjhfNmQ4MTkwZTJlYmJlN2M3MzMzNTc5NmIxOTE4Yzk4OGVfSUQ6NzU5ODc1NDk3OTU5NTc4MzM5Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 时间戳转换

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTZhMDBhMDA3OTNhYTZhZGRkMjRhNzA0MTU4MmZjMjhfYzVlNGYyOWE3MjRlMDVhOWJkNDYwYTA1Mjg2YTcyN2JfSUQ6NzU5ODc1NTA3NTY3NDY3MjA1OF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmRmZmMyZTRjYmRlZjQ1OGNkYzRmZTQ2MDdkMzc4MmFfNTMyYmI5MzNmNTQ1OWMxMGIxNzY4OTA3YmY5NDljZWVfSUQ6NzU5ODc1NTIzNjIxMTYyNTE1NF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## BKP简介

BKP\(Backip Registers\)备份寄存器

BKP可用于存储用户应用程序数据\.当VDD\(2\.0\~3\.6\)电源被切断，他们仍然由VBAT\(1\.8V\~3\.6V\)维持供电。当系统在待机模式下被唤醒,或系统复位或电源复位,他们也不会被复位

TAMPER引脚产生的侵入事件将所有备份寄存器内容清除

RTC引脚输出RTC校准时钟,RTC闹钟脉冲或秒脉冲

存储RTC时钟校准寄存器

用户数据存储容量: 

20字节\(中容量和小容量\) / 84字节 \(大容量和互连型\)



## BKP基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDMxYTI4YjJhYTZmNDg3MzBkNjJiNGMzMGQ3ODBhZmFfNjFmN2Q3YWEwMWI0NzM5NmUzMzJiZjcyMWE5ZGRmZjJfSUQ6NzU5OTEyNjczMzc3ODAxMzQwOF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## RTC简介

RTC是一个独立的定时器,可为系统提供时钟和日历的功能

RTC和时钟配置系统处于后备区域,系统复位时数据不清零,VDD\(2\.0\~3\.6V\)断电后可借助VBAT\(1\.8V\~3\.6V\)供电继续走时

32位的可编程计数器,可对应Unix时间戳的秒计数器

20位的可编程预分频器，可适配不同频率的输入时钟



可选择三种RTC时钟源

HES时钟除以128\(通常为8MHz/128\)

LSE振荡器时钟\(通常为32\.768KHz\)

LSI振荡器时钟\(40KHz\)





## RTC框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTNjMjdmYmFmZGFjYzMwOWI0NGY4M2ZkNjQ5YmEwZjFfZjZiZWE3NWIzYmU4MDVkNTg1NDA1ZDBkZGU5NWU0ODJfSUQ6NzU5OTEzMTI5ODkyOTc3Mzc4NV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## RTC基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZTMxY2YxMTI0Y2I4MzI1MWQ2YWQ2NzFkYjc3ZDUwZWFfMzQyYmJjNjY5MDlhNjY1ZWU0ZWFjZjYyYjdlMTA5ZWZfSUQ6NzU5OTI0MzI1NDExODUwMTU2Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## RTC操作注意事项

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWUzZTg2YmEyMDgyMTVhYzdiMjZmZGRjYzRlZmM3YzhfNjY5ZTAzZTQ3ZWU1MjRiN2FlOTdkZjk0NWY5ZTZjMDRfSUQ6NzU5OTI0NDU4MDg3ODI1NzMzMl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## PWR简介

PWR\(Power Control\) 电源控制

PWR负管理STM32内部的电源供电部分,可以实现可编程电压检测器和低功耗模式的功能

可编程电压检测器\(PVD\)可以监控VDD电源电压,当VDD下降到PVD阈值以下或上升到PVD阈值之上时,PVD会触发终端,用于执行紧急关闭任务

低功耗模式包括睡眠模式\(Sleep\),停机模式\(Stop\)和待机模式\(Standby\),降低STM32的功耗,延长设备使用时间









## 电源框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=M2Y3Yjg3ZGIyZTVmZGQwNDcwMjRhNjZkYjkxYjU3NjVfZmZhMDM4NjIwMjFjOWE1YzRhMDc4OGQwMmQ1MTUxNjhfSUQ6NzU5OTQ4Mzg3MjY2OTk0NDc2MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 低功耗模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YWQzZDY4NGJlYjc4YTUwYzA1MDAwNTc2ZmRhYWNjNDhfMjI1YjY3NTk0ZDI2MmEwMzI2MzcwZDgwNTcwNmM2YmFfSUQ6NzU5OTQ5MzE0Mzc1NTM5NDI3Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 模式选择

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzUxNjZjMTg2NzBkNmEyNWQ3Njk0N2E1NWM3ZDJhOGNfMWQzOTQ4ZGY0OGVmNzc0ZWEyMWI1MTFjMGY0ZDFkZGJfSUQ6NzU5OTQ5NTQ4NTkwMDgxOTQxNV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 睡眠模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YjQ0NzRkOTM0MzIxMDNhZDAzY2ViMmQyYzE2NmVhYjVfZDk0ZmE5YzUxZjQ3YjQ4NTQxZDIyNWVlMDAxNjgxZTJfSUQ6NzU5OTQ5NjA5MDAxODU0ODk0OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 停止模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MWMzODQyNmExOTRmMjZlYzE2ZGMxYTJlNTc1OWI5MGVfNmEzZjFiZjFlMDhiNTNhZWNmMGNmN2FhMGQ0YTMzYzNfSUQ6NzU5OTQ5NjQ0MTUxODE4MTU2N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 待机模式

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWFkNTlmMDZjNDcwMDIxOTkwNWU0NzkwOTliYjg0OWVfMDRjOTQ4OGMwMDljMmRjYjYzMTA1MzhhZDYxNDA4OTVfSUQ6NzU5OTQ5NzA3NzI4NjYyMDM1N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## RCC时钟树

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzM0ZWYwYzk1MmEyYWRkNTI1ZTMzMmVmMDM2ZWIwNzdfNDQ3ZGQ2NTEzYjlkOTczYTU0ZjE3N2Y2MDIzMWMyYTZfSUQ6NzU5OTUwMTgzOTY0MTg4OTc1MV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## WDG简介

WDG\(Watchdog\)看门狗

看门狗可以监控程序的运行状态,当程序因为设计漏洞,硬件故障,电磁干扰等原因,出现卡死或跑飞现象时,看门狗能及时复位程序,避免程序陷入长时间的罢工状态,保证系统的可靠性和安全性

看门狗本质是一个定时器,当指定时间范围内,程序没有执行喂狗\(重置计数器\)操作时,看门狗硬件电路就会自动产生复位信号

STM32内置两个看门狗

独立看门狗\(IWDG\): 独立工作,对事件精度要求低

窗口看门狗\(WWDG\): 要求看门狗在精确计时窗口起作用



## IWDG框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NWEzYzY0YjlkYTNmYWEzNGVjNjU2Mzg3YzU0NWIwNTlfNTFhYzE5MDlmOGJlMjg1NDIyNzI1N2ZlOWE2OTQzMmNfSUQ6NzU5OTYwMTU3Mzg0NTI1NzQzOF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 定时中断基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzNlYmQ5Y2ZhODI2MDIyMjg2ODY5YWZiYmZhZjA3ZWJfNTc4MGMyNjViMzZhZWFhYWVhYzI1MThmNjIwMzQ2OWVfSUQ6NzU5OTYwMjExMjc5MjM0OTY1Ml8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## IWDG键寄存器

键寄存器本质是控制寄存器,用于控制硬件电路的工作

在可能存在干扰的情况下,一般通过在整个键寄存器写入特定值来

代替控制寄存器写入一位的功能,以降低硬件电路受到干扰的概率



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTNkNTZiYzRkODc0ZjdlMzBmZGVjOGE1Njg0N2Q5MmZfNTFiMDY2NzU0MjUzNjNiYTUyMmZlNTM0ZmE1MmI0ZDVfSUQ6NzU5OTYwNTEzNDA5NTc4MTA0OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## IWDG超时时间

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTY0YWM3N2IxYTk3ODJjOGY1MjZkOWI4NTY1N2FjZWFfOTE5NmE2N2Y4N2Q3ZWI0YTY2MjgwMzMwYjE0M2Q3Y2ZfSUQ6NzU5OTYwNjc2NDE3ODkzNDc0Ml8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## WWDG工作特性

递减计数器T\[6:0\]的值小于0x40时,WWDG产生复位

递减计数器T\[6:0\]在窗口W\[6:0\]外被重新装载时,WWDG产生复位

递减计数器T\[6:0\]等于0x40时可以产生早期唤醒 中断\(EWI\)，用于重装载计数器以避免WWDG复位

定期写入WWDG\_CR寄存器\(喂狗\)以避免WWDG复位



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWQ0M2Y0ZTUzNjk5YjA2ZmNjNzE1OTAxYzkzMWM2MGZfZGEzNTE1N2YyNDljOTQxMDhjNmUyYzIzM2Y2YjZlYmVfSUQ6NzU5OTYwODEyMDkzNTY4MTIzMF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## WWDG框图

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZGEwNWY0NmMwM2U2ZWEyZjkxZDEwMjYzOWQwY2EyMjJfMWZkYWQyNDY4ODYwYTFkNjRkMmRiYmM3YWIxNGVmZmZfSUQ6NzU5OTYwODMzMjI3NDA2MDUxNl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## WWDG超时时间

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YmZjMjQ1YTNmODNiN2Q4NTg0NjlhNTc5MGIyNjA5NTNfZDBlNjQxMDcyNWVjODZmZmQ4YWVjZmRiOGZjOGYzOGRfSUQ6NzU5OTYxNDUyOTc2MDkwNjQ0Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## IWDG和WWDG对比

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=N2M2NTZkOTgyM2RmNTdhNTkzOGIwMTczNzdjMTNjZTlfY2E2YjQ5MjIyNTZmNTJmMmUwMGRlZDZmYTcyY2M2MDRfSUQ6NzU5OTYxNTQwMTYyMjAzMTMxMV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





补充:

预分频器:

作用: 对定时器的输入时钟进行分频,从而改变计数器的计数频率

预分频器决定了计数器跑多快



重装载寄存器：

作用: 定义自动重装值,即计数器计数的周期

重装载寄存器决定了计数器“数到多少为止



状态寄存器：

作用: 指示定时器当前的各种状态,特别是中断事件的状态

状态寄存器是告知程序员“定时器发生了什么情况”的信号灯。



两次喂狗间隔必须大于看门狗计数的最小更新时间: 

在初始化阶段，如果我们在启动看门狗后立即喂狗，然后很快进入主循环，主循环中又很快喂狗，但第一次喂狗可能因为LSI不稳定而没有成功，而第二次喂狗虽然成功，但此时计数器已经递减了很多，导致第二次喂狗和第三次喂狗之间的时间间隔可能超过了看门狗的超时时间（因为从启动到第二次喂狗的时间很长，而第二次喂狗后，计数器重新加载，但程序执行到第三次喂狗的时间间隔可能超过了设定值）。





## FLASH简介

STM32F1系列的FLASH包含程序存储器,系统存储器和选项字节三个部分,通过闪存存储器接口\(外设\)可以对程序存储器和选项字节进行擦除和编程

读写FLASH的用途: 

利用程序存储器的剩余空间来保存掉电不丢失的用户数据

通过在程序中编程\(IAP\),实现程序的自我更新

在线编程,用于更新程序存储器的全部内容,它通过JTAG,SWD协议或系统加载程序\(Bootloader\)下载程序

在程序中编程可以使用微控制器支持的任意种通信接口下载程序





## 存储器映像

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzljMDZhN2UyMGIxYjBlYTZmODY0YThlMDk3MzM3OGZfNmQ4YjdlODc4MjQ2NWUzMWUxZjIxYTc2ZTU4MDI2YTFfSUQ6NzU5OTk1MTkzMDIwODIyNjI1OV8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 闪存模块组织

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NDU2MDMyYTU2YzdiN2RiMmY0MWZmNTgzZGNjMDljZTFfMDIxNzhkZTNhNDc4ZmYyNjYzOTAzNDU0NDQ5NmRlODJfSUQ6NzU5OTk1MzM0MjEyMjk3MDMwMF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## FLASH基本结构

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Y2ZlNGE3ZWU0ZGQzZWNjMjc0OWVlMmFmYTlhMzkyZDhfMWZiZTg0MDViNzFjMzVhNjlkZjU5ZjkwZDI5MDQ0N2JfSUQ6NzU5OTk1NDkwMDY1NDc1NDc3Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)









## FLASH解锁

FPEC共有三个键值:

RDPRT键 = 0x000000A5

KEY1 = 0x45670123

KEY2 = 0xCDEF89AB

解锁:

复位后,FPEC被保护,不能写入FLASH\_CR

在FLASH\_KEYR先写入KEY1，再写入KEY2，解锁

错误的操作序列会在下次复位前锁死FPEC和FLASH\_CR

加锁:

设置FLASH\_CR中的LOCK位锁住FPEC和FLASH\_CR









## 使用指针访问存储器

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGJjNjE3Y2ZmODZkZjI4ODVkMTk3ZDU2ZGZjYTNhYmRfOGZkMTlkZWIwNDc5OWMzOWE3YTIzODY3ZDczZWEwMmFfSUQ6NzU5OTk1NjY1ODMxNTkyMjM2Ml8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 程序存储器全擦除

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzFmOTgxZjFhZjBiOGRkZWE4NDg4NjgwNjkyMmYzMjBfOGRmNWI2NTljYmRhMjRmMzU1ZGQ4ZjE4Y2MxYWY4MmFfSUQ6NzU5OTk2MDQwODE2NDUzNTI0N18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)



## 程序存储器页擦除

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTNmNTM5ZDhmZGVjYTdmMTA3ZGJjZGZlMWZmNWZhNzlfZTljZDMwZTYzY2NlNDA2MjIwOGExY2Y4NTIzNmEyNWFfSUQ6NzU5OTk2MDg0MDkyNjMwMTM4M18xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 程序存储器编程

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MGZlNjJiMTIxYjdhNGE3NWJiNGYxODQ1NjEwZTc1NzBfZTI5NDI3NjBiZTA2YzkwYmIyOWI5NDQ0ODcyYmYyNGFfSUQ6NzU5OTk2MTExOTg2ODQ1NTg2Nl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 选项字节

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OWQyNzFmNGYxMGMwN2FjOThiOGY4ZjQ3YTBiN2VmYjZfZjY1ZjMwZDI1NmUwZGNlYWVhZjQ3ODIxOWZhYjdiMjhfSUQ6NzU5OTk2MTU3MjMxMTg0NjA5MF8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## 选项字节擦除

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjIxZGRkN2FiYTRjNTgxNTNmNjZiODk5NDc5NjQ0ZTlfYWExYzUzODEyOTdiMGFmYmE5OWNhZDc3YWViMGU4ODVfSUQ6NzU5OTk2MjM2ODI3NDA1ODQzNl8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)







## 器件电子签名

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MzAyMDQ5NzJkNzQzYzI5MjAwNDdiZjFiOGFkNTFiZjZfMDA5MTk3ZTU1NzUyNWQ5MzIyZmIwMzE1NDdmN2IzMjRfSUQ6NzU5OTk2Mjc5ODAwOTg2MzM4Ml8xNzgwNzMzMjM5OjE3ODA4MTk2MzlfVjM)





## 字,半字,字节

字: 

计算机内存的基本单位,由8位\(bit\)组成

表示范围: 0 \~ 255\(无符号\) , 或\-128 \~ 127\(有符号\)

典型C语言类型: uint8\_t,char

```C++
uint8_t data_byte = 0xFF;  *// 1字节数据*
```



半字: 

由16位\(2字节\)组成

表示范围: 0 \~ 65535（无符号），或 \-32768 \~ 32767（有符号\)

```C++
uint16_t data_halfword = 0x1234;  // 2字节数据
```

典型C语言类型: uint16\_t,short



字: 

在ARM架构中,1字 = 32位\(4字节\)

（注意：在x86架构中“字”通常为16位，但在ARM中固定为32位。）



表示范围: 0 \~ 4294967295（无符号），或 \-2147483648 \~ 2147483647（有符号）。

```C++
uint32_t data_word = 0xABCD1234;  // 4字节数据
```

典型C语言类型: uint32\_t,int

