---
title: "xv6 for RISC\\-V"
slug: "xv6-for-risc-v"
date: 2026-06-06T16:10:22+08:00
draft: false
source_file: "xv6 for RISC-V.md"
source_size: 33824
source_lines: 1750
tags:
  - "读书"
categories: []
---

# xv6 for RISC\-V

## jyy xv6导读



### 基础知识

应用视角的操作系统: 对象 \+ API

把操作系统当提供服务的黑盒子



xv6:Unix v6的现代"克隆"



文件关系

源文件 \(\.c/\.cpp\)  →  编译  →  目标文件 \(\.o\)  →  链接  →  可执行文件

↓

依赖文件 \(\.d\)  →  告诉构建系统何时需要重新编译





1\.源文件 \(Source Files\)

`.c` \- C语言源文件

`.cpp`/`.cc` \- C\+\+源文件

`.h`/`.hpp` \- 头文件





2\.目标文件 \(Object Files\) \- `.o` 文件

编译后的中间二进制文件





3\. 依赖文件 \(Dependency Files\) \- `.d` 文件

记录编译依赖关系的Makefile片段

```C++
# 编译时同时生成依赖文件
gcc -c main.c -o main.o -MMD -MF main.d
```





程序的执行\(状态变化序列\)有时比代码\(状态机\)更容易理解

使用工具进行“ tracing ”（追踪）

理解并发Bug

建立确定性心智模型





阅读makefile文件可以比较好的帮助理解代码





### xv6文件结构



kernel/目录 \- 操作系统内核

包含了xv6操作系统的所有内核代码,这是操作系统的核心部分



mkfs/目录 \- 文件系统创建工具

包含创建xv6系统镜像的工具



user/目录 \- 用户空间程序

包含所有在用户空间运行的程序









### xv6 22个系统调用

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZDA4NTk5YmE5ZTliZDE3NDU3ZDI2N2U2ZDY3OWU4NDJfN2IzNDJmNjI1YTZmYTcxNDU5NjNlODQ2NDY3MjYxYzFfSUQ6NzYwMDY4NjM2NjQxODY4NTEzMV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)







### xv6进程的地址空间

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzM4ZmQ1ZTY3NzhiNGFhODdhNjE3ZjlhMmUzZGZhMjBfMjFlOWViNGJiMzUwYTI2YzcxMTg0YjI4MDM3NWI1NDlfSUQ6NzYwMTM1NjE2NzQyNDEzNDA4NF8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### xv6系统调用

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTAzMjkyMTQ3NWVmMmFkNTZiYTY4N2YyYzlkOGM1ZGNfN2UwYmVjNWI0N2E1OTliNDE3YjUyMDEwMmUyNDg0MmVfSUQ6NzYwMTQ3ODExNDQ2ODA3MjY2Ml8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### Trampoline\(跳板\)

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDM3MDNiOGFkMjU4NDEwYTlkZmIxYTU3ZGNkOTdhZDBfMDk2OGY5MDI5ZGY4YTg1ZWU5OGVhODA5NzIyZjU2ZDNfSUQ6NzYwMTQ3ODIzMzY5NTUzODEyMF8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### 程序的状态

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NmQyMWUzOGE2OTNkZTI0OTM2Njg3YzljZmQwMzAwNmJfNTMyNDY1OGIwMTQ0ZjQ4M2U3ZTY4ZWM2NzNhMTUwMjNfSUQ6NzYwMzU2ODczNTYyMjAwODAwN18xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### 虚拟化: 状态的管理



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ODI3NjVmNTBlZjJlOTljMGNiYzc1ZTJhNTRlZmE5NzRfNmJiMDJkNDA1NTk0OTVjODI3OTkzYmNlNWZjMDhmMmRfSUQ6NzYwMzU3Mzg5NDcxMjc1NzE3OV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### 状态的封存：Trivial的操作系统实现

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MTk2NWJmZGVlYWY1YzA4NTcwNzU2ZDk4MGUyMjFiMmZfNjU5ZjQ5ZDVjNzM3Y2E2NjE2YTQzNTNlZTIwMGFlNjlfSUQ6NzYwMzk1Mjg0NzUzMjg1NDIwMl8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### 状态的封存: 体系结构相关的处理

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NTQ5YzdiZDkyNmQ3ZGY0NGY0NTI0YWI5ZWZkM2Q0NTVfMjY1OTFlZWZjODA4MDdjYWQyNTBjZjRjYjk3MzU1MjdfSUQ6NzYwNDEwNTg0NTc2ODU3MTg0N18xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)







### 再次调试系统调用

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzBjZTM2YzhjM2I1ZTg2ZWRhYTY5ZWVjYmJhZDJlM2RfZmZlMjI3OTk3NDE2ZjNjNDJmMTk0YzRmOWE3NDBiODJfSUQ6NzYwNDEwODEyNTAwNzc5MzA5Ml8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





### 调用usertrap\(\)后的系统状态

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjRiNGEyNjdlYTQ0YTYzZTEzZjQ1YjM4MDJlNWE0NGZfOGZhNDhjOWRiOWJkMWEyYzRlYzFkNTUxMWNlZjgzZjdfSUQ6NzYwNDExMDA3Njg0ODI0NTcxOV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)









### 小结: 状态机的封存

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=MDg4ODQ3NDYwY2ZhYzdmYWI2NjEwOWZjNjhkNThjMWRfZDM2NGIxMzQ2YzVlZTA2MWQ3MDFiMTc4NTdlNzczZTRfSUQ6NzYwNDExMDU5MTIzNzY1NTc3MV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)









### invariant状态

invariant\(不变式\)在计算机科学中指在程序的某些关键点上必须始终为真的条件或约束

状态机被封存



### 总结

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzkzZGY4ZmFmNWI2YWY4ZWIwMmFmYmMxYWFhOGRmMmNfOTNhM2QxOThkZDZiMTI5ZWIxM2JlOWY4MWE0ZTcwY2ZfSUQ6NzYwNDExMjY0MzI5MjQ4MjQ4OV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)



操作系统是中断处理程序

操作系统是状态机的管理者























## 缓存系统bio\.c/bio\.h





### LRU

```C++
最近使用的（最新）                         最久未用的（最旧）
               ↓                                          ↓
    [head] ←→ [bufA] ←→ [bufB] ←→ [bufC] ←→ ... ←→ [bufZ] ←→ [head]
      ↑        最新使用                                  最旧使用      ↑
      └─────────────────────────────────────────────────────────────┘
                    循环连接
```



xv6的LRU维护特点: 惰性更新 \+ 只在释放时移动

核心原则：

缓存命中时：不移动缓冲区在LRU中的位置

缓存未命中时：从链表尾部查找空闲缓冲区

缓冲区释放时：只有当引用计数归零 \(`refcnt == 0`\) 时才移动到链表头部



传统LRU

假设: 

最近被访问的数据，在不久的将来很可能再次被访问

很久没有被访问的数据，未来也不太可能被访问



每次访问缓存项时,都会将其移动到链表的头部

当缓存满的时候删除尾部节点\(最久未使用\)





```C++

```





### 缓存系统

```C++
bcache（缓存系统）
        ╔═══════════════════════════════════╗
        ║ struct spinlock lock;            ║ ← 保护整个系统
        ║                                   ║
        ║ struct buf buf[NBUF];            ║ ← 缓冲区数组
        ║ ┌─────────┐ ┌─────────┐         ║
        ║ │ buf[0]  │ │ buf[1]  │ ...     ║ ← 每个都是struct buf
        ║ │ valid=1 │ │ valid=0 │         ║
        ║ │ dev=1   │ │ dev=1   │         ║
        ║ │ blk=123 │ │ blk=456 │         ║
        ║ │ data[]  │ │ data[]  │         ║
        ║ └─────────┘ └─────────┘         ║
        ║                                   ║
        ║ struct buf head;                ║ ← LRU链表头
        ║   prev → buf[28]                ║
        ║   next → buf[1]                 ║
        ╚═══════════════════════════════════╝
                 ↑        ↑        ↑
       通过prev/next指针连接所有缓冲区
       形成LRU双向循环链表
```





## 睡眠锁sleeplock\.h/sleeplock\.c

补充: 也叫条件锁或长时间持有的锁

```C++
// Long-term locks for processes
struct sleeplock {
  uint locked;       // Is the lock held?
  struct spinlock lk; // spinlock protecting this sleep lock
  
  // For debugging:
  char *name;        // Name of lock.
  int pid;           // Process holding lock
};



// Sleeping locks

#include "types.h"
#include "riscv.h"
#include "defs.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "proc.h"
#include "sleeplock.h"

void
initsleeplock(struct sleeplock *lk, char *name)
{
  initlock(&lk->lk, "sleep lock");
  lk->name = name;
  lk->locked = 0;
  lk->pid = 0;
}

void
acquiresleep(struct sleeplock *lk)
{
  acquire(&lk->lk);
  while (lk->locked) {
    sleep(lk, &lk->lk);
  }
  lk->locked = 1;
  lk->pid = myproc()->pid;
  release(&lk->lk);
}

void
releasesleep(struct sleeplock *lk)
{
  acquire(&lk->lk);
  lk->locked = 0;
  lk->pid = 0;
  wakeup(lk);
  release(&lk->lk);
}

int
holdingsleep(struct sleeplock *lk)
{
  int r;
  
  acquire(&lk->lk);
  r = lk->locked && (lk->pid == myproc()->pid);
  release(&lk->lk);
  return r;
}

```







## 自旋锁spinlock\.c/spinlock\.h

```C++
// Mutual exclusion lock.
struct spinlock {
  uint locked;       // Is the lock held?

  // For debugging:
  char *name;        // Name of lock.
  struct cpu *cpu;   // The cpu holding the lock.
};



// Mutual exclusion spin locks.

#include "types.h"
#include "param.h"
#include "memlayout.h"
#include "spinlock.h"
#include "riscv.h"
#include "proc.h"
#include "defs.h"

void
initlock(struct spinlock *lk, char *name)
{
  lk->name = name;
  lk->locked = 0;
  lk->cpu = 0;
}

// Acquire the lock.
// Loops (spins) until the lock is acquired.
void
acquire(struct spinlock *lk)
{
  push_off(); // disable interrupts to avoid deadlock.
  if(holding(lk))
    panic("acquire");

  // On RISC-V, sync_lock_test_and_set turns into an atomic swap:
  //   a5 = 1
  //   s1 = &lk->locked
  //   amoswap.w.aq a5, a5, (s1)
  while(__sync_lock_test_and_set(&lk->locked, 1) != 0)
    ;

  // Tell the C compiler and the processor to not move loads or stores
  // past this point, to ensure that the critical section's memory
  // references happen strictly after the lock is acquired.
  // On RISC-V, this emits a fence instruction.
  __sync_synchronize();

  // Record info about lock acquisition for holding() and debugging.
  lk->cpu = mycpu();
}

// Release the lock.
void
release(struct spinlock *lk)
{
  if(!holding(lk))
    panic("release");

  lk->cpu = 0;

  // Tell the C compiler and the CPU to not move loads or stores
  // past this point, to ensure that all the stores in the critical
  // section are visible to other CPUs before the lock is released,
  // and that loads in the critical section occur strictly before
  // the lock is released.
  // On RISC-V, this emits a fence instruction.
  __sync_synchronize();

  // Release the lock, equivalent to lk->locked = 0.
  // This code doesn't use a C assignment, since the C standard
  // implies that an assignment might be implemented with
  // multiple store instructions.
  // On RISC-V, sync_lock_release turns into an atomic swap:
  //   s1 = &lk->locked
  //   amoswap.w zero, zero, (s1)
  __sync_lock_release(&lk->locked);

  pop_off();
}

// Check whether this cpu is holding the lock.
// Interrupts must be off.
int
holding(struct spinlock *lk)
{
  int r;
  r = (lk->locked && lk->cpu == mycpu());
  return r;
}

// push_off/pop_off are like intr_off()/intr_on() except that they are matched:
// it takes two pop_off()s to undo two push_off()s.  Also, if interrupts
// are initially off, then push_off, pop_off leaves them off.

void
push_off(void)
{
  int old = intr_get();

  // disable interrupts to prevent an involuntary context
  // switch while using mycpu().
  intr_off();

  if(mycpu()->noff == 0)
    mycpu()->intena = old;
  mycpu()->noff += 1;
}

void
pop_off(void)
{
  struct cpu *c = mycpu();
  if(intr_get())
    panic("pop_off - interruptible");
  if(c->noff < 1)
    panic("pop_off");
  c->noff -= 1;
  if(c->noff == 0 && c->intena)
    intr_on();
}

```



## 自旋锁和睡眠锁对比





## ELF/elf\.h



## xv6引导加载器\(Boot Loader\)/bootmain\.c

加载内核: 从硬盘读取 xv6 内核（ELF 格式的可执行文件）

内存布局: 将内核的各部分放到正确内存位置

转交控制权: 跳转到内核的入口点，启动操作系统





## 控制台/console\.c

### 环形缓冲区 \-\> 循环队列

环形缓冲区是一种循环队列数据结构,特别适合处理生产者\-消费者场景的数据流

直线缓冲区: 满了就得停，或者移动所有数据

环形缓冲区: 跑到终点后回到起点,可以无限循环使用

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OGE2ODJmNmEwMjE4MzU4M2UxMjA5ZTMyMGUwMDAxM2ZfMTY2MzEzZTIxOWY0MmE4ZTdmYTg1OWQ4ZTkzYzExMzFfSUQ6NzYwMjY0MTUyMDkxNTA0MTUwMl8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)









![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=OTU1ZjBmNTM1YWNkMjQxZWZmNjYwYTRmZDVjNDc4MjhfYjkzNThkMzMxZjVmOGFlYmMyM2VmNWQ3M2M3OGM3MmVfSUQ6NzYwMjY0MTgwMTMwNDE4MTk4Ml8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=NGFiZTkyMWY4NTRmYWQwNTA0NzNhMGU2YjBiZmFlZGVfM2RmMDRmYmM5YzNiNDAyNDNhNGQwNWM0NmFkZTM4ODFfSUQ6NzYwMjY0NjA4NjI1MDk5MDc5OF8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)

两个指针之间应该空一个空格

不然WriteIndex和readIndex重合的话就会分不清楚缓冲区是满了还是空了













xv6的控制台缓冲区是一个支持行编辑的生产者\-消费者环形缓冲区: 

生产者\(consoleintr\): 接收键盘中断,写入字符

消费者\(consoleread\): 响应read系统调用,读取字符

三个指针: 实现复杂的行编辑功能\(退格,删除行等\) 







## 串口UART/uart\.c





### UART和USART的区别


UART: 通用异步收发器

功能: 仅支持异步串行通信

特点: 没有时钟线,依赖双方约定的波特率









USART: 通用同步/异步收发器

功能: 支持同步和异步两种通信模式

特点: 可以配置为同步模式\(有时钟线\)或异步模式







## 管道/pipe\.c



环形缓冲区: 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Nzg5ZjY1NmUwNWE2ZDViMjkyYjkwNjZmNmUyNGYxZTZfMThjNTAyMDMxYTY0ZGNmNzE1ZjRhYTI1ZjJlNGU0OWZfSUQ6NzYwMjg0ODE3ODk4MzYwMzE2Nl8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)













## 进程proc\.c/proc\.h





## 文件日志系统/log\.c

解决: 崩溃恢复\(Crash Recovery\)

不要直接修改真正的磁盘数据块。先把你打算做的修改\.全部写到一个叫做"日志区"的专用地方。只有当日志区完整记录了所有修改后,才把它们搬运到真正的目的地\.



这样，如果断电：

**断在日志写完之前**：重启后，系统发现日志不完整，直接丢弃。就像操作从未发生过。文件系统完好。

**断在日志写完之后**：重启后，系统发现日志里有完整的操作记录，于是重新执行这些操作（Replay）。文件系统完好。











## 内核主函数/main\.c









## RISC\-V 平台级中断控制器（PLIC）的驱动程序/plic\.c

PLIC     \-\>  Platform Level Interrupt Controller



中断是一种硬件通知机制，允许外部设备或内部异常打断处理器的正常执行流程，让处理器立即处理紧急事件。



中断的类型: 

外部中断（硬件中断）

内部中断（异常/陷阱）







## 系统调用/syscall\.c/syscall\.h











## 进程相关系统调用/sysproc\.c

















## 中断和异常处理/trap\.c



用户程序执行ecall指令

↓

CPU设置stvec寄存器指向uservec（在trampoline中）

↓

CPU跳转到uservec（汇编代码）

↓

uservec保存用户寄存器到trapframe

↓

uservec设置内核栈，跳转到usertrap（C代码）

↓

usertrap调用syscall\(\)处理系统调用

↓

prepare\_return准备返回用户空间

↓

userret（在trampoline中）恢复用户寄存器

↓

返回用户程序继续执行





## 文件系统/sysfile\.c









## 虚拟内存管理/vm\.c/vm\.h























## xv6: a simple Unix\-like teaching operating system















## xv6在RISC\-V架构与x86架构的区别













## trampoline\.s/蹦床代码

trampoline是一段特殊的汇编代码,位于用户地址空间和内核地址空间的相同虚拟地址处



主要功能: 

1\.在用户态和内核态之间安全地切换

2\.保存和恢复用户上下文

3\.切换页表

4\.跳转到内核陷阱处理程序





共享映射:

trampoline页面在内核页表和所有用户页表中都映射到相同的虚拟地址\\

（`TRAMPOLINE = 0x3ffffff000`）



权限设置：
在用户页表中,trampoline页面被标记为可执行但不可写;在内核页表中,可读可写



位置固定:
总是位于地址空间的最高页 





xv6的系统调用实现: 

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZWQ1Y2ExNzA4N2YwYTk1M2YwMGE1M2UzY2I4NGFjNWJfYTdjMDdmOGM4M2NlNGZjZmQ0ZGI2Zjk0N2FhZjNjZTdfSUQ6NzYwMzM3NTgxNTk2MjcyNTMxOV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)



`trampoline`（蹦床）和`trapframe`（陷阱帧）是处理用户态和内核态之间切换的关键机制。它们共同协作以实现安全的上下文切换和系统调用处理。



协作流程: 

1\.用户程序通过ecall进入内核

2\.trampoline保存用户上下文到trapframe

3\.内核处理系统调用/陷阱

4\.trampoline从trapframe恢复用户上下文

5\.返回用户程序继续执行





```YAML
#
        # low-level code to handle traps from user space into
        # the kernel, and returns from kernel to user.
        #
        # the kernel maps the page holding this code
        # at the same virtual address (TRAMPOLINE)
        # in user and kernel space so that it continues
        # to work when it switches page tables.
        # kernel.ld causes this code to start at 
        # a page boundary.
        #

#include "riscv.h"
#include "memlayout.h"

.section trampsec
.globl trampoline
.globl usertrap
trampoline:
.align 4
.globl uservec
uservec:    
        #
        # trap.c sets stvec to point here, so
        # traps from user space start here,
        # in supervisor mode, but with a
        # user page table.
        #

        # save user a0 in sscratch so
        # a0 can be used to get at TRAPFRAME.
        csrw sscratch, a0

        # each process has a separate p->trapframe memory area,
        # but it's mapped to the same virtual address
        # (TRAPFRAME) in every process's user page table.
        li a0, TRAPFRAME
        
        # save the user registers in TRAPFRAME
        sd ra, 40(a0)
        sd sp, 48(a0)
        sd gp, 56(a0)
        sd tp, 64(a0)
        sd t0, 72(a0)
        sd t1, 80(a0)
        sd t2, 88(a0)
        sd s0, 96(a0)
        sd s1, 104(a0)
        sd a1, 120(a0)
        sd a2, 128(a0)
        sd a3, 136(a0)
        sd a4, 144(a0)
        sd a5, 152(a0)
        sd a6, 160(a0)
        sd a7, 168(a0)
        sd s2, 176(a0)
        sd s3, 184(a0)
        sd s4, 192(a0)
        sd s5, 200(a0)
        sd s6, 208(a0)
        sd s7, 216(a0)
        sd s8, 224(a0)
        sd s9, 232(a0)
        sd s10, 240(a0)
        sd s11, 248(a0)
        sd t3, 256(a0)
        sd t4, 264(a0)
        sd t5, 272(a0)
        sd t6, 280(a0)

        # save the user a0 in p->trapframe->a0
        csrr t0, sscratch
        sd t0, 112(a0)

        # initialize kernel stack pointer, from p->trapframe->kernel_sp
        ld sp, 8(a0)

        # make tp hold the current hartid, from p->trapframe->kernel_hartid
        ld tp, 32(a0)

        # load the address of usertrap(), from p->trapframe->kernel_trap
        ld t0, 16(a0)

        # fetch the kernel page table address, from p->trapframe->kernel_satp.
        ld t1, 0(a0)

        # wait for any previous memory operations to complete, so that
        # they use the user page table.
        sfence.vma zero, zero

        # install the kernel page table.
        csrw satp, t1

        # flush now-stale user entries from the TLB.
        sfence.vma zero, zero

        # call usertrap()
        jalr t0

.globl userret
userret:
        # usertrap() returns here, with user satp in a0.
        # return from kernel to user.

        # switch to the user page table.
        sfence.vma zero, zero
        csrw satp, a0
        sfence.vma zero, zero

        li a0, TRAPFRAME

        # restore all but a0 from TRAPFRAME
        ld ra, 40(a0)
        ld sp, 48(a0)
        ld gp, 56(a0)
        ld tp, 64(a0)
        ld t0, 72(a0)
        ld t1, 80(a0)
        ld t2, 88(a0)
        ld s0, 96(a0)
        ld s1, 104(a0)
        ld a1, 120(a0)
        ld a2, 128(a0)
        ld a3, 136(a0)
        ld a4, 144(a0)
        ld a5, 152(a0)
        ld a6, 160(a0)
        ld a7, 168(a0)
        ld s2, 176(a0)
        ld s3, 184(a0)
        ld s4, 192(a0)
        ld s5, 200(a0)
        ld s6, 208(a0)
        ld s7, 216(a0)
        ld s8, 224(a0)
        ld s9, 232(a0)
        ld s10, 240(a0)
        ld s11, 248(a0)
        ld t3, 256(a0)
        ld t4, 264(a0)
        ld t5, 272(a0)
        ld t6, 280(a0)

        # restore user a0
        ld a0, 112(a0)
        
        # return to user mode and user pc.
        # usertrapret() set up sstatus and sepc.
        sret

```







处理器的虚拟化

为什么死循环不能使计算机被彻底卡死? 



原理上:

1\.硬件会发生中断\(类似于强行插入的ecall\)

2\.切换到操作系统代码执行

3\.操作系统代码可以切换到另一个进程执行











## xv6面经





### 惰性分配内存







### 内存写时复制











### 内存的超售机制







### xv6的物理内存如何分配







### 定时器回调如何实现











### xv6配置环境的时候申请多大物理内存









### xv6内存的惰性分配



惰性分配的问题: 

频繁触发缺页异常陷入内核态开销很大







惰性分配的优化方法: 

预取,根据进程访问局部性预取相邻的多个页,文件映射时,linux默认readhead为128KB









### 文件系统的读写全流程







### xv6是什么类型的操作系统介绍一下,是否是实时操作系统











### 操作系统分几个模块













### 虚拟地址翻译过程











### 系统调用的详细过程









### 进程同步的策略,进程,线程如何同步









### 有看过linux内核设计吗

考虑后面看一下





















## 中山OS

短期调度

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZmFmMmU1ZmEwZTQ4MzUxMDBjOGNlOGQyMDkzNmIxM2NfZjJkYmUwMDFhOGIzNDI1NWIxYWY4ZTY3MDdmZmY0NzlfSUQ6NzYwNTA0Njc1ODYzMjYyMzA3Ml8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)









中期调度

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=ZjllODNiNzhkNjNkYjBmMGMzZmU4YWUxM2ZhN2M5Y2FfYjY3YzRmYmZjNWE0NWMyZDFmODRjODQ4YzE0ZjgyYzRfSUQ6NzYwNTA0NjY1MDQ4Njg1Mjc4Nl8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)







进程调度总览

![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=Yzg0YTJjM2Q2N2MxMDVhYzM1OTk3YjJjZDM0MWRjZjdfZTYzNzk5NmFkNDY0OWRjMmY2YzViMWQ5NDk4NDFjMTNfSUQ6NzYwNTA0OTAzMDkyMTkxNTMyMl8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)











## vscode gdb调试


1\.继续:\(Continue\)/F5
含义: 解除暂停,全速奔跑吧
发生什么:

xv6会从当前暂停的地方恢复运行,CPU全速工作

直到遇到下一个断点,它才会再次停下来



2\.逐过程\(Step Over\)/F10

含义: 往下走一行,但我不想进这个函数里面看

发生什么: 

执行当前高亮的一行代码

如果这行代码调用一个函数,调试器会瞬间把那个函数跑完,然后停在下一行



3\.单步调试\(Step Into\)/F11

含义: 往下走一行,如果有函数,我要钻进去看细节

发生什么: 

执行当前行

如果这行函数调用了函数,调试器会跳进那个函数的内部,停在那个函数的第一行



4\.跳出\(Step Out\)/shift \+ F11

含义: "我不小心钻进来了,或者我看腻了,快带我出去"
发生什么:

让我当前所在的函数瞬间执行完剩下的所有代码

然后停在调用它的那行代码的下一行



光标停在的哪一行,执行了吗? 

没有执行

当黄色的高亮条\(光标\)停在某一行代码上时,代表CPU"正准备"执行这一行代码,但还没下手







## xv6\-lab



### lab1\-util



#### Boot xv6/启动xv6 



#### Sleep/睡眠

为xv6实现UNIX的sleep程序;你的sleep程序应暂停用户指定的时钟滴答数\.一个滴答是xv6内核 定义的时间单位,即定时器芯片两次中断之间的间隔\.你的解决方法应放在文件user/sleep\.c中







#### pingpong/乒乓


编写一个程序,利用UNIX系统调用,通过一对管道\(每个方向一个\),在两个进程之间"乒乓"传递一个字节
具体流程如下: 

1\.父进程应该向子进程发送一个字节

2\.子进程应该: 

接收这个字节

打印\<pid\>: received ping \(其中\<pid\>是它的进程ID\)

把这个字节写回管道,发给父进程

退出

3\.父进程应该:

从子进程读取这个字节

打印\<pid\>: received pong

退出

一些提示: 

使用pipe来创建管道

使用fork来创建子进程

使用read从管道读取,使用write向管道写入

使用getpid获取当前进程的ID

别忘了把程序名加入到Makefile的UPROGS列表中

xv6的用户程序能用的库函数很少,你可以查看user/user\.h里的列表



预期输出: 

```C++
$ pingpong
4: received ping
3: received pong
$
```









#### primes/质数 素数筛 Sieve of Eratosthenes（埃拉托斯特尼筛法）

编写一个并发版本的"素数筛"程序,通过管道\(pipes\)来筛选素数\.这个创意归功于Unix管道的发明者Doug Mcllroy



使用pipe和fork来建立一个流水线

1\.每一个进程将数字2到35输入到管道中

2\.对于每一个新发现的素数,你需要创建一个新的进程

这个新进程从它的左邻居\(通过一个管道\)读取数据

将筛选后的数据写给它的右邻居\(通过另一个管道\)

3\.因为xv6的资源\(文件描述符和进程数\)有限,第一个进程只要算到35就可以停止了



一些提示: 

严谨关闭文件描述符: 非常重要\! 每个进程必须关闭它不需要的文件描述符\.否则,你的程序会在还没算到35之前就耗尽xv6的资源\(xv6默认每个进程只能打开16个文件\)\.



等待子进程: 当第一个进程处理完35后,它应该等待这个流水线结束\(包括所有的子进程,孙子进程等\)\.主进程应该在所有输出打印完毕,且所有其他进程都退出之后才退出



读取结束信号: 当管道的写入端（Write\-side）被关闭时，`read` 函数会返回 0。利用这个特性来判断什么时候结束。



直接传整数: 最简单的方法是直接向管道写入 32 位（4字节）的 `int`，而不是像字符串那样用 ASCII 格式读写。



按需创建: 你应该只在需要的时候\(发现新素数时\)才创建新的进程,不要一开始就全创建好



别忘了把程序加入Makefile UPROGS中



![Image](https://internal-api-drive-stream.feishu.cn/space/api/box/stream/download/authcode/?code=YzNjZDVhODJjMGJjMDk1ZWZkM2Q1Yjg1OTc0NGY3OWJfMjZkNmMzNzU1ZjM4Mjc4NWU4MjZiYmNmN2ZkZDdlY2JfSUQ6NzYwOTUxMTY1MTIzNDA1NzQxOV8xNzgwNzMzNDE4OjE3ODA4MTk4MThfVjM)





#### find/查找

编写一个简单版本的UNIX find 程序: 在一个目录树\(directory tree\) 中查找所有具有特定名称的文件\. 你的代码应该写在 user/find\.c 文件中



\[提示 \(Hints\) 解析\]

1. 看看user/ls\.c 是如何读取目录的: 这是最重要的一条提示\! 在Unix中，目录其实也是一种"文件", 里面存的是"目录项"\(文件或子目录的名字和它们对应的索引节点号\)\.ls\.c里有完整读取目录的代码模板

2. 使用递归\(recursion\): 因为目录里面可能还有目录,所以当程序遇到目录时,需要调用自己进入子目录继续找

3. 不需要递归遍历\.和\.\.: \.代表当前目录, \.\.代表上一级目录\.如果不跳过他们,程序就会在原地无限打转

4. 文件系统的更改在qemu运行期间是持久的: 如果你建了文件,下次启动还在\. 如果想得到干净的文件系统,可以运行make clean然后再make qemu

5. 你需要使用C语言的字符串: 复习一下C语言里字符串是如何\\0结尾的

6. 注意,不能像Python里那样 == 比较字符串: 必须使用strcmp\(\) 函数

7. 把程序添加到Makefile的UPROGS中: 这样编译系统才会把你的find\.c编译成可在xv6里运行的命令











#### XARGS

写一个简化版的xargs程序\.它需要从标准输入\(stdin\)一行一行地读取数据,然后对每一行执行指定的命令,把读取到的这一行内容作为参数喂给这个命令



**例子 1**：`echo hello too | xargs echo bye`

左边输出：`hello too`（通过管道传给右边）。

右边接收：`xargs` 读取到了 `hello too`。

xargs 的动作：xargs 后面跟着的命令是 `echo bye`。它把 `hello too` 追加到后面，变成了 `echo bye hello too`。

最终执行：打印出 `bye hello too`。

**例子 2**：`echo "1\n2" | xargs -n 1 echo line` \(在咱们的 lab 中不需要实现 `-n` 这个优化，只要**按行读取**就行\)。

第一行是 `1`，执行 `echo line 1`。

第二行是 `2`，执行 `echo line 2`。



官方提示\(Hints\): 

用fork和exec对每一行输入调用命令\.父进程要用wait等待子进程执行完毕\.

逐个字符读取输入,直到遇到换行符\\n

kernel/param\.h里有一个MAXARG，定义了最大参数个数,声明数组时可以用\.

写完时记得把程序加到Makefile的UPROGS







### lab2\-system calls



#### 系统调用跟踪（System call tracing）（中等难度）

在本作业中，你将添加一个系统调用跟踪（tracing）功能，这可能会在你调试后续的实验时有所帮助。你将创建一个新的 `trace` 系统调用来控制跟踪。它应接受一个参数：一个整数“掩码（mask）”，其二进制位指定了要跟踪哪些系统调用。

例如，要跟踪 `fork` 系统调用，程序需要调用 `trace(1 << SYS_fork)`，其中 `SYS_fork` 是 `kernel/syscall.h` 中定义的系统调用号。你需要修改 xv6 内核，以便在每次系统调用即将返回时，如果掩码中设置了该系统调用对应的位，则打印出一行信息。这行信息应该包含：**进程 ID**、**系统调用的名称**和**返回值**；你不需要打印系统调用的参数。

`trace` 系统调用应该为调用它的进程及其随后 `fork`（派生）出的所有子进程启用跟踪，但不应影响其他无关的进程。

我们提供了一个用户级程序 `trace`，用于在启用跟踪的情况下运行另一个程序（参见 `user/trace.c`）。当你完成实验后，你应该能看到如下类似的输出：

Bash

$ trace 32 grep hello README
3: syscall read \-\> 1023
3: syscall read \-\> 966
3: syscall read \-\> 70
3: syscall read \-\> 0
$
$ trace 2147483647 grep hello README
4: syscall trace \-\> 0
4: syscall exec \-\> 3
4: syscall open \-\> 3
4: syscall read \-\> 1023
4: syscall read \-\> 966
4: syscall read \-\> 70
4: syscall read \-\> 0
4: syscall close \-\> 0
$
$ grep hello README
$
$ trace 2 usertests forkforkfork
usertests starting
test forkforkfork: 407: syscall fork \-\> 408
408: syscall fork \-\> 409
409: syscall fork \-\> 410
410: syscall fork \-\> 411
409: syscall fork \-\> 412
410: syscall fork \-\> 413
409: syscall fork \-\> 414
411: syscall fork \-\> 415
\.\.\.
$   

输出示例解析：\(\)\(\)\(\)\(\)



在上面的**第一个示例**中，`trace` 调用 `grep` 且仅跟踪 `read` 系统调用。参数 `32` 即为 `1 << SYS_read`。\(\)\(\)\(\)\(\)



在**第二个示例**中，`trace` 运行 `grep` 并跟踪所有系统调用；`2147483647` 这个数字的低 31 位全部为 1。\(\)\(\)\(\)



在**第三个示例**中，程序没\(\)有被 `trace` 运行，因此没有打印任何跟踪输出。\(\)\(\)



在**第四个示例**中，`usertests` 中的 `forkforkfork` 测试的所\(\)有后代进程的 `fork` 系统调用都被跟踪了。\(\)



如果你的程序行为如上所示（进程 ID 具体数字可能不同），那么你的解\(\)决方案就是正确的。



---

一些提示（Hints）：

在 `Makefile` 的 `UPROGS` 中添加 `$U/_trace`。

运行 `make qemu`，你会发现编译器无法编译 `user/trace.c`，因为该系统调用的用户空间存根（stubs）还不存在：

请在 `user/user.h` 中为该系统调用添加函数原型。

在 `user/usys.pl` 中添加存根（stub）。

在 `kernel/syscall.h` 中添加系统调用编号。 *Makefile 会调用 Perl 脚本 **`user/usys.pl`** 来生成 **`user/usys.S`**（这就是实际的系统调用存根），这些存根使用 RISC\-V 的 **`ecall`** 指令陷入（transition）到内核。* 解决编译问题后，再次运行 `make qemu` 并在 xv6 shell 中执行 `trace 32 grep hello README`；它会失败，因为你还没有在内核中实现该系统调用。

在 `kernel/sysproc.c` 中添加一个 `sys_trace()` 函数来实现这个新的系统调用。你需要在 `proc` 结构体（参见 `kernel/proc.h`）中添加一个新的变量，用来记录传入的参数（掩码）。从用户空间获取系统调用参数的函数位于 `kernel/syscall.c` 中，你可以在 `kernel/sysproc.c` 中看到它们的使用示例。

修改 `fork()` 函数（参见 `kernel/proc.c`），将跟踪掩码从父进程复制到子进程。

修改 `kernel/syscall.c` 中的 `syscall()` 函数以打印跟踪输出。为了打印出系统调用的名称，你需要添加一个包含系统调用名称的字符串数组，以便用系统调用号来进行索引。

















































