---
title: "QT"
slug: "qt"
date: 1970-01-21T23:10:24+08:00
draft: false
source_file: "feishu://qt"
source_size: 8671
source_lines: 258
tags: []
categories: []
---

# QT
## QT控件:
### QPushBotton
作用: 可点击的按钮

### QLabel
作用: 显示文字或图片


### QSlider
作用: 滑块


### QTextEdit
作用: 带边框的分组框

### QProgress
作用: 进度条

## QWeight
QWeight是Qt中所有用户界面对象的基类: 
它提供最基本的窗口/控件功能: 尺寸,位置,鼠标键盘事件,
绘图,样式表，父子关系


## QMainWindow

补充: 
QMainWindow 继承自 QWidget，但增加了菜单栏、工具栏、状态栏、停靠窗口等布局区域。

QMainWindow 提供了一个预定义布局的主窗口框架

┌─────────────────────────────────┐
│          菜单栏 (Menu Bar)       │
├─────────────────────────────────┤
│          工具栏 (Tool Bars)      │
├──────────────────┬──────────────┤
│                  │              │
│   停靠窗口区域    │   中心部件    │
│  (Dock Widgets)  │ (Central     │
│                  │  Widget)     │
│                  │              │
├──────────────────┴──────────────┤
│          状态栏 (Status Bar)     │
└─────────────────────────────────┘



### 1. 中心部件相关
void setCentralWidget(QWidget *widget)
作用：设置窗口中央区域显示的部件（必须调用）。
参数：任何 QWidget 子类（如 QTextEdit、QTableWidget、自定义部件）。
示例：
cpp
QTextEdit *edit = new QTextEdit;setCentralWidget(edit);
QWidget *centralWidget() const
作用：返回当前的中心部件，没有则返回 nullptr。

### 2. 菜单栏相关
void setMenuBar(QMenuBar *menuBar)
作用：将指定的菜单栏设置为窗口的菜单栏。
注意：通常直接用 menuBar() 获取默认菜单栏，无需手动创建。
示例：
cpp
QMenuBar *myMenuBar = new QMenuBar(this);myMenuBar->addMenu("文件");setMenuBar(myMenuBar);
QMenuBar *menuBar() const
作用：返回窗口的菜单栏（如果没有则自动创建一个空的菜单栏）。

### 3. 状态栏相关
void setStatusBar(QStatusBar *statusBar)
作用：设置状态栏。
示例：
cpp
QStatusBar *sb = new QStatusBar(this);sb->showMessage("就绪");setStatusBar(sb);
QStatusBar *statusBar() const
作用：返回状态栏（如果没有则自动创建）。

### 4. 工具栏相关
void addToolBar(QToolBar *toolbar)
作用：添加一个工具栏，默认放在顶部区域。
示例：
cpp
QToolBar *toolBar = new QToolBar("主要工具");toolBar->addAction("打开");addToolBar(toolBar);
void addToolBar(Qt::ToolBarArea area, QToolBar *toolbar)
作用：将工具栏添加到指定区域：
Qt::TopToolBarArea（顶部）
Qt::BottomToolBarArea（底部）
Qt::LeftToolBarArea（左侧）
Qt::RightToolBarArea（右侧）
void insertToolBar(QToolBar *before, QToolBar *toolbar)
作用：在某个已有工具栏之前插入新工具栏。
Qt::ToolBarArea toolBarArea(const QToolBar *toolbar) const
作用：返回指定工具栏当前所在的区域。
void setToolButtonStyle(Qt::ToolButtonStyle style)
作用：控制所有工具栏上的按钮样式（仅图标、仅文字、文字在图标下等）。

### 5. 停靠窗口（QDockWidget）相关
void addDockWidget(Qt::DockWidgetArea area, QDockWidget *dockwidget)
作用：将停靠窗口添加到指定区域（左侧、右侧、顶部、底部）。
示例：
cpp
QDockWidget *dock = new QDockWidget("文件列表");dock->setWidget(new QListWidget);addDockWidget(Qt::LeftDockWidgetArea, dock);
void addDockWidget(Qt::DockWidgetArea area, QDockWidget *dockwidget, Qt::Orientation orientation)
作用：在分割停靠区域时指定布局方向。
void splitDockWidget(QDockWidget *first, QDockWidget *second, Qt::Orientation orientation)
作用：将两个停靠窗口以分割方式排列（水平或垂直）。
void tabifyDockWidget(QDockWidget *first, QDockWidget *second)
作用：将第二个停靠窗口以标签页形式与第一个合并。
void setDockOptions(DockOptions options)
作用：设置停靠窗口的行为，例如：
QMainWindow::AnimatedDocks（动画效果）
QMainWindow::AllowTabbedDocks（允许标签页）
QMainWindow::VerticalTabs（垂直标签）

### 6. 布局与外观
void setCorner(Qt::Corner corner, Qt::DockWidgetArea area)
作用：指定哪个停靠区域可以占据窗口的角落。
示例：让右上角属于左侧停靠区：
cpp
setCorner(Qt::TopRightCorner, Qt::LeftDockWidgetArea);
void setDocumentMode(bool enabled)
作用：启用“文档模式”（没有单独的工具栏/菜单栏边框，适合嵌入文档）。

### 7. 状态保存与恢复（重要！）
QByteArray saveState(int version = 0) const
作用：保存当前窗口所有工具栏、停靠窗口的位置、大小、可见性状态。
返回值：可以保存到文件或 QSettings。
bool restoreState(const QByteArray &state, int version = 0)
作用：恢复之前保存的状态。
示例：
cpp
// 保存QSettings settings("MyCompany", "MyApp");settings.setValue("mainWindow/state", saveState());// 恢复restoreState(settings.value("mainWindow/state").toByteArray());

### 8. 其他常用函数
void setIconSize(const QSize &iconSize)
作用：设置工具栏图标的大小。
void setToolTipDuration(int msec)
作用：设置工具提示显示的毫秒数。
QMenu *createPopupMenu()
作用：创建右击工具栏/停靠窗口时弹出的菜单（可重写以自定义）。



## QApplication

补充: 
QApplication 把自己存到全局变量 → QWidget::show() 通过全局变量找到 QApplication → 把自己注册进 QApplication 的窗口列表 → app.exec() 遍历这个列表分发事件。


每个使用Qt的GUI程序有且只有一个QApplication对象
(或者它的兄弟QGuiApplication/QCoreApplication)
管理事件循环: 处理窗口刷新,鼠标点击,键盘输入等事件
初始化应用程序: 处理命令行参数,设置系统字体/样式
提供全局信息: 屏幕尺寸,剪贴板,样式主题,调色板
管理资源: 处理翻译文件(.qm),样式表(QSS),图标搜索路径










## 事件循环
### 核心概念:
exec()开启了一个无限循环,不断从系统消息队列中取出事件
(鼠标点击,键盘,重绘等) 并发送到对应的窗口部件


### QApplication::exec()
作用: 启动应用程序的主事件循环,没有它,窗口会一闪而过然后退出
阻塞行为: 调用后程序会一直运行,直到你调用quit()或关闭所有窗口

典型用法: 

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MyMainWindow w;
    w.show();
    return app.exec();  // 进入主事件循环
}

### QDialog::exec()
作用: 以模态方式显示一个对话框












## QT数据类型
### QString – 字符串
作用：Unicode 字符串，Qt 的专用字符串类型，支持隐式共享（写时复制），效率高。


### QColor – 颜色
作用：存储 ARGB、HSV、CMYK 等颜色值，常用于绘图、样式表


### QByteArray – 字节数组
作用：存储原始字节数据（二进制或 8 位文本），常用于文件读写、网络传输。


### QList<T> – 列表
最常用的容器，支持通过索引快速访问，尾部插入快。存储类型 T 必须是可复制的（QObject 子类不能直接存储，需用指针）。


### QVector<T> – 动态数组
作用：与 QList 类似，但连续内存访问效率更高。在 Qt 6 中，QList 与 QVector 几乎统一，但习惯上仍可区分使用。


### QMap<K, V> – 映射（有序）
作用：键值对存储，按键排序。


### QHash<K, V> – 哈希表（无序）
作用：更快的查找，键不排序。


### QVariant – 通用值容器
作用：可存储任意 Qt 支持的类型（int, QString, QColor，甚至自定义类型需注册），用于模型/视图、设置项、信号槽跨类型传参。



### QRect / QRectF – 矩形
作用：存储整型坐标的矩形（x, y, width, height），常用于窗口几何、绘图区域。


### QPoint / QPointF – 点
作用：存储 (x, y) 坐标。


### QSize / QSizeF – 尺寸
作用：存储宽度和高度。



### QDateTime / QDate / QTime – 日期时间
处理日期、时间及时区


### QUrl – 统一资源定位符
解析和构建 URL，用于网络请求、本地文件路径（QUrl::fromLocalFile()）。



### QJsonDocument / QJsonObject / QJsonArray – JSON
解析和生成 JSON 数据，常用于 Web API 通信。
