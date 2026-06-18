---
title: "opencv知识库\\-\\-\\-整理"
slug: "opencv知识库-整理"
date: 2026-06-06T16:05:04+08:00
draft: false
tags: []
categories: []
---

# opencv知识库\-\-\-整理

## 基本概念

预处理

按我的话来说，所谓预处理，就是在图像还未真正进行识别处理前，对图像进行简易、全局的处理。这一块通常在产生图片时进行操作，不属于我们[视觉识别](https://so.csdn.net/so/search?q=%E8%A7%86%E8%A7%89%E8%AF%86%E5%88%AB&spm=1001.2101.3001.7020)的重点（但是预处理真的很重要）。这里仅简单介绍我们使用的两个预处理过程。

降低曝光度

通过相机，我们能够源源不断地获取到当前的画面，也就是一帧帧的图像。自瞄算法处理的对象，就是这每一张图像。

在实战中，由于环境光的干扰，如果直接对图片进行算法处理，会错误的提取多余的特征，导致算法的准确度和速度大大降低。由于灯条自己会发光，可以有效和将灯条与环境光很好的区分开来。为了便于分离灯条与其他光线，一般将曝光设置得很低，如下图为相机获取到的画面：（RoboMaster视觉教程（1）摄像头）







## 常见关键字







## 数据类型\-\-\-类对象

1\. `cv::Point2f`

`cv::Point2f` 是一个二维点，包含两个浮点数（`x` 和 `y`），用于表示二维空间中的点。

- **加法和减法**：可以直接对 `Point2f` 进行加法和减法操作。

- **计算距离**：使用 `cv::norm` 函数计算两点之间的欧几里得距离。

- 例如：

float distance = norm\(point1 \- point2\);

2\. `cv::Point3f`

`cv::Point3f` 是一个三维点，包含三个浮点数（`x`、`y` 和 `z`），用于表示三维空间中的点。

- **加法和减法**：可以直接对 `Point3f` 进行加法和减法操作。

- **计算距离**：使用 `cv::norm` 函数计算两点之间的欧几里得距离。

- 例如：

float distance = norm\(point1 \- point2\);



3\. `cv::Point2i` 和 `cv::Point3i`

除了浮点数版本的 `Point2f` 和 `Point3f`，OpenCV 还提供了整数版本的点类型：

`cv::Point2i`：二维整数点，包含两个整数（`x` 和 `y`）。

`cv::Point3i`：三维整数点，包含三个整数（`x`、`y` 和 `z`）。

它们的用法与浮点数版本类似，只是数据类型为整数。

### Mat赋初值（经常忘记）

cv::Mat cameraMatrix = \(cv::Mat\_\<double\>\(3, 3\) \<\<

1200\.0, 0\.0, 640\.0,

0\.0, 1200\.0, 360\.0,

0\.0, 0\.0, 1\.0;

## 函数









## 常见程序设计流程

### PNP测距





### robomaster装甲板识别代码讲解

需求:用方框框住装甲板

分析:

观察视频，我发现装甲板有两条竖直平行的灯条在装甲板的左右两端。所以大致分析，解决该问题的关键就是通过筛选出视频中的灯条，通过计算像素坐标，来绘制矩形框住装甲板。

视频中的灯条可以看成一个旋转矩形RotatedRect，为了方便后续对灯条几何特征（角度/中心点等）进行成对匹配，我将矩形的各个属性：宽，长，中心，角度，面积，封装成一个灯条类。



将VideoCaputure初始化，用于读取视频中的每一帧



为了方便对图像中灯条的提取

我首先对图像进行了预处理

1\.由于视频中的灯条是红色的，选择红色通道进行二值化

2\.将图片进行阈值处理，灯条是图片中亮度较高的区域，阈值220可以有效提取明亮区域

3\.利用高斯模糊消除微小噪声点

4\.对图像进行膨胀操作，连接断裂的灯条区域，将灯条扩大，方便提取，使用5x5矩形结构元素element增强膨胀元素



接着检测灯条的轮廓

利用findContours函数读取预处理好的图像，获得存储灯条轮廓的点集

hierarchy表示输出各个轮廓的继承关系

RETR\_TREE表示检测所有轮廓，并且建立所有的继承关系，CHAIN\_APPROX\_NONE表示把轮廓的所有点存储



对轮廓进行处理与筛选

将面积太小，点数太少，长宽比太大也就是太过细长的灯条筛选掉



使用椭圆拟合函数**fitEllipse:返回旋转矩形**







## opencv常用100个API

1. 图像操作

cv2\.imread\(filename, flags\) \- 读取图像。

cv2\.imwrite\(filename, img\) \- 保存图像。

cv2\.imshow\(window\_name, img\) \- 显示图像。

cv2\.cvtColor\(src, code\) \- 转换图像颜色空间。

cv2\.resize\(src, dsize, fx, fy, interpolation\) \- 缩放图像。

cv2\.rotate\(src, rotateCode\) \- 旋转图像。

cv2\.flip\(src, flipCode\) \- 翻转图像。

cv2\.split\(src\) \- 拆分通道。

cv2\.merge\(mv\) \- 合并通道。

cv2\.copyMakeBorder\(src, top, bottom, left, right, borderType, value\) \- 添加边框。

2. 图像变换

cv2\.warpAffine\(src, M, dsize\) \- 仿射变换。

cv2\.getAffineTransform\(srcPoints, dstPoints\) \- 获取仿射变换矩阵。

cv2\.warpPerspective\(src, M, dsize\) \- 透视变换。

cv2\.getPerspectiveTransform\(srcPoints, dstPoints\) \- 获取透视变换矩阵。

cv2\.remap\(src, map1, map2, interpolation\) \- 重映射。

cv2\.resize\(src, dsize\) \- 调整大小。

cv2\.getRotationMatrix2D\(center, angle, scale\) \- 获取旋转矩阵。

cv2\.invertAffineTransform\(M\) \- 仿射矩阵求逆。

cv2\.convertScaleAbs\(src, alpha, beta\) \- 调整对比度和亮度。

cv2\.normalize\(src, dst, alpha, beta, norm\_type\) \- 归一化。

3. 绘图功能

cv2\.line\(img, pt1, pt2, color, thickness\) \- 画线。

cv2\.rectangle\(img, pt1, pt2, color, thickness\) \- 画矩形。

cv2\.circle\(img, center, radius, color, thickness\) \- 画圆。

cv2\.ellipse\(img, center, axes, angle, startAngle, endAngle, color, thickness\) \- 画椭圆。

cv2\.polylines\(img, pts, isClosed, color, thickness\) \- 画多边形。

cv2\.fillPoly\(img, pts, color\) \- 填充多边形。

cv2\.putText\(img, text, org, fontFace, fontScale, color, thickness\) \- 添加文本。

4. 图像阈值

cv2\.threshold\(src, thresh, maxval, type\) \- 图像二值化。

cv2\.adaptiveThreshold\(src, maxValue, adaptiveMethod, thresholdType, blockSize, C\) \- 自适应阈值。

cv2\.inRange\(src, lowerb, upperb\) \- 范围筛选。

5. 图像平滑与滤波

cv2\.blur\(src, ksize\) \- 均值滤波。

cv2\.GaussianBlur\(src, ksize, sigmaX\) \- 高斯滤波。

cv2\.medianBlur\(src, ksize\) \- 中值滤波。

cv2\.bilateralFilter\(src, d, sigmaColor, sigmaSpace\) \- 双边滤波。

cv2\.filter2D\(src, ddepth, kernel\) \- 任意核卷积。

6. 边缘检测与轮廓

cv2\.Canny\(image, threshold1, threshold2\) \- 边缘检测。

cv2\.findContours\(image, mode, method\) \- 查找轮廓。

cv2\.drawContours\(image, contours, contourIdx, color, thickness\) \- 绘制轮廓。

cv2\.arcLength\(contour, closed\) \- 计算轮廓周长。

cv2\.contourArea\(contour\) \- 计算轮廓面积。

cv2\.approxPolyDP\(curve, epsilon, closed\) \- 多边形逼近。

cv2\.boundingRect\(points\) \- 计算矩形边界。

cv2\.minEnclosingCircle\(points\) \- 最小包围圆。

cv2\.convexHull\(points\) \- 凸包。

cv2\.isContourConvex\(contour\) \- 判断是否为凸形。

7. 形态学操作

cv2\.erode\(src, kernel, iterations\) \- 腐蚀。

cv2\.dilate\(src, kernel, iterations\) \- 膨胀。

cv2\.morphologyEx\(src, op, kernel\) \- 形态学操作（开闭运算等）。

cv2\.getStructuringElement\(shape, ksize\) \- 获取结构元素。

8. 图像直方图

cv2\.calcHist\(images, channels, mask, histSize, ranges\) \- 计算直方图。

cv2\.equalizeHist\(src\) \- 直方图均衡化。

cv2\.createCLAHE\(clipLimit, tileGridSize\) \- 自适应直方图均衡化。

9. 特征检测与描述

cv2\.SIFT\_create\(\) \- SIFT特征检测。

cv2\.ORB\_create\(\) \- ORB特征检测。

cv2\.FastFeatureDetector\_create\(\) \- FAST特征检测。

cv2\.MSER\_create\(\) \- MSER特征检测。

cv2\.BRISK\_create\(\) \- BRISK特征检测。

cv2\.SimpleBlobDetector\_create\(\) \- 简单Blob检测。

cv2\.goodFeaturesToTrack\(src, maxCorners, qualityLevel, minDistance\) \- 检测角点。

10. 特征匹配

cv2\.BFMatcher\(normType\) \- 暴力匹配器。

cv2\.FlannBasedMatcher\(\) \- FLANN匹配器。

cv2\.drawMatches\(img1, kp1, img2, kp2, matches, outImg\) \- 绘制匹配结果。

11. 视频操作

cv2\.VideoCapture\(source\) \- 打开视频文件或摄像头。

cv2\.VideoWriter\(filename, fourcc, fps, frameSize\) \- 保存视频。

cap\.read\(\) \- 读取视频帧。

cap\.isOpened\(\) \- 检查视频是否打开。

cap\.release\(\) \- 释放视频资源。

12. 几何变换与数学操作

cv2\.addWeighted\(src1, alpha, src2, beta, gamma\) \- 图像加权。

cv2\.bitwise\_and\(src1, src2\) \- 按位与。

cv2\.bitwise\_or\(src1, src2\) \- 按位或。

cv2\.bitwise\_not\(src\) \- 按位取反。

cv2\.bitwise\_xor\(src1, src2\) \- 按位异或。

cv2\.minMaxLoc\(src\) \- 最值定位。

cv2\.reduce\(src, dim, rtype\) \- 归约操作。

13. 模板匹配

cv2\.matchTemplate\(image, templ, method\) \- 模板匹配。

cv2\.minMaxLoc\(result\) \- 获取匹配位置。

14. 深度学习相关

cv2\.dnn\.readNetFromCaffe\(protoTxt, model\) \- 读取Caffe模型。

cv2\.dnn\.readNetFromTensorflow\(model, config\) \- 读取TensorFlow模型。

cv2\.dnn\.readNetFromONNX\(model\) \- 读取ONNX模型。

cv2\.dnn\.blobFromImage\(image, scalefactor, size, mean, swapRB, crop\) \- 图像转换为深度学习输入。

15. 基本工具

cv2\.waitKey\(delay\) \- 等待键盘输入。

cv2\.destroyAllWindows\(\) \- 销毁所有窗口。

cv2\.getTickCount\(\) \- 获取时间戳。

cv2\.getTickFrequency\(\) \- 获取时间频率。

cv2\.setMouseCallback\(window\_name, callback\) \- 设置鼠标回调。

16. 深入功能

cv2\.calcOpticalFlowFarneback\(prev, next, flow, pyrScale, levels, winsize, iterations, polyN, polySigma, flags\) \- 光流计算。

cv2\.cornerHarris\(src, blockSize, ksize, k\) \- Harris角点检测。

cv2\.cornerSubPix\(image, corners, winSize, zeroZone, criteria\) \- 亚像素角点优化。

17. 自定义与扩展

cv2\.getTrackbarPos\(trackbarname, winname\) \- 获取滑块值。

cv2\.createTrackbar\(trackbarname, winname, value, count, onChange\) \- 创建滑块。

cv2\.fillConvexPoly\(img, points, color\) \- 填充凸多边形。

cv2\.fillPoly\(img, pts, color\) \- 填充多边形。

18. 图像与视频编码解码

cv2\.imencode\(ext, img\) \- 编码图像。

cv2\.imdecode\(buf, flags\) \- 解码图像。

cv2\.VideoWriter\_fourcc\(c1, c2, c3, c4\) \- 获取视频编码器。

19. 其他实用功能

cv2\.phase\(x, y\) \- 计算幅角。

cv2\.cartToPolar\(x, y\) \- 笛卡尔坐标到极坐标转换。

cv2\.polarToCart\(magnitude, angle\) \- 极坐标到笛卡尔坐标转换。

cv2\.kmeans\(data, K, bestLabels, criteria, attempts, flags\) \- KMeans 聚类。

cv2\.connectedComponents\(image\) \- 连通域分析。





## 补充API

#### 1\.glob

void cv::glob\(cv::String pattern, std::vector\<cv::String\>\& result, bool recursive = false\);

- **pattern**：文件路径模式，支持通配符（如 `*` 和 `?`）。例如，`"./data/*.jpg"` 表示获取 `data` 文件夹下所有扩展名为 `.jpg` 的文件。

- **result**：用于存储匹配路径的容器，类型为 `std::vector<cv::String>`。

- **recursive**：是否递归搜索子文件夹。默认为 `false`，表示仅搜索当前目录。

#### 2\.Size

`cv::Size` 指定图像尺寸

`cv::Size` 是一个简单的结构体，包含两个成员变量：`width` 和 `height`。它通常用于指定图像的尺寸，例如在 `cv::resize` 函数中

`cv::Size(-1, -1)` 的含义

在 OpenCV 的 `cv::resize` 函数中，`cv::Size` 的参数用于指定目标图像的大小。如果将 `cv::Size` 的宽度和高度都设置为 `-1`，这通常意味着目标图像的大小是通过缩放比例（`fx` 和 `fy`）来计算的，而不是直接指定目标尺寸。

例如:

cv::resize\(src, dst, cv::Size\(\-1, \-1\), fx, fy, interpolation\);

#### 3\.`findChessboardCorners`

在 OpenCV 中，`cv::findChessboardCorners` 是一个用于检测棋盘格角点的函数，广泛应用于相机标定和三维重建等任务中

bool cv::findChessboardCorners\(

InputArray image,          // 输入图像，必须是8位灰度或彩色图像

Size patternSize,          // 棋盘格的尺寸，表示内部角点的数量（例如8x6的棋盘格，patternSize为\(7,5\)）

OutputArray corners,       // 检测到的角点坐标

int flags = CALIB\_CB\_ADAPTIVE\_THRESH \+ CALIB\_CB\_NORMALIZE\_IMAGE // 操作标志

\);

- `image`：输入图像，必须是8位灰度或彩色图像。

- `patternSize`：棋盘格的尺寸，表示内部角点的数量（例如8x6的棋盘格，patternSize为\(7,5\)）。

- `corners`：检测到的角点坐标，存储为 `std::vector<cv::Point2f>`。

- `flags`：操作标志，可以组合以下值：

    - `CALIB_CB_ADAPTIVE_THRESH`：使用自适应阈值。

    - `CALIB_CB_NORMALIZE_IMAGE`：对图像进行归一化。

    - `CALIB_CB_FAST_CHECK`：快速检查图像是否包含棋盘格，如果未找到则提前退出

#### 4\.cornerSubPix//用于优化角点坐标//亚像素级精确定位

void cv::cornerSubPix\(

InputArray image,          // 输入图像，通常是单通道灰度图像。

InputOutputArray corners,  // 输入角点的初始坐标（例如由 `findChessboardCorners` 或 `goodFeaturesToTrack` 检测到的角点），优化后的角点坐标将直接输出到此参数\[^23^\]\[^24^\]。

Size winSize,              // 搜索窗口的一半尺寸。例如，`Size(5, 5)` 表示搜索窗口大小为 `(5*2+1)×(5*2+1)=11×11`\[^21^\]\[^23^\]。

Size zeroZone,             // 死区的一半尺寸，用于避免搜索区域的中心部分。值为 `(-1, -1)` 表示没有死区\[^21^\]\[^23^\]。

TermCriteria criteria      // 迭代过程的终止条件，可以是最大迭代次数或精度阈值\[^23^\]。

\);

参数说明

1. `image`：输入图像，必须是单通道灰度图像。

2. `corners`：角点的初始坐标（输入）和优化后的坐标（输出）。初始坐标通常由 `findChessboardCorners` 或 `goodFeaturesToTrack` 提供。

3. `winSize`：搜索窗口的一半尺寸，决定了角点优化时考虑的区域范围。

4. `zeroZone`：死区的一半尺寸，用于避免搜索区域的中心部分。值为 `(-1, -1)` 表示没有死区。

5. `criteria`：迭代终止条件，通常设置为 `TermCriteria::EPS + TermCriteria::MAX_ITER`，表示达到指定精度或最大迭代次数时停止。

#### 5\.`calibrateCamera`

在 OpenCV 中，`cv::calibrateCamera` 是一个用于相机标定的函数，通过一系列棋盘格图像来计算相机的内参和外参，以及畸变系数。以下是关于 `cv::calibrateCamera` 的使用方法和一个完整的示例代码。

double cv::calibrateCamera\(

InputArrayOfArrays objectPoints,    // 三维空间中的点坐标（通常是棋盘格的角点）

InputArrayOfArrays imagePoints,     // 图像中的对应点坐标（棋盘格角点的图像坐标）

Size imageSize,                     // 图像的尺寸

InputOutputArray cameraMatrix,      // 输出的相机内参矩阵

InputOutputArray distCoeffs,        // 输出的畸变系数

OutputArrayOfArrays rvecs,          // 每幅图像的旋转向量

OutputArrayOfArrays tvecs,          // 每幅图像的平移向量

int flags = 0,                      // 标定选项

TermCriteria criteria = TermCriteria\(TermCriteria::COUNT \+ TermCriteria::EPS, 30, DBL\_EPSILON\) // 迭代终止条件

\);

参数说明

1. `objectPoints`：三维空间中的点坐标，通常是棋盘格的角点。对于每幅图像，这些点的坐标是相同的。

2. `imagePoints`：检测到的棋盘格角点的图像坐标。

3. `imageSize`：图像的尺寸（宽和高）。

4. `cameraMatrix`：相机内参矩阵，输出结果。

5. `distCoeffs`：畸变系数，输出结果。

6. `rvecs`：每幅图像的旋转向量，表示相机的旋转。

7. `tvecs`：每幅图像的平移向量，表示相机的平移。

8. `flags`：标定选项，例如 `CALIB_FIX_PRINCIPAL_POINT`、`CALIB_FIX_ASPECT_RATIO` 等。

9. `criteria`：迭代优化的终止条件。

#### 6\.find4QuardCornerSubpix//用于优化角点坐标

`cv::find4QuadCornerSubpix` 是一个用于精确定位四边形四个角点亚像素位置的函数。它通常在已经通过其他方法（如 `cv::goodFeaturesToTrack` 或 `cv::cornerHarris`）粗略定位角点之后使用，以提高角点检测的准确性

bool cv::find4QuadCornerSubpix\(

InputArray img,

InputOutputArray corners,

Size region\_size

\);

1. `img`：输入图像，应为灰度图，类型为 8\-bit 或浮点型的单通道图像。

2. `corners`：输入/输出参数。初始的角点坐标作为输入，优化后的角点坐标作为输出。这是一个包含 `(x, y)` 坐标的浮点数向量。

3. `region_size`：搜索窗口大小。对于每个角点，将在这个区域内的子窗口中寻找更准确的位置。



#### 7\.drawChessboardCorners

`cv::drawChessboardCorners` 是 OpenCV 中用于在图像上绘制检测到的棋盘格角点的函数。它通常用于相机标定过程中，帮助可视化检测到的角点，以验证角点检测的准确性

void cv::drawChessboardCorners\(

InputOutputArray image,    // 目标图像，必须是8位彩色图像

Size patternSize,          // 棋盘格的内角点数，格式为 cv::Size\(columns, rows\)

InputArray corners,        // 检测到的角点数组，由 findChessboardCorners 函数输出

bool patternWasFound       // 指示是否成功检测到完整的棋盘格，应传入 findChessboardCorners 的返回值

\);

1. `image`：目标图像，必须是8位彩色图像。

2. `patternSize`：棋盘格的内角点数，格式为 `cv::Size(columns, rows)`，其中 `columns` 和 `rows` 分别是棋盘格的列数和行数（注意是内角点数，而非方格数）。

3. `corners`：检测到的角点数组，由 `findChessboardCorners` 函数输出。

4. `patternWasFound`：指示是否成功检测到完整的棋盘格，应传入 `findChessboardCorners` 的返回值。

#### 8\.getAffineTransform

计算仿射变换矩阵

- 仿射变换是一种二维坐标到二维坐标的线性变换，它保持了直线和平行性，但可以改变形状和大小。

- 需要三个点

cv::Mat cv::getAffineTransform\(const Point2f src\[\], const Point2f dst\[\]\);

cv::Mat cv::getAffineTransform\(InputArray src, InputArray dst\);

- `src`：源图像中三角形顶点的坐标，需要提供三个点。

- `dst`：目标图像中相应三角形顶点的坐标，与 `src` 中的点一一对应。

- 返回值：一个 2×3 的浮点型矩阵，表示从 `src` 到 `dst` 的仿射变换矩阵

使用步骤

1. **计算仿射变换矩阵**：通过 `getAffineTransform` 函数计算出源图像和目标图像之间的仿射变换矩阵。

2. **应用仿射变换**：使用 `warpAffine` 函数将计算出的仿射变换矩阵应用到图像上，实现图像的仿射变换。





#### 9\.getPerspectiveTransform

计算透视变换矩阵

- 透视变换是一种更复杂的变换，它将一个平面映射到另一个平面，可以改变直线的平行性，从而实现更复杂的几何变换。

- 透视变换矩阵是一个 **3×3** 的矩阵，形式如下：

- 需要四个点

OpenCV 中用于计算透视变换矩阵的函数。它通过给定的四个点对（源点和目标点）来计算从源图像到目标图像的透视变换矩阵。这个矩阵可以用于将图像从一个平面映射到另一个平面，实现更复杂的几何变换，例如将矩形图像映射为平行四边形或梯形。

cv::Mat cv::getPerspectiveTransform\(const Point2f src\[\], const Point2f dst\[\]\);

cv::Mat cv::getPerspectiveTransform\(InputArray src, InputArray dst\);



- `src`：源图像中的四个点的坐标，这些点必须是不共线的。

- `dst`：目标图像中对应的四个点的坐标，与 `src` 中的点一一对应。

- **返回值**：一个 3×3 的浮点型矩阵，表示从 `src` 到 `dst` 的透视变换矩阵。

使用步骤

1. **定义源点和目标点**：选择源图像和目标图像中的四个点。

2. **计算透视变换矩阵**：使用 `getPerspectiveTransform` 函数计算透视变换矩阵。

3. **应用透视变换**：使用 `warpPerspective` 函数将计算出的透视变换矩阵应用到图像上，实现图像的透视变换。

#### 10\.warpAffine

是 OpenCV 中用于应用仿射变换的函数。它通过一个 2×3 的仿射变换矩阵，将输入图像映射到输出图像。这种变换可以实现平移、旋转、缩放和剪切等操作。

- **仿射变换（Affine Transformation）**：

    - 仿射变换是一种二维坐标到二维坐标的线性变换，保持直线和平行性，但可以改变形状和大小。

    - 可以实现平移、旋转、缩放和剪切等操作。

    - **矩阵维度**：2×3 的矩阵。

    - `warpAffine`：

        - 适用于简单的几何变换，如平移、旋转、缩放和剪切。

        - 常用于局部变换，例如将图像的一部分旋转或缩放后嵌入到另一幅图像中。

        - 示例：将图像的一部分旋转 45 度并缩放 0\.5 倍。

        需要三个点对    三个点不能共线

void cv::warpAffine\(

InputArray src,          // 输入图像

OutputArray dst,         // 输出图像

InputArray M,            // 2×3 的仿射变换矩阵

Size dsize,              // 输出图像的大小

int flags = INTER\_LINEAR,// 插值方法

int borderMode = BORDER\_CONSTANT, // 边界填充模式

const Scalar\& borderValue = Scalar\(\) // 边界填充值

\);

参数说明

- `src`：输入图像，可以是任意通道数的单通道或多通道图像。

- `dst`：输出图像，其大小由 `dsize` 参数决定，类型与输入图像相同。

- `M`：2×3 的仿射变换矩阵，通常由 `getAffineTransform` 或其他方式计算得到。

- `dsize`：输出图像的大小，格式为 `cv::Size(width, height)`。

- `flags`：插值方法，常用的有：

    - `INTER_LINEAR`：双线性插值（默认值）。

    - `INTER_NEAREST`：最近邻插值。

    - `INTER_CUBIC`：双三次插值。

- `borderMode`：边界填充模式，常用的有：

    - `BORDER_CONSTANT`：用指定的 `borderValue` 填充边界。

    - `BORDER_REPLICATE`：复制边缘像素。

    - `BORDER_REFLECT`：反射边缘像素。

- `borderValue`：当 `borderMode` 为 `BORDER_CONSTANT` 时，用于填充边界的值，默认为黑色（0）。

#### 11\.warpPerspective

- 适用于更复杂的几何变换，如透视校正、文档扫描、3D 效果等。

- 常用于将图像从一个平面映射到另一个平面，例如将倾斜的文档图像校正为正面视图。

- 示例：将矩形图像变换为梯形或平行四边形。

需要四个点对    这四个点不能共线,且不能共面

- **透视变换（Perspective Transformation）**：

    - 透视变换是一种更复杂的变换，可以改变直线的平行性，从而实现更复杂的几何变换，例如将矩形变换为梯形或平行四边形。

    - 适用于模拟三维空间中的视角变化，例如文档扫描、透视校正等。

    - **矩阵维度**：3×3 的矩阵。

`warpPerspective` 是 OpenCV 中用于应用透视变换的函数。它通过一个 3×3 的透视变换矩阵，将输入图像映射到输出图像。这种变换可以实现图像的倾斜、扭曲或视角变化，通常用于模拟三维空间中的视角变化

void cv::warpPerspective\(

InputArray src,          // 输入图像

OutputArray dst,         // 输出图像

InputArray M,            // 3×3 的透视变换矩阵

Size dsize,              // 输出图像的大小

int flags = INTER\_LINEAR,// 插值方法

int borderMode = BORDER\_CONSTANT, // 边界填充模式

const Scalar\& borderValue = Scalar\(\) // 边界填充值

\);

参数说明

- `src`：输入图像，可以是任意通道数的单通道或多通道图像。

- `dst`：输出图像，其大小由 `dsize` 参数决定，类型与输入图像相同。

- `M`：3×3 的透视变换矩阵，通常通过 `getPerspectiveTransform` 函数计算得到。

- `dsize`：输出图像的大小，格式为 `cv::Size(width, height)`。

- `flags`：插值方法，常用的有：

    - `INTER_LINEAR`：双线性插值（默认值）。

    - `INTER_NEAREST`：最近邻插值。

    - `INTER_CUBIC`：双三次插值。

- `borderMode`：边界填充模式，常用的有：

    - `BORDER_CONSTANT`：用指定的 `borderValue` 填充边界。

    - `BORDER_REPLICATE`：复制边缘像素。

- `borderValue`：当 `borderMode` 为 `BORDER_CONSTANT` 时，用于填充边界的值，默认为黑色（0）。

1. **计算透视变换矩阵**：使用 `getPerspectiveTransform` 函数计算 3×3 的透视变换矩阵。

2. **调用 **`warpPerspective`：将计算得到的矩阵应用到输入图像上，生成输出图像。



12\.norm

`cv::norm` 函数用于计算矩阵或向量的范数。它是一个非常有用的工具，可以用于测量向量的长度、矩阵的大小，或者计算两个矩阵之间的差异

double cv::norm\(InputArray src1, int normType = NORM\_L2, InputArray mask = noArray\(\)\);

- `src1`：输入矩阵或向量。

- `normType`：范数类型，默认为 `NORM_L2`，即欧几里得范数。其他常见范数类型包括：

    - `NORM_L1`：L1 范数，即绝对值之和。

    - `NORM_L2`：L2 范数，即欧几里得范数。

    - `NORM_INF`：无穷范数，即最大绝对值。

    - `NORM_HAMMING`：汉明距离。

- `mask`：可选参数，用于指定计算范数时的掩码。



#### 12\.solvePnP

bool solvePnP\(InputArray objectPoints, InputArray imagePoints, InputArray cameraMatrix, InputArray distCoeffs, OutputArray rvec, OutputArray tvec, bool useExtrinsicGuess = false, int flags = SOLVEPNP\_ITERATIVE\);

- `objectPoints`：目标物体的 3D 点坐标，类型为 `vector<Point3f>` 或 `Mat`。

- `imagePoints`：目标物体在图像中的 2D 投影点坐标，类型为 `vector<Point2f>` 或 `Mat`。

- `cameraMatrix`：相机的内参矩阵，类型为 `Mat`。

- `distCoeffs`：相机的畸变系数，类型为 `Mat`。

- `rvec`：输出的旋转向量（Rodrigues 表示法），类型为 `Mat`。

- `tvec`：输出的平移向量，类型为 `Mat`。

- `useExtrinsicGuess`：是否使用初始的外参估计值。如果为 `true`，则 `rvec` 和 `tvec` 会被用作初始猜测值。

- `flags`：指定 PnP 算法的类型，常见的选项包括：

    - `SOLVEPNP_ITERATIVE`：使用非线性优化方法（默认）。

    - `SOLVEPNP_P3P`：使用 P3P 算法（至少需要 3 个点）。

    - `SOLVEPNP_UPNP`：使用 UPnP 算法。

    - `SOLVEPNP_DLS`：使用 DLS 算法（至少需要 2 个点）。

    - `SOLVEPNP_AP3P`：使用 AP3P 算法。

#### 13\.norm

在 OpenCV 的 C/C\+\+ 接口中，`norm` 函数用于计算数组的范数，它在图像处理和计算机视觉中非常有用，例如用于计算图像之间的差异或特征向量的长度。以下是关于 `norm` 函数的详细说明：

double cv::norm\(InputArray src1, InputArray src2 = noArray\(\), int normType = NORM\_L2, InputArray mask = noArray\(\)\);

- **参数说明**：

    - `src1`：输入数组（图像或矩阵）。

    - `src2`：可选的第二个输入数组，如果提供，则计算两个数组之间的范数。

    - `normType`：范数类型，默认为 `NORM_L2`，可选值包括：

        - `NORM_INF`：无穷范数，即最大绝对值。

        - `NORM_L1`：L1 范数，即绝对值之和。

        - `NORM_L2`：L2 范数，即欧几里得范数。

        - `NORM_L2SQR`：L2 范数的平方。

        - `NORM_HAMMING`：汉明范数，适用于二进制数据。

        - `NORM_HAMMING2`：汉明范数的变体。

        - `NORM_MINMAX`：归一化范数。

    - `mask`：可选的掩码，用于指定哪些元素参与计算。

## API区别和异同

#### `minAreaRect` 和 `fitEllipse` 是 OpenCV 中用于轮廓拟合的两种不同方法，它们的主要区别如下：



1\. 功能定义

**`minAreaRect`**：用于计算能够完全包围输入点集（通常是轮廓）的最小面积矩形。这个矩形可以是旋转的，因此能够更好地适应不规则形状。

**`fitEllipse`**：用于拟合一个椭圆，使其最优地匹配输入的点集（通常是轮廓）。这个椭圆能够更好地描述轮廓的形状特征。



2\. 返回值

**`minAreaRect`**：返回一个 `RotatedRect` 对象，包含以下信息：

矩形的中心点坐标。

矩形的宽度和高度。

矩形的旋转角度。

**`fitEllipse`**：返回一个椭圆的参数，包括：

椭圆的中心点坐标。

椭圆的主轴和次轴长度。

椭圆的旋转角度。



3\. 使用场景

**`minAreaRect`**：适用于需要最小面积矩形来包围轮廓的场景，例如目标检测、物体定位等。它能够提供更紧凑的包围形状。

**`fitEllipse`**：适用于需要描述轮廓的形状特征或进行椭圆拟合的场景，例如在医学图像分析中拟合细胞形状。



4\. 输入要求

**`minAreaRect`**：输入为一组二维点集（通常是轮廓的点集）。

**`fitEllipse`**：输入同样为一组二维点集，但要求点集的数量至少为 5 个。



5\. 输出形状

**`minAreaRect`**：输出是一个旋转矩形，可以通过 `cv2.boxPoints` 获取其四个顶点。

**`fitEllipse`**：输出是一个椭圆，可以通过 `cv2.ellipse` 绘制。



总结

如果目标是找到最小面积的矩形包围框，选择 `minAreaRect`。

如果目标是拟合一个椭圆来描述轮廓的形状，选择 `fitEllipse`。



希望这些信息能帮助你理解两者的区别。





