---
title: "AI辅助解决弹道复现BUG"
slug: "ai辅助解决弹道复现bug"
date: 2026-06-13T11:05:24+08:00
draft: false
source_file: "feishu://ai辅助解决弹道复现bug"
source_size: 488161
source_lines: 7219
tags:
  - "AI"
  - "机器人"
categories:
  - "机器人视觉"
---

AI辅助解决弹道复现BUG
#pragma once
#include <vector>
// ROS相关头文件
#include "ros/ros.h"
#include "rm_msgs/Armor.h"
#include "rm_msgs/ArmorArray.h"
#include "rm_msgs/RmSerial.h" 
#include <std_msgs/Float64.h>
#include <angles/angles.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Quaternion.h>
// OpenCV相关头文件
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
// 标准库头文件
#include <iostream>
#include <cmath>
// 时间相关头文件
#include <chrono>
#include <thread>
#include <fstream>
#include <yaml-cpp/yaml.h>
// 自定义头文件
#include "Calculater.hpp"
#include "GimbalPos.hpp"
#include "TargetModel.hpp"
#include "visual.hpp"
#include "CoorConverter.hpp"
#include "math.hpp"
#include "MPC.hpp"
#include "trajectory_visualizer.hpp"
using namespace std;
using namespace cv;
/*
    自瞄文档
    标准模型状态向量
    X(0) -> 机器人中心的x坐标
    X(1) -> 机器人中心x方向速度
    X(2) -> 机器人中心的y坐标
    X(3) -> 机器人中心y方向速度
    X(4) -> 左侧装甲板的固定高度
    X(5) -> 右侧装甲板的固定高度
    X(6) -> 小装甲板旋转半径（左侧）
    X(7) -> 大装甲板旋转半径（右侧）
    X(8) -> 机器人整体偏航角yaw
    X(9) -> 偏航角速度palstance
    前哨站状态向量
    X(0): 机器人中心的x坐标
    X(1): 机器人中心的y坐标
    X(2): 第一块装甲板高度h1
    X(3): 第二块装甲板高度h2
    X(4): 第三块装甲板高度h3
    X(5): 偏航角yaw
    X(6): 偏航角速度palstance
*/
/**
@brief 将弧度约束在[-pi, pi]范围内，大于n，减去2n；小于-n，加上2n
@param angle 角度 
*/
#ifndef _std_radian
#define _std_radian(angle) ((angle) + round((0 - (angle)) / (2 * PI)) * (2 * PI))
#endif
//追踪类
template<TimeSpan _trackTime>
class Tracker{
private:
    double COMMAND_TIMESPAN;            //电控延迟
    double local_gravity_;              //重力加速度
    double eTime;                       //曝光时间
    /* ======================== 系统参数 ======================== */
    //ROS相关
    ros::NodeHandle nh;                 //ROS节点句柄
    ros::Publisher debugpub;            //debug发布者
    ros::Publisher debugpub1;           //debug1发布者
    std_msgs::Float64 debugdate;        //debug数据
    std_msgs::Float64 debugdate1;       //debug1数据
    rm_msgs::RmSerial RmSerialData;     //接收串口数据
    tf2_ros::Buffer tfBuffer_;            // TF 缓冲区
    tf2_ros::TransformListener tfListener; // 声明一个tf2_ros::TransformListener对象，并传入tfBuffer
    rm_msgs::ArmorArrayConstPtr m_armors;  // 接收装甲板数据 
    //图像处理相关
    cv::Mat frame;          //接收的原始图像
    cv::Mat frame_;         //被处理的图像副本(保护原图像)
    cv::Mat camera_matrix_; //相机内参(构造函数中)
    cv::Mat dist_coeffs_;   //畸变系数(构造函数中)
    Image img;              //图像工具
    //坐标变换工具
    CAL::Calculater* cal;   //计算工具
    CoordinateTransformer* coorConverter;
    //装甲板评分参数
    std::vector<int> col;                       // 数列中的每个数代表矩阵的每一列
    std::vector<int> row;                       // 数列中的每个数代表矩阵的每一行
    std::vector<int> tmp_v;                     // 存储C(n,k)的中间结果
    std::vector<std::vector<int>> result;       // 存储C(n,k)的结果
    std::vector<std::vector<int>> nAfour;       // 存储A(n,4)的结果
    std::vector<std::vector<int>> fourAfour;    // 存储A(4,4)的结果
    std::map<int, int> row_col;                 // 存储最终结果，row_col[i]=j表示矩阵的第i行第j列是要选取的数
    //数据关联参数
    double min = 0, tmp = 0;
    /* ======================== 跟踪控制参数 ======================== */
    //基本跟踪参数
    char TrackingID;            //跟踪中的装甲板ID
    bool Switch_Armor;          //装甲板切换标识符
    double trackTime;           //目标丢失判定时间(秒)
    //角度补偿参数
    double pitch_compensation;  //pitch补偿
    double yaw_compensation;    //yaw补偿
    //火力控制参数
    bool all_fire = false;      //完全火力模式(不停止射击)
    /* ======================== 装甲板预测参数 ======================== */
    //空间参考点参数
    Eigen::Vector3d pegPos;         //空间标准点(用于装甲板跳变判断)
    double peg_point_pixel_now;     //当前装甲板在像素坐标系的位置
    double peg_point_pixel_last;    //位于像素标准坐标系的上一个装甲板位置
    //子弹参数
    double BulletVector = 22;           //子弹速度(初值给24m/s)
    /* ======================== 时间戳管理 ======================== */
    //装甲板跟踪时间
    long long Now_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//当前时间戳(用于计算丢失时间)
    long long Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//丢失时间戳(用于计算丢失时间)
    //帧率计算
    long long begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
    long long end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间
    // C++工具时间戳
    long long tool_begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//首时间
    long long tool_end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//尾时间
    // ros工具时间戳
    ros::Time tool_begin_Time_ros = ros::Time::now();
    ros::Time tool_end_Time_ros = ros::Time::now();
    // 获取装甲板tf时间(判断是否是相同帧)
    ros::Time last_tf_time = ros::Time::now();
    /* ======================== 标志位 ======================== */
    bool functional = true;                 // 射击模式
    int m_center_tracked;                   // 锁中心状态标志位
    bool m_track_center;                    // 是否跟随中心
    bool m_fix_on;                          // 重力补偿开关
    std::string config_path_;               // 存储配置路径
    unique_ptr<TargetModel> targetModel;    // 目标模型
    ros::Publisher AngPub;                  // 角度话题发布者
    std::shared_ptr<MPC> m_MPC;             // 模型预测控制器
    // =========================== 阈值 ====================================
    double m_score_tolerance;               //装甲板匹配得分最大值
    double m_switch_threshold;              // 更新装甲板切换的角度阈值，角度制
    double m_force_aim_palstance_threshold; // 强制允许发射的目标旋转速度最大值，弧度制
    double m_aim_angle_tolerance;           // 自动击发时目标装甲板相对偏角最大值，角度制
    double m_aim_pose_tolerance;            // 自动击发位姿偏差最大值，弧度制
    double m_aim_center_angle_tolerance;    // 跟随圆心自动击发目标偏角判断，角度制
    double m_switch_trackmode_threshold;    // 更换锁中心模式角速度阈值，弧度制
    double m_aim_center_palstance_threshold;// 跟随圆心转跟随装甲板的目标旋转速度最大值，弧度制
    // =========================== 可视化 ===================================
    std::vector<std::vector<Eigen::Vector3d>> visual_armor_position_pose_temp;  // 用于可视化存储观测装甲板的全局变量
    GimbalPose m_cur_pose;                  // 当前位姿
    GimbalPose m_target_pose;               // 目标位姿
    GimbalPose m_target_pose_debug;         // 用于debug,比较mpc和传统模式
    // =========================== 串口补偿 =======================================
    double m_rollOffset = 0;
    double m_pitchOffset = 0;
    double m_yawOffset = 0;
public:
    Tracker(ros::NodeHandle& nh,const std::string& config_path) : config_path_(config_path), tfListener(tfBuffer_)
    {
        this->nh = nh;
        AngPub = nh.advertise<geometry_msgs::Vector3>("/auto_angle", 1000);
        debugpub = nh.advertise<std_msgs::Float64>("/debugpub", 1000);
        debugpub1 = nh.advertise<std_msgs::Float64>("/debugpub1", 1000);
        cal = new CAL::Calculater(this->nh,this->img);
        tfBuffer_.setUsingDedicatedThread(true);
        m_armors.reset(); // 显式初始化为空
        // 初始化时创建 TargetModel
        targetModel = std::make_unique<TargetModel>(config_path_);
        coorConverter = new CoordinateTransformer(config_path_);
        m_MPC = std::make_shared<MPC>(config_path_); // 控制器初始化
        // 检查配置文件路径是否有效
        if (!config_path_.empty()) {
            try {
                setParam(config_path_);
                ROS_INFO("Successfully loaded parameters from: %s", config_path_.c_str());
            } catch (const std::exception& e) {
                ROS_ERROR("Failed to load parameters: %s", e.what());
            }
        } else {
            ROS_WARN("No configuration file path provided. Using default parameters.");
        }
    }
    ~Tracker() {
        delete cal;
        delete coorConverter;
    }
    /****************
    * @brief 回调函数: 接受串口信息，并更新内部变量、TF、坐标系
    * @param _serial ROS 消息的智能指针，包含子弹速度、射击标志、补偿角度等
    */
    void SetSerial(const rm_msgs::RmSerialConstPtr _serial){
        if (_serial) {
            RmSerialData = *_serial;
            // 目前注释掉了(到时候测试一下)
            // if(_serial->BulletVec > 10){
            //     BulletVector = _serial->BulletVec;
            // }
            if(_serial->ShootFlag == 'f'){
                functional = true;
            }else if(_serial->ShootFlag == 'a'){
                functional = true;
            }else{
                functional = true;
            }
            // 从串口获取当前装甲板的姿态
            m_cur_pose.roll = RmSerialData.Roll;
            m_cur_pose.pitch = RmSerialData.Pitch;
            m_cur_pose.yaw = RmSerialData.Yaw;    
            // debug
            // ROS_INFO("cur: pitch:  %lf" , RmSerialData.Pitch);
            // ROS_INFO("BulletVector:  %lf" , BulletVector);
            // std::vector<Eigen::Vector3d> imuabsPos = cal->GetPos("map","imu",1);
            // cal->TFUpdata("map","imuabs",{0.0, 0.0, 0.0},{0.0, 0.0, imuabsPos[1].z()},0);
            // ROS_WARN("have serial 3333333333333333333333333333");
        }
    }
    /*****************
    * @brief 回调函数: 接收相机节点发送的图像,用于debug
    * @param img_ 图像信息
    * @param frame  供算法线程直接使用的原始图
    * @param frame_ 额外再 clone 一份，用于可视化
    */
    void doimage(const sensor_msgs::ImageConstPtr img_)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr =  cv_bridge::toCvCopy(img_, sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        frame = cv_ptr->image.clone();
        frame_ = frame.clone();
        // ROS_WARN("have image 111111111111111111");
    }
    // 回调函数: 接收识别发送的装甲板序列
    void doArmors(rm_msgs::ArmorArrayConstPtr armors)
    {
        m_armors = armors;
        // ROS_WARN("have armor 222222222222222222");
    }
    /**
    * @brief  主跟踪循环：每帧调用一次，完成“决策 → 预测 → 补偿 → 发布”全链路
    */
    void Track()
    {
        if (!m_armors) {
            cout << "消息队列为空" << endl;
            return;
        }
        try
        {
            //begin_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            if(!functional){
                return;
            }
            TargetModel* targetModel_temp = nullptr;
            targetModel_temp = armorUpdate(m_armors);
            if(targetModel_temp != nullptr){
                cv::Point2d finAngle = {0.0, 0.0};
                GimbalPose target_pose_temp = reconstruction_choose_compensation();
                // 调试输出(打印云台需要转动到的角度)
                // cout << "pitch: " << target_pose_temp.pitch << " " << "yaw: " << target_pose_temp.yaw << endl;
                // 打印需要移动的相对角
                // finAngle.x = target_pose_temp.pitch - m_cur_pose.pitch; // 计算需要移动的pitch角度
                // finAngle.y = target_pose_temp.yaw - m_cur_pose.yaw;     // 计算需要移动的yaw角度
                // // // 陀螺仪的绝对角
                finAngle.x = target_pose_temp.pitch; // 计算需要移动到的pitch角度
                finAngle.y = target_pose_temp.yaw;     // 计算需要移动到的yaw角度
                // debug: 云台跟随效果rqt_plot打印
                // yaw角
                // debugdate.data = m_cur_pose.yaw;            // 当前云台yaw角度
                // debugdate1.data = target_pose_temp.yaw;     // 计算出云台需要转动的yaw角度
                // // pitch角
                // debugdate.data = m_cur_pose.pitch;          // 当前云台pitch角度
                // debugdate1.data = target_pose_temp.pitch;   // 计算出云台需要转动的pitch角度
                // debug: 自动打弹打印
                // cout << "finAngle.x: " << finAngle.x << " " << "finAngle.y: " << finAngle.y << endl;
                // cout << "是否允许打弹: " << targetModel_temp->auto_fire << endl;
                // if (targetModel_temp->auto_fire) {
                //     debugdate.data = 1;
                // }
                // else {
                //     debugdate.data = 0;
                // }
                Pub_Aangle(true, targetModel_temp->auto_fire, finAngle);
            }else{
                Pub_Aangle(false);
            }
            // 帧率控制
            // double time_line = 30.0;
            // end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            // double frame_time = (end_Time - begin_Time)/1000.0;
            // if ((end_Time - begin_Time) - time_line < 0.0)// 帧率控制
            // {
            //     int sleep_time_ms = static_cast<int>(time_line - (end_Time - begin_Time));
            //     std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            //     //frame_time = 0.03;
            //     end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            //     frame_time = (end_Time - begin_Time)/1000.0;
            //     // cout << "sleep_time_ms:" << sleep_time_ms << "ms" << endl;
            //     // cout << "frame_time:" << frame_time
1000 << "ms" << endl;
            // }
            // begin_Time = end_Time;
            // debugdate1.data = frame_time
1000; 
            debugpub.publish(debugdate);
            debugpub1.publish(debugdate1);
        }
        catch (const std::exception& e)
        {
            ROS_ERROR("[Exception] In Track function: %s", e.what());
        }
    }
    // 重构选板
    GimbalPose reconstruction_choose_compensation() {
        // 目标装甲板
        Armor abs_facing_armor;
        Armor abs_target_armor;
        double hit_time = 0;
        double center_hit_time = 0;
        double m_time_off = COMMAND_TIMESPAN + eTime + 0.025;
        /**
         * 严格意义上来说，如果要准确预测击中时刻的装甲板位置的话，需要解一个非线性方程。此处采用一种近似的解法
         * 根据当前最近装甲板距离计算击中时间，用于预测目标装甲板出现的位置
         * 事实上相当于一步牛顿迭代法，或者说一阶的线性化
         
/
        // 选板模式
        /
        这段代码有两种选板模式:
            1.锁中心模式:
                条件: 目标旋转角速度绝对值超过阈值
            2.瞄准最佳装甲板模式:
                条件: 目标旋转角速度绝对值低于或接近阈值
        */
        // 计算击打时间
        Point3d point_temp1 = Point3d(targetModel->m_status->getFacingArmor(0).position.x(),targetModel->m_status->getFacingArmor(0).position.y(),targetModel->m_status->getFacingArmor(0).position.z());
        center_hit_time = getDistance(point_temp1) / BulletVector + m_time_off;
        Point3d point_temp2 = Point3d(targetModel->m_status->getClosestArmor(0, 0).position.x(),targetModel->m_status->getClosestArmor(0, 0).position.y(),targetModel->m_status->getClosestArmor(0, 0).position.z());
        hit_time = getDistance(point_temp2) / BulletVector + m_time_off;
        // 选板
        abs_facing_armor = targetModel->m_status->getFacingArmor(center_hit_time);
        abs_target_armor = targetModel->m_status->getClosestArmor(hit_time, m_switch_threshold);
        // 判断是否锁中心
        if (m_track_center && ((abs(targetModel->m_status->palstance) - m_switch_trackmode_threshold) > m_aim_center_palstance_threshold)){
            m_center_tracked = true;    // 允许锁中心
            // ROS_INFO("锁中心");
        }
        else {
            m_center_tracked = false;   // 不允许锁中心
            // ROS_INFO("不锁中心");
        }
        /**
         * 判断自动击发，条件如下：
         * 1. 目标旋转速度小于一定阈值，或目标装甲板相对偏角不超过一定范围
         * 2. EKF先验稳定
         * 3. 没有长时间丢识别
         * 4. 当前云台跟随稳定，即当前位姿和目标位姿相近
         */
        cv::Point2d vector_c = targetModel->m_status->center;                                                                                                                   // 目标旋转中心本身的向量
        cv::Point2d vector_a = cv::Point2d(targetModel->m_status->center.x - abs_target_armor.position.x(), targetModel->m_status->center.y - abs_target_armor.position.y());   // 从旋转中心指向当前要瞄准的具体装甲板的向量
        double abs_angle = R2D(acos((vector_c.x * vector_a.x + vector_c.y * vector_a.y) / (getDistance(vector_c) * getDistance(vector_a))));                                    // 计算上述两个向量之间的夹角
        // debug
        // cout << "角速度" << targetModel->m_status->palstance << " " << "角速度阈值" << m_force_aim_palstance_threshold << endl;     // 目标旋转角速度低于强制瞄准阈值
        // cout << "夹角" << abs_angle << " " << "夹角阈值" << m_aim_angle_tolerance << endl;                                         // 装甲板偏角度
        // cout << "云台和目标姿态差" << (m_target_pose - m_cur_pose).norm() << " " << "姿态差阈值" << m_aim_pose_tolerance << endl;   // 云台和目标姿态差(云台跟随效果)
        // cout << "!Switch_Armor:" << !Switch_Armor << endl;
        // 目标的旋转角度低于强制瞄准阈值或计算出装甲板偏角小于容差,这意味着要么目标转得慢,容易瞄准; 要么即使目标目标在快速旋转,但刚好有一个装甲板转到了非常正对摄像头(夹角很小),也是很好的设计时机
        // 观测器,目标持续可见,云台跟随良好(云台当前的实际姿态与需要旋转的目标之间的差异很小,说明云台基本对准目标)
        // 由于电控发送过来的角度是上面是负下面是正
        // 而我们计算出来的角度是上面是正下面是负数
        // 由于火控的判断需要和电控发送过来的角度进行比较
        // 为了保证逻辑的完整性,所以在火控这里创建局部变量,将pitch角变为负数
        // 然后不影响整体逻辑,整体的逻辑在发送角度的地方pitch角度给负号
        GimbalPose m_target_pose_temp = m_target_pose;
        m_target_pose_temp.pitch = -m_target_pose_temp.pitch;
        // debugdate.data =   m_cur_pose.pitch;           // rqt预测x车中心
        // debugdate1.data =  m_target_pose_temp.pitch;   // rqt预测x车中心
        // debugdate.data =   m_cur_pose.yaw;           // rqt预测x车中心
        // debugdate1.data =  m_target_pose_temp.yaw;   // rqt预测x车中心
        // 自动开火
        if ((abs(targetModel->m_status->palstance) < m_force_aim_palstance_threshold || abs_angle < m_aim_angle_tolerance) &&
            targetModel->ekf->stable() && !Switch_Armor &&
            (m_target_pose_temp - m_cur_pose).norm() < m_aim_pose_tolerance
            )
        {
            targetModel->auto_fire = true;
            if (m_center_tracked && !(abs_angle < m_aim_center_angle_tolerance))
            {
                targetModel->auto_fire = false;
            }
        }
        else
        {
            targetModel->auto_fire = false;
        }
        // 云台控制模式
        bool control_mode = 0; // 0: 普通模式 1: mpc模式
        if (control_mode == 0) {
            if (m_center_tracked) {
                m_target_pose = getAngle(Point3d(abs_facing_armor.position.x(),abs_facing_armor.position.y(),abs_facing_armor.position.z()), m_cur_pose, BulletVector);
                // projectMapPointsToImage(targetModel->m_status->getArmors(center_hit_time),visual_armor_position_pose_temp);   // 这里打印的是计算的子弹飞行击中的时间的那一刻的重构装甲板
                projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);                    // 这里打印的重构装甲板是和观测装甲板同一时间的预测装甲板
                // // debug
                // Armors armors_temp_debug = targetModel->m_status->getArmors(center_hit_time);
                // for (int i = 0; i < armors_temp_debug.size(); i++) {
                //     cal->TFUpdata("map","armor" + std::to_string(i), armors_temp_debug[i].position, armors_temp_debug[i].rpy, i);
                // }
                // cal->TFUpdata("map","carcenter",{targetModel->m_status->center.x,targetModel->m_status->center.y,0},{0.0, 0.0, 0.0},0);//上传目标的TF
                // debug(弹道打印)
                // 记录当前时间
                tools::ShootParam shoot_param;
                shoot_param.v0 = BulletVector;
                shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
                shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_facing_armor.position);
                long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                tools::draw_simulated_bullets(this->coorConverter,shoot_param,frame_,draw_visual_now_time);
            }
            else {
                m_target_pose = getAngle(Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z()), m_cur_pose, BulletVector);
                // projectMapPointsToImage(targetModel->m_status->getArmors(hit_time),visual_armor_position_pose_temp);     // 这里打印的是计算的子弹飞行击中的时间的那一刻的重构装甲板
                projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);               // 这里打印的重构装甲板是和观测装甲板同一时间的预测装甲板
                // // debug
                // Armors armors_temp_debug = targetModel->m_status->getArmors(hit_time);
                // for (int i = 0; i < armors_temp_debug.size(); i++) {
                //     cal->TFUpdata("map","armor" + std::to_string(i), armors_temp_debug[i].position, armors_temp_debug[i].rpy, i);
                // }
                // cal->TFUpdata("map","carcenter",{targetModel->m_status->center.x,targetModel->m_status->center.y,0},{0.0, 0.0, 0.0},0);//上传目标的TF
                // debug(弹道打印)
                // 记录当前时间
                tools::ShootParam shoot_param;
                shoot_param.v0 = BulletVector;
                shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
                shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);
                long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
                tools::draw_simulated_bullets(this->coorConverter,shoot_param,frame_,draw_visual_now_time);
            }
        }
        else {
            // 西工大轨迹规划器
            m_target_pose = m_MPC->getGimbalSpeed(targetModel->m_status,m_cur_pose,BulletVector);
        }
        // // debug
        // /////////////////////////////////////////////
        // // 云台控制模式
        // if (m_center_tracked) {
        //     m_target_pose = getAngle(Point3d(abs_facing_armor.position.x(),abs_facing_armor.position.y(),abs_facing_armor.position.z()), m_cur_pose, BulletVector);
        //     // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);
        // }
        // else {
        //     m_target_pose = getAngle(Point3d(abs_target_armor.position.x(),abs_target_armor.position.y(),abs_target_armor.position.z()), m_cur_pose, BulletVector);
        //     // projectMapPointsToImage(targetModel->m_status->getArmors(0),visual_armor_position_pose_temp);
        // }
        // // 西工大轨迹规划器
        // m_target_pose_debug = m_MPC->getGimbalSpeed(targetModel->m_status,m_cur_pose,BulletVector);
        // // cout << "状态: " << targetModel->m_status->palstance << endl;
        // // cout << "弹速: " << BulletVector << endl;
        // // cout << "当前云台状态: " << "pitch: " << m_cur_pose.pitch << "yaw: " << m_cur_pose.yaw << endl;
        // // debugdate.data = m_target_pose.yaw;           // 未规划
        // // debugdate1.data = m_target_pose_debug.yaw;    // mpc规划
        // cout << "未规划yaw: " << m_target_pose.yaw << endl;
        // cout << "mpc规划yaw: " << m_target_pose_debug.yaw << endl;
        // // debugdate.data = m_target_pose.pitch;           // 未规划
        // // debugdate1.data = m_target_pose_debug.pitch;    // mpc规划
        // // cout << "未规划pitch: " << m_target_pose.pitch << endl;
        // // cout << "mpc规划pitch: " << m_target_pose_debug.pitch << endl;
        // /////////////////////////////////////////////
        return m_target_pose;
    }
    /**
     *@brief: 根据装甲板坐标、当前云台位姿、射速等信息计算目标位姿
     */
    GimbalPose getAngle(cv::Point3d position, GimbalPose cur_pose, double bullet_speed)
    {
        GimbalPose solve_angle;
        cv::Point3d gun_base = position;
        double pitch_in_gun = std::atan(gun_base.z / std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y));
        double yaw_in_gun   = std::atan(gun_base.y / gun_base.x);
        solve_angle.pitch =  pitch_in_gun;
        solve_angle.yaw   =  yaw_in_gun;
        // 重力补偿
        if (m_fix_on) {
            double dis = sqrt(pow(gun_base.x, 2) + pow(gun_base.z, 2) + pow(gun_base.y, 2));
            double angle_fix = 0;
            // debug /////////////////////////////////////////////////////////////
            // cout << "dis: " << dis << endl;
            // cout << "disxy: " << std::sqrt(gun_base.x * gun_base.x + gun_base.y * gun_base.y) << endl;
            // cout << "position.z" << position.z << endl;
            // cout << "pitch: " << solve_angle.pitch << endl;
            //////////////////////////////////////////////////////////////////////
            if (abs(9.8 * dis * pow(cos(-solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(-solve_angle.pitch) <= 1)
            {
                angle_fix = 0.5 * (asin((9.8 * dis * pow(cos(-solve_angle.pitch), 2)) / pow(bullet_speed, 2) - sin(-solve_angle.pitch)) - solve_angle.pitch);
                // cout << "重力补偿角: " << angle_fix << endl;
            }
            else
            {
                std::cout << "[AngleSolver] Wrong distance in angle_fix" << std::endl;
            }
            solve_angle.pitch += angle_fix;
        }
        // debug 判断枪口是否能够正确瞄准装甲板
        // //////////////////////////////////////////////////////////////////////
        // std::vector<Eigen::Vector3d> Postest = cal->GetPos("gun","DETECT4",1);
        // cal->TFUpdata("gun","DETECT6_gun", Postest[0], Postest[1], 1);
        // 安装误差导致的硬补偿
        solve_angle.pitch += pitch_compensation;
        solve_angle.yaw += yaw_compensation;
        return solve_angle;
    }
    /**
    * @brief  发布最终云台控制角度与开火指令的“一站式”接口
    * @param  hasAngle          是否成功计算出目标角度（true=有，false=无）
    * @param  auto_fire         本帧是否允许自动开火（默认 false）
    * @param  euler_angles_temp 目标相对角度（弧度），默认 (0,0)
    */
    /////////////////////
    // 周期性发送debug
    // int cntflag = 0;
    // int flag = 0;
    ///////////////////
    void Pub_Aangle(bool hasAngle, bool auto_fire = false, const cv::Point2d & euler_angles_temp = cv::Point2d(0, 0)){
        ///////////////////////////////////////////
        // 周期性发送debug
        // cntflag++;
        // if (cntflag == 2000) {
        //     if (flag == 0) {
        //         flag = 1;
        //     }
        //     else {
        //         flag = 0;
        //     }
        //     cntflag = 0;
        // }
        /////////////////////////////////////////
        // debug: 云台跟随效果rqt_plot打印
        // // yaw角
        // debugdate.data = m_cur_pose.yaw;           // 当前云台yaw角度
        // debugdate1.data = euler_angles_temp.y;     // 计算出云台需要转动的yaw角度
        // // pitch角
        // debugdate.data = m_cur_pose.pitch;          // 当前云台pitch角度
        // debugdate1.data = euler_angles_temp.x;   // 计算出云台需要转动的pitch角度
        geometry_msgs::Vector3 Angle;
        if(hasAngle && euler_angles_temp.x != 0.0 && euler_angles_temp.y != 0.0){
            Angle.x = (euler_angles_temp.x) + m_pitchOffset;
            Angle.y = euler_angles_temp.y - m_yawOffset;
            Angle.z = 1;
            // debugdate.data = m_cur_pose.yaw;    // 当前云台yaw角度
            // debugdate1.data = Angle.y;          // 计算出云台需要转动的yaw角度
            // 自动打弹
            if(auto_fire || (all_fire && TrackingID != 6 && TrackingID != -1)){
                Angle.z = 3; // 设置设计指令码
            }
            // debug
            ///////////////////////////////////////
            // Angle.z = 3;    // 测试电控打弹 (一直发射子弹)
            // Angle.z = 1;    // 测试电控打弹
            // Angle.z = 0;    // 测试电控打弹
            //////////////////////////////////////
            // 周期性发送debug
            // if (flag == 1) {
            //     Angle.z = 3;
            // }
            // else {
            //     Angle.z = 1;
            // }
            ////////////////////////////////
            AngPub.publish(Angle);
        }else{
            // 如果没有角度信息,返回陀螺仪角度
            Angle.x = RmSerialData.Pitch - m_pitchOffset;
            Angle.y = RmSerialData.Yaw - m_yawOffset;
            Angle.z = 0;
            // cout << "RmSerialData.Pitch:" << -RmSerialData.Pitch << endl;
            // cout << "RmSerialData.Yaw:" << RmSerialData.Yaw << endl;
            // debug
            ///////////////////////////////////
            // Angle.z = 3;    // 测试电控打弹 (一直发射子弹)
            // Angle.z = 1;    // 测试电控打弹
            // Angle.z = 0;    // 测试电控打弹
            //////////////////////////////////
            // 周期性发送debug
            // if (flag == 1) {
            //     Angle.z = 3;
            // }
            // else {
            //     Angle.z = 1;
            // }
            ///////////////////////////////////
            AngPub.publish(Angle);
        }
        // debug: 自动开火
        // debugdate.data = Angle.z;
    }
    // 参数配置函数
    void setParam(const std::string &file_path) {
        try {
            // 1. 文件存在性检查
            if (file_path.empty()) {
                ROS_ERROR("Empty config file path provided");
                return;
            }
            //2. 检查文件是否打开
            std::ifstream file_check(file_path);
            if (!file_check.is_open()) {
                ROS_ERROR("Failed to open config file: %s. Reason: %s",
                        file_path.c_str(), strerror(errno));
                return;
            }
            ROS_INFO("Loading parameters from: %s", file_path.c_str());
            // 3. 加载 YAML 文件并添加详细错误处理
            YAML::Node config;
            try {
                config = YAML::LoadFile(file_path);
            } catch (const YAML::BadFile& e) {
                ROS_ERROR("Bad YAML file: %s", e.what());
                return;
            } catch (const YAML::ParserException& e) {
                ROS_ERROR("YAML parsing error at line %d: %s", e.mark.line, e.what());
                return;
            }
            //加载参数
            if (config["Tracker"]){
                YAML::Node track = config["Tracker"];
                COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);
                eTime = track["eTime"].as<double>(0.002);
                local_gravity_ = track["local_gravity_"].as<double>(9.80665);
                trackTime = track["trackTime"].as<double>(0.7);
                pitch_compensation = track["pitch_compensation"].as<double>(0);
                yaw_compensation = track["yaw_compensation"].as<double>(0);
                m_score_tolerance = track["score_tolerance"].as<double>(1);
                m_switch_threshold = track["switch_threshold"].as<double>(15);                                    // 添加yawl文件
                m_aim_angle_tolerance = track["aim_angle_tolerance"].as<double>(40);                              // 添加yaml文件
                m_aim_pose_tolerance = track["aim_pose_tolerance"].as<double>(0.05);                              // 添加yaml文件
                m_aim_center_angle_tolerance = track["aim_center_angle_tolerance"].as<double>(1);                 // 添加yaml文件
                m_switch_trackmode_threshold = track["switch_trackmode_threshold"].as<double>(1.2);               // 添加yaml文件
                m_aim_center_palstance_threshold = track["aim_center_palstance_threshold"].as<double>(6.5);       // 添加yaml文件
                m_force_aim_palstance_threshold = track["force_aim_palstance_threshold"].as<double>(0.8);         // 添加yaml文件
                m_track_center = track["track_center"].as<double>(0);
                m_fix_on = track["fix_on"].as<double>(1);
                // ROS_INFO("Tracker parameters loaded:");
                // ROS_INFO("COMMAND_TIMESPAN:%.3f",COMMAND_TIMESPAN);
                // ROS_INFO("eTime:%.3f",eTime);
                // ROS_INFO("local_gravity_:%.3f",local_gravity_);
                // ROS_INFO("trackTime:%.3f",trackTime);
                // ROS_INFO("pitch_compensation:%.3f",pitch_compensation);
                // ROS_INFO("yaw_compensation:%.3f",yaw_compensation);
                // ROS_INFO("track_center: %d", m_track_center);
            }
            else {
                ROS_WARN("No 'Tracker' section in config file");
            }
            // 加载相机参数
            if (config["Camera"]){
                YAML::Node camera = config["Camera"];
                // 加载相机内参矩阵
                if (camera["camera_matrix"]) {
                    std::vector<double> cam_matrix_array = camera["camera_matrix"].as<std::vector<double>>();
                    if (cam_matrix_array.size() == 9) {
                        // 创建3*3相机内参矩阵
                        camera_matrix_ = (cv::Mat_<double>(3, 3) <<
                        cam_matrix_array[0], cam_matrix_array[1], cam_matrix_array[2],
                        cam_matrix_array[3], cam_matrix_array[4], cam_matrix_array[5],
                        cam_matrix_array[6], cam_matrix_array[7], cam_matrix_array[8]);
                    }
                }
                else {
                    ROS_WARN("No 'camera_matrix' section in config file");
                }
                // 加载畸变系数矩阵
                if (camera["dist_coeffs"]) {
                    std::vector<double> dist_coeffs_array = camera["dist_coeffs"].as<std::vector<double>>();
                    if (dist_coeffs_array.size() == 5) {
                        dist_coeffs_ = (cv::Mat_<double>(1, 5) <<
                            dist_coeffs_array[0], dist_coeffs_array[1], dist_coeffs_array[2],
                            dist_coeffs_array[3], dist_coeffs_array[4]);
                    }
                }
                else {
                    ROS_WARN("No 'dist_coeffs' section in config file");
                }
                // 加载补偿误差函数载串口补偿
                if (config["InstallOffset"])
                {
                    YAML::Node mod = config["InstallOffset"];
                    m_rollOffset = mod["rollOffset"].as<double>(0);
                    m_pitchOffset = mod["pitchOffset"].as<double>(0);
                    m_yawOffset = mod["yawOffset"].as<double>(0);
                    // ROS_INFO("InstallOffset parameters loaded:");
                    // ROS_INFO("m_rollOffset:%.3f",m_rollOffset);
                    // ROS_INFO("m_pitchOffset:%.3f",m_pitchOffset);
                    // ROS_INFO("m_yawOffset:%.3f",m_yawOffset);
                    // ROS_INFO("InstallOffset parameters loaded successfully");
                }
                else
                {
                    ROS_WARN("No 'CoordinateTransformer' section in config file");
                }
            }
        }
        catch (const YAML::Exception& e) {
            ROS_ERROR("YAML parsing error: %s", e.what());
        }
        catch (const std::exception& e) {
            ROS_ERROR("Error loading parameters: %s", e.what());
        }
        catch (...) {
            ROS_ERROR("Unknown error occurred while loading config file");
        }
    }
    /**
    * @brief 把世界坐标系下的 3D 点投影到当前图像平面
    * @param objectPoints 世界坐标系中的一组 3D 点
    * @param frame 用于可视化的 OpenCV 图像
    * @param can_show 是否弹出窗口并打印调试信息
    * @return 投影到图像上的 2D 像素坐标
    */
   // 重投影这里传入了相机的内参和旋转矩阵,如果重投影的结果有问题(rviz也是这样)(很有可能是识别那里世界坐标系变换到相机坐标系有问题),这很大可能是因为相机内参和畸变系数不准导致的,这时候需要重新标定相机
    std::vector<cv::Point2d> projectPointsToImage(const std::vector<cv::Point3d>& objectPoints, cv::Mat& frame, bool can_show) {
        cv::Mat rvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//旋转向量
        cv::Mat tvec = (cv::Mat_<float>(3, 1) << 0.0f, 0.0f, 0.0f);//平移向量
        std::vector<cv::Point2d> imagePoints;
        cv::projectPoints(objectPoints, rvec, tvec, camera_matrix_, dist_coeffs_, imagePoints);
        if(can_show){
            for (size_t i = 0; i < imagePoints.size(); ++i) {
                cv::circle(frame, imagePoints[i], 7, cv::Scalar(0, 255, 0), -1);
                //std::cout << "3D Point " << i << ": " << objectPoints[i] << std::endl;
                //std::cout << "Projected 2D Point " << i << ": " << imagePoints[i] << std::endl;
            }
            cv::imshow("Draw Points", frame);
            cv::waitKey(1);
        }
        return imagePoints;
    }
    /**
    * @brief 把“世界坐标系下的三维标准点”重映射到图像二维平面
    * @param pegPos 标准点在“map”坐标系下的三维坐标 (x,y,z)
    * @param cal TF/坐标变换工具类实例
    * @param projectPointsToImage 3D→2D 投影函数
    * @param frame_ 克隆图形用于可视化
    * @return 该点在图像上的像素坐标 (u,v)；若 TF 异常则返回 (0,0)
    */
    cv::Point2d project_pegPoints(Eigen::Vector3d& pegPos){
        Eigen::Vector3d resultPos;
        resultPos << pegPos.x(), pegPos.y(), pegPos.z();
        Eigen::Vector3d resultPose;
        resultPose << 0, 0, 0;
        cal->TFUpdata("map","PEG", resultPos, resultPose, 1);
        auto absPos_temp = cal->GetPos("cam","PEG",1);
        if((absPos_temp[0].x() == 0 && absPos_temp[0].y() == 0 && absPos_temp[0].z() == 0) || isnan(absPos_temp[1].z())){
            cout << "重投影坐标系未更新！" << endl;
            return cv::Point2d(0.0, 0.0);
        }
        cv::Point3d objectPoint( absPos_temp[0].x(), absPos_temp[0].y(), absPos_temp[0].z());
        std::vector<cv::Point3d> objectPoints = {objectPoint};
        std::vector<cv::Point2d> imagePoints = projectPointsToImage(objectPoints, frame_, false);
        return imagePoints[0];
    }
private:
    /**
     * @brief 装甲板处理总入口：决策 → 丢失检测 → 预测 → 返回平滑 3D 状态
     * @param armors 当前帧所有检测到的装甲板数组
     * @return TargetModel* 目标模型指针（正常状态）或nullptr（异常状态）
     
/
    TargetModel
 armorUpdate(rm_msgs::ArmorArrayConstPtr armors)
    {
        // 1.时间戳管理
        // 获取当前时间戳(毫秒)
        Now_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        // 2.计算目标丢失时间(毫秒)
        double Lost_Time = (Now_Time_armor - Track_Time_armor)/1000.0;
        // 目标丢失状态处理(上一帧的目标状态)
        if(!Switch_Armor){
            // 跟踪中状态:打印跟踪当前ID
            // std::cout << "追踪中TrackingID: " << static_cast<int>(TrackingID) << std::endl;
        }else{
            // 目标丢失状态: 重置跟踪ID和旋转方向
            // std::cout << "目标丢失!重置TrackingID!" << std::endl;
            TrackingID = -1;
        }
        // 3.装甲板决策
        rm_msgs::Armor* detect = nullptr;
        if(armors->armors.size()){
            // 决策函数: 从所有装甲板中选择最佳跟踪目标(本帧决策)
            detect = decision(armors, TrackingID, Switch_Armor);
            if(detect != nullptr){ // 本帧状态更新
                if(Switch_Armor){
                    // 目标丢失后重新锁定: 更新跟踪ID
                    TrackingID = detect->number;
                    Switch_Armor = false;
                    // 更新追踪时间戳
                    Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//更新追踪时间戳
                }else if(detect->number == TrackingID){
                    // 正常跟踪: 更新时间戳
                    Track_Time_armor = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();//更新追踪时间戳
                }else{
                    // 检测到非跟踪目标: 忽略
                    detect = nullptr;
                }
            }
        }
        // 4. 目标丢失判定
        if(Lost_Time > trackTime){
            // 普通目标: 超过跟踪时间预测判定丢失
            if(TrackingID != 6){
                Switch_Armor = true;
            // 前哨站:3倍跟踪时间阈值与判定丢失
            }else if(Lost_Time > trackTime
3){
                Switch_Armor = true;
            }
        }
        // 运行预测流程
        TargetModel
 targetModel_temp = nullptr;
        targetModel_temp = runPrediction(Switch_Armor, detect);
        return targetModel_temp;
    }
    /**
    * @brief  装甲板决策函数 —— 从所有候选装甲板中选出“应被瞄准”的那一块
    * @param  armors          当前帧所有装甲板数组（ROS 消息）
    * @param  TrackingID_temp 正在跟踪的目标 id（字符形式）
    * @param  Switch_Armor_temp 是否允许切换目标（true = 允许重新选择）
    * @return rm_msgs::Armor*   选中的装甲板指针；nullptr 表示无有效目标
    * @note   决策流程：先按距离/ID 筛，再按旋转方向挑同号板
    
/
    rm_msgs::Armor
 decision(rm_msgs::ArmorArrayConstPtr armors,
                             const char& TrackingID_temp,
                             const bool& Switch_Armor_temp){
        rm_msgs::Armor* detect_temp = nullptr;
        double minDisToCenter = std::numeric_limits<double>::max();
        int cunt = 0;
        for(auto& armor:armors->armors){
            if(Eigen::Vector3d({armor.armorPose.position.x,armor.armorPose.position.y,armor.armorPose.position.z}).norm() >= 6.0 || armor.number == 9){
                if(armor.number != 6){
                    continue;
                }else if(Eigen::Vector3d({armor.armorPose.position.x,armor.armorPose.position.y,armor.armorPose.position.z}).norm() >= 10.0 || armor.number == 9){
                    continue;
                }
            }
            if(armor.number == TrackingID_temp || (Switch_Armor_temp && armor.armorToCenterDis < minDisToCenter)){
                if (!detect_temp || armor.armorToCenterDis < minDisToCenter) {
                    detect_temp = const_cast<rm_msgs::Armor*>(&armor);
                    minDisToCenter = armor.armorToCenterDis;
                }
            }
            if(armor.number == TrackingID_temp){
                cunt++;
            }
        }
        if(detect_temp == nullptr){
            //std::cout << "决策结果为空！" << std::endl;
            return nullptr;
        }
        return detect_temp;
    }
    /**
     * @brief 初始化目标模型类型
     * @param detect 当前帧检测到的装甲板信息指针
     * @return bool true-模型已初始化完成，false-模型未初始化完成
     
/
    bool initModel(rm_msgs::Armor
 detect)
    {
        //detect->type删掉,观测不准,detect->type基地直接瞄，不过卡尔曼
        if (detect != nullptr && targetModel->m_type_init_cnt < 3)
        {
            // 大装甲板1号->英雄
            // 小装甲板2~7号->普通步兵
            if ((detect->number >= 1 && detect->number <= 5) || detect->number == 7)
            {
                // 类型变化时重置计数器
                if (targetModel->m_model_type != KinematicModel::STANDARD)
                    targetModel->m_type_init_cnt = 0;
                // 设置为标准模型
                targetModel->m_model_type = KinematicModel::STANDARD;
                targetModel->m_type_init_cnt++;
            }
            else if  (detect->number == 6)
            {
                if (targetModel->m_model_type != KinematicModel::OUTPOST)
                    targetModel->m_type_init_cnt = 0;
                targetModel->m_model_type = KinematicModel::OUTPOST;
                targetModel->m_type_init_cnt++;
            }
            // 连续3帧确认后初始化模型
            if (targetModel->m_type_init_cnt >= 3)
            {
                setModelType(targetModel->m_model_type);
                return true; // 已经初始化
            }
            else
            {
                // 不允许击打
                targetModel->auto_fire = false;
                return false; // 没有初始化
            }
        }
        if (targetModel->m_type_init_cnt < 3) {
            return false;  // 没有检测到目标,没有初始化
        }
        else {
            return true;   // 已经初始化,大于3帧，处于跟踪状态,短暂丢失
        }
    }
    /**
     * @brief 重置预测模块和目标模型状态
     * @param lost_state 目标丢失状态（true-目标丢失，false-目标存在）
     * @param abnormal 预测状态异常标志（true-状态异常，false-状态正常）
     * @return bool 返回当前的目标丢失状态
     * @note 重置条件：目标丢失(lost_state)或预测状态异常(abnormal)
     */
    bool reset(bool lost_state, bool abnormal)
    {
         // 检查是否需要重置(目标丢失或状态异常)
        if(lost_state || abnormal){
            // 1. 创建新的目标模型实例(完全重置)
            targetModel = std::make_unique<TargetModel>(config_path_);
            return lost_state; // 丢失状态
        }
        else
        {
             return lost_state; // 跟踪状态
        }
    }
    /**
     * @brief 执行预测流程的核心函数
     
/
    TargetModel
 runPrediction(bool Switch_Armor_temp, rm_msgs::Armor* detect){
       // 检查重置
        if (reset(Switch_Armor_temp, targetModel->abnormal)){
            return nullptr;
        }
        // 初始化
        bool isInitialized = initModel(detect);
        if (isInitialized){
            // 更新目标模型(执行核心预测流程)
            updateTargetModel(detect);
        }
        else{
            return nullptr;
        }
        // 返回预测结果(异常状态返回nullptr,正常状态返回目标模型指针)
        if (targetModel->abnormal) // 标志位异常返回空,正常的话检查是否初始化
        {
            return nullptr;
        }
        else
        {
            return targetModel.get();
        }
    }
    /**
     * @brief 更新目标模型状态
     * @param detect 当前帧检测到的装甲板信息指针
     
/
    void updateTargetModel(rm_msgs::Armor
 detect) {
        // 初始化装甲板属性(设置ID,类型)
        setArmorProps(detect);
        // 核心预测处理:使用扩展卡尔曼滤波器进行状态预测和更新
        updateEKFState(detect);
    }
    /**
    * @brief 设置装甲板基本属性
    * @param detect 当前检测到的装甲板消息指针（nullptr 则跳过）
    
/
    void setArmorProps(rm_msgs::Armor
 detect){
        if(detect != nullptr){
            // 设置装甲板ID
            targetModel->armor.id = detect->number;
            // 设置装甲板类型
            targetModel->armor.armor_type = detect->type;
            // 根据装甲板类型设置数量
            targetModel->count = (detect->number == 6) ? 3 : 4;
       }
    }
    /**
     * @brief 相机坐标系点重投影可视化
     * @param pre_armors 预测的装甲板位置信息
     * @param armors_position_pose 观测到的装甲板位置和姿态信息
     */
    void projectCameraPointsToImage(Armors pre_armors,std::vector<std::vector<Eigen::Vector3d>> armors_position_pose) {
        std::vector<cv::Point3d> objectPoints;
        // 将观测的装甲板放进预测的装甲板数组来投影观测的装甲板
        if (armors_position_pose.size()) {
            Armor temp;
            temp.position.x() = armors_position_pose[0][0].x();
            temp.position.y() = armors_position_pose[0][0].y();
            temp.position.z() = armors_position_pose[0][0].z();
            pre_armors.push_back(temp);
        }
        // 5个点的重投影(观测点和重构出来的四块装甲板)
        for (int i = 0; i < pre_armors.size(); i++) {
            coorConverter->tfupdate_imu({m_cur_pose.roll,m_cur_pose.pitch,m_cur_pose.yaw});
            Eigen::Vector3d temp = coorConverter->map2Cam(pre_armors[i].position);
            cv::Point3d point_temp_1 = Point3d(temp.x(),temp.y(),temp.z());
            objectPoints.push_back(point_temp_1);
        }            
        // 重投影装甲板中心点
        std::vector<cv::Point2d> imagePoints = projectPointsToImage(objectPoints, frame_, 0);
        for (size_t i = 0; i < imagePoints.size(); i++) {
            cv::circle(frame_,imagePoints[i],7,cv::Scalar(0,255,0),-1);
        }
        int count = targetModel->m_status->number;
        // 为每个装甲板绘制四个角点和对角线
        for (int i = 0; i < count; i++) {
            Eigen::Vector3d rpy = pre_armors[i].rpy;           // 装甲板姿态
            Eigen::Vector3d center = pre_armors[i].position;   // 装甲板中心位置
            double yaw = pre_armors[i].rpy.z() - m_cur_pose.yaw;
            std::vector<Eigen::Vector3d> cornerpoint = coorConverter->armor2Corner(center,targetModel->armor.id,0.2618, yaw);
            // 将角点从世界系 -> 相机系
            for (int i = 0; i < cornerpoint.size(); i++) {
                cornerpoint[i] = coorConverter->map2Cam(cornerpoint[i]);
            }
            // 调用前进行类型转换
            std::vector<cv::Point3d> cvPoints;
            cvPoints.reserve(cornerpoint.size());
            for (const auto& point : cornerpoint) {
                cvPoints.emplace_back(point.x(), point.y(), point.z());
            }
            // 投影每块装甲板的四个角点
            std::vector<cv::Point2d> cornerPoints2D = projectPointsToImage(cvPoints, frame_, 0);
            // 绘制四个角点
            for (const auto& point : cornerPoints2D) {
                cv::circle(frame_,point , 5, cv::Scalar(0, 0, 255), -1); // 红色角点
            }
            // 绘制装甲板边框(连接四个角点)
            for (int j = 0; j < 4; j++) {
                int next_j = (j + 1) % 4;
                cv::line(frame_, cornerPoints2D[j], cornerPoints2D[next_j],
                cv::Scalar(255, 255, 0), 2); // 青色边框
            }
            // 绘制对角线
            cv::line(frame_, cornerPoints2D[0], cornerPoints2D[2],
                cv::Scalar(0, 255, 255), 2); // 黄色对角线1（左上到右下）
            cv::line(frame_, cornerPoints2D[1], cornerPoints2D[3],
                cv::Scalar(0, 255, 255), 2); // 黄色对角线2（右上到左下）
        }
        // 最后显示图像
        cv::imshow("Draw Points", frame_);
        cv::waitKey(1);
    }
    /**
     * @brief 二维地图投影可视化
     * @param pre_armors 预测的装甲板位置信息
     * @param armors_position_pose 观测到的装甲板位置和姿态信息
     
/
    void projectMapPointsToImage (Armors pre_armors,std::vector<std::vector<Eigen::Vector3d>> armors_position_pose) {
        // // debug
        // cout << "预测的第一块装甲板的x坐标: " << pre_armors[0].position.x() << endl;
        // cout << "预测的第一块装甲板的y坐标: " << pre_armors[0].position.y() << endl;
        const double PIXELS_PER_METER = 100; // 比例尺 -> 100像素对应实际观测1米
        // 创建白色背景图像
        Mat whiteImage(800, 800, CV_8UC3, Scalar(255, 255, 255));
        // 绘制基准线
        for (int i = 1; i <= 8; i++) {
            if (i == 6) {
                cv::circle(whiteImage, cv::Point(400, 400), i * PIXELS_PER_METER, cv::Scalar(0, 0, 255), 1);
                continue;
            }
            cv::circle(whiteImage, cv::Point(400, 400), i * PIXELS_PER_METER, cv::Scalar(0, 0, 0), 1);
        }
        // 绘制估计装甲板位置
        for (int i = 0; i < pre_armors.size(); i++) {
            // 绘制实心圆点，半径5像素，绿色
            cv::circle(whiteImage, cv::Point(-pre_armors[i].position.y()
*PIXELS_PER_METER + 400, -pre_armors[i].position.x()
PIXELS_PER_METER + 400), 3, cv::Scalar(0, 255, 0), -1);
            // break; // 测试(只打印第一块预测装甲板)
        }
        // 绘制观测装甲板位置
        for (int i = 0; i < armors_position_pose.size(); i++) {
            // 绘制观测装甲板
            double x = armors_position_pose[i][0].x();
            double y = armors_position_pose[i][0].y();
            cv::circle(whiteImage, cv::Point(-y
PIXELS_PER_METER + 400, -x*
PIXELS_PER_METER + 400), 3, cv::Scalar(0, 0, 0), -1);
            double r[2] = {0.25,0.25};
            // double yaw = armors_position_pose[0][1].z();
            double yaw = armors_position_pose[i][1].z();
            double center_x = armors_position_pose[i][0].x() - r[0] * cos(yaw);
            double center_y = armors_position_pose[i][0].y() - r[0] * sin(yaw);
            cv::circle(whiteImage, cv::Point(-center_y
PIXELS_PER_METER + 400, -center_x
PIXELS_PER_METER + 400), 3, cv::Scalar(255, 0, 0), -1);
            // whiteImage.at<Vec3b>(armor_x
20 + 400, armor_y
20 + 400) = Vec3b(0, 255, 0);
            // 绘制实心圆点，半径5像素，黑色
            // // 绘制重构出来的装甲板 蓝色
            // // if (i > 1) {
            // //     continue;
            // // }
            for (int j = 0; j < 4; j++) {
                double armor_x = center_x + r[j%2] * cos(yaw + 0.5 * j * PI);
                double armor_y = center_y + r[j%2] * sin(yaw + 0.5 * j * PI);
                cv::circle(whiteImage, cv::Point(-armor_y
PIXELS_PER_METER + 400, -armor_x*PIXELS_PER_METER + 400), 3, cv::Scalar(255, 0, 0), -1);
            }
        }
        int x = static_cast<int>(-pre_armors[(targetModel->m_status->index)].position.y()*PIXELS_PER_METER + 400);
        int y = static_cast<int>(-pre_armors[(targetModel->m_status->index)].position.x()*PIXELS_PER_METER + 400);
        // cout << "targetModel->m_status->index: " << targetModel->m_status->index << endl;
        // 选择距离中心点角度最小的点(这里和实际算法不一样,只是单纯编写一个算法框架)
        // 根据上一帧选择的重构装甲板index(编号)绘制击打线
        // 绘制击打线(击打预测)
        cv::line(whiteImage, Point(400,400), cv::Point(x,y), cv::Scalar(255, 0, 0), 1, cv::LINE_AA);
        // 圈出打击目标
        cv::circle(whiteImage, cv::Point(x,y), 5, cv::Scalar(0, 0, 255), 1);
        // 绘制自身车辆位置(地图中心)
        cv::circle(whiteImage, cv::Point(400, 400), 5, cv::Scalar(0, 0, 0), -1);
        imshow("White Image with a Black Pixel", whiteImage);
        waitKey(1); // 等待按键
    }
    /**
     * @brief 使用扩展卡尔曼滤波器更新目标状态
     * @param detect 当前帧检测到的装甲板信息（rm_msgs::Armor指针）
     *              - 非nullptr：表示有装甲板检测，执行完整流程
     *              - nullptr：表示无装甲板检测，执行纯预测
     
/
    void updateEKFState(rm_msgs::Armor
 detect)
    {
        /*
        命名规则:
        DETECT{id}(基础装甲板)
        DETECT{id}_0(另一块装甲板)
        */
        // 主要通过依赖TF的异常处理来跳过不存在的装甲板
        int armor_id = targetModel->armor.id;                                   // 获取当前装甲板的ID
        std::string base_armor_frame = "DETECT" + std::to_string(armor_id);
        ros::Time unified_time;
        bool base_transform_found = false;
        std::vector<std::vector<Eigen::Vector3d>> all_Armor_Position_Pose;      // 存储所有装甲板的位置和姿态
        std::vector<Eigen::Vector3d> base_position_pose;
        base_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第一个向量置零
        base_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第二个向量置零
        // 声明变量
        geometry_msgs::TransformStamped base_transform;
        geometry_msgs::TransformStamped other_transform;
        // 在拿装甲板前延时(使得获取装甲板的坐标尽可能精确)
        // // 帧率控制
        end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
        double frame_time = (end_Time - begin_Time)/1000.0;
        if ((end_Time - begin_Time) - 25.0 < 0.0)// 帧率控制
        {
            int sleep_time_ms = static_cast<int>(25.0 - (end_Time - begin_Time));
            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time_ms));
            //frame_time = 0.03;
            end_Time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
            frame_time = (end_Time - begin_Time)/1000.0;
            // cout << "sleep_time_ms:" << sleep_time_ms << "ms" << endl;
            // cout << "frame_time:" << frame_time
1000 << "ms" << endl;
        }
        begin_Time = end_Time;
        // debugdate1.data = frame_time
1000; 
        // 查询第一块装甲板
        try{
            // 等待缓冲区更新
            if (tfBuffer_.canTransform("map", base_armor_frame, ros::Time(0), ros::Duration(1))) {
                base_transform = tfBuffer_.lookupTransform("map", base_armor_frame, ros::Time(0));
            }
            targetModel->update_count++; 
            // // debug: 计算tf上传时间差
            ////////////////////////////////////////////////////////////
            // tool_begin_Time_ros = base_transform.header.stamp;
            // ros::Duration test = tool_begin_Time_ros - tool_end_Time_ros;
            // debugdate.data = test.toSec() * 1000;
            // cout << "duration_time" << tool_begin_Time_ros - tool_end_Time_ros << "ms" << endl;
            // tool_end_Time_ros = tool_begin_Time_ros;
            // cout << "打印第一块装甲板///////////////////////" << endl;
            // cout << "打印位置x:" << base_transform.transform.translation.x << endl;
            // cout << "打印位置y:" << base_transform.transform.translation.y << endl;
            // cout << "打印位置z:" << base_transform.transform.translation.z << endl;
            //ROS_DEBUG("[ArmorQuery] 成功获取基础装甲板 %s 的变换，统一时间戳: %f", base_armor_frame.c_str(), unified_time.toSec());
            //////////////////////////////////////////////////////////////
            base_transform_found = true;
            unified_time = base_transform.header.stamp;
            // 初始化上一块装甲板的时间
            if (targetModel->update_count == 1) {
                last_tf_time = unified_time;
            }
            if (unified_time <= last_tf_time && targetModel->update_count != 1) {
                // ROS_WARN("时间戳相等装甲板未更新");
                base_transform_found = false;
            }
            last_tf_time = unified_time;
            // 提取装甲板的位置
            Eigen::Vector3d base_position(
                base_transform.transform.translation.x,
                base_transform.transform.translation.y,
                base_transform.transform.translation.z
            );
            // 提取装甲板的姿态
            tf2::Quaternion qtn(base_transform.transform.rotation.x,
                        base_transform.transform.rotation.y,
                        base_transform.transform.rotation.z,
                        base_transform.transform.rotation.w);
            tf2::Matrix3x3 matrix(qtn);
            double roll, pitch, yaw;
            matrix.getRPY(roll, pitch, yaw);
            Eigen::Vector3d base_pose;
            base_pose.x() = roll;
            base_pose.y() = pitch;
            base_pose.z() = yaw;
            base_pose.z() = targetModel->last_yaw + angles::shortest_angular_distance(targetModel->last_yaw, base_pose.z());//获得两帧之间的最小角度，更新当前yaw(避免π跳到-π)
            targetModel->last_yaw = base_pose.z();
            // rqt打印观测值
            // debugdate.data = base_position[0];           // rqt打印第一块装甲板观测x
            // debugdate.data = base_position[1];           // rqt打印第一块装甲板观测y
            // debugdate.data = base_position[2];           // rqt打印第一块装甲板观测z
            // debugdate.data = base_pose[2] / 3.14 * 180;  //rqt打印第一块装甲板观测yaw
            // 打印观测值
            // cout << "第一块装甲板观测值: " << endl;
            // cout << "机器人中心的x坐标: " << base_position[0] << endl;
            // cout << "机器人中心的y坐标: " << base_position[1] << endl;
            // cout << "装甲板的高度: " << base_position[2]<< endl;
            // cout << "机器人整体偏航角yaw: " << base_pose[2] << endl;
            // // 计算协方差 (用于给ekf初值R)
            // double R = calculate_R(base_position[0]);      // 计算x的协方差    -> pose
            // double R = calculate_R(base_position[1]);   // 计算y的协方差    -> pose
            // double R = calculate_R(base_position[2]);   // 计算z的协方差    -> pose
            // double R = calculate_R(base_pose[2]);       // 计算yaw的协方差  -> yaw
            // debugdate.data = R;
            // cout << "协方差: " << R << endl; 
            // 将位置和姿态存储到数据结构中
            base_position_pose[0] = base_position;  // 位置
            base_position_pose[1] = base_pose;      // 姿态
            /*
            如果串口没有发数据,那么坐标系就不会更新就会进这段代码提前返回,已经debug测试过,没有发送串口数据后会进入这段代码
            //坐标系异常检测
            */
            if((base_position_pose[0].x() == 0 && base_position_pose[0].y() == 0 && base_position_pose[0].z() == 0)
                || isinf(base_position_pose[0].x()) || isinf(base_position_pose[0].y()) || isinf(base_position_pose[0].z())
                || isnan(base_position_pose[0].x()) || isnan(base_position_pose[0].y()) || isnan(base_position_pose[0].z())
                || isnan(base_position_pose[1].z()) || isinf(base_position_pose[1].z())){
                cout << "惯性系坐标系未更新！" << endl;
                targetModel->abnormal = true;
                return;
            }
            // 存储第一块装甲板的位置和姿态
            all_Armor_Position_Pose.push_back(base_position_pose);
        }
        catch (tf2::TransformException &ex) {
            ROS_ERROR("[ArmorQuery] 无法获取基础装甲板 %s 的变换: %s", base_armor_frame.c_str(), ex.what());
            // 设置目标状态为异常并返回
            targetModel->abnormal = true;
            ROS_WARN("[ArmorQuery] 由于基础装甲板查询失败，跳过本帧处理");
            return;
        }
        // 查询第二块装甲板
        if (base_transform_found) {
            std::string other_armor_frame = "DETECT" + std::to_string(armor_id) + "
" + std::to_string(0);
            try {
                // 等待缓冲区更新
                other_transform = tfBuffer
.lookupTransform("map", other_armor_frame, unified_time, ros::Duration(0));
                // 提取装甲板位置
                Eigen::Vector3d other_position(
                    other_transform.transform.translation.x,
                    other_transform.transform.translation.y,
                    other_transform.transform.translation.z
                );
                // 提取装甲板姿态
                tf2::Quaternion qtn(other_transform.transform.rotation.x,
                            other_transform.transform.rotation.y,
                            other_transform.transform.rotation.z,
                            other_transform.transform.rotation.w);
                tf2::Matrix3x3 matrix(qtn);
                double roll, pitch, yaw;
                matrix.getRPY(roll, pitch, yaw);
                Eigen::Vector3d other_pose;
                other_pose.x() = roll;
                other_pose.y() = pitch;
                other_pose.z() = yaw;
                other_pose.z() = targetModel->last_yaw + angles::shortest_angular_distance(targetModel->last_yaw, other_pose.z());//获得两帧之间的最小角度，更新当前yaw(避免π跳到-π)
                targetModel->last_yaw = other_pose.z();
                //debugdate1.data = other_position[0];  // rqt打印第二块装甲板观测x
                //debugdate1.data = other_position[1];  // rqt打印第二块装甲板观测y
                //debugdate1.data = other_position[2];  // rqt打印第二块装甲板观测z
                //debugdate1.data = other_pose[2];      // rqt打印第二块装甲板观测yaw
                // cout << "第二块装甲板观测值: " << endl;
                // cout << "机器人中心的x坐标: " << other_position[0] << endl;
                // cout << "机器人中心的y坐标: " << other_position[1] << endl;
                // cout << "装甲板的高度: " << other_position[2]<< endl;
                // cout << "机器人整体偏航角yaw: " << other_pose[2] << endl;
                // 将位置和姿态存储到数据结构中
                std::vector<Eigen::Vector3d> other_position_pose;
                other_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第一个向量置零
                other_position_pose.emplace_back(Eigen::Vector3d::Zero()); // 第二个向量置零
                other_position_pose[0] = other_position;  // 位置
                other_position_pose[1] = other_pose;      // 姿态
                //坐标系异常检测
                if((other_position_pose[0].x() == 0 && other_position_pose[0].y() == 0 && other_position_pose[0].z() == 0)
                    || isinf(other_position_pose[0].x()) || isinf(other_position_pose[0].y()) || isinf(other_position_pose[0].z())
                    || isnan(other_position_pose[0].x()) || isnan(other_position_pose[0].y()) || isnan(other_position_pose[0].z())
                    || isnan(other_position_pose[1].z()) || isinf(other_position_pose[1].z())){//坐标系异常检测
                    cout << "惯性系坐标系未更新！" << endl;
                    targetModel->abnormal = true;
                    return;
                }
                // 存储第二块装甲板的位置和姿态
                all_Armor_Position_Pose.push_back(other_position_pose);
            }
            catch(tf2::TransformException &ex) {
                // ROS_INFO("未查询到第二块装甲板yyyyyyyyyyyyy");
            }
        }
        visual_armor_position_pose_temp = all_Armor_Position_Pose; // 用于可视化
        // 有装甲板->上传惯性系+预测+更新+更新目标对象
        if (base_transform_found)
        {
            if (all_Armor_Position_Pose.empty()) {
                ROS_WARN("[ArmorQuery] 未找到任何有效的装甲板数据");
                targetModel->abnormal = true;
                return;
            } else {
                ROS_DEBUG("[ArmorQuery] 成功获取 %zu 块装甲板的数据", all_Armor_Position_Pose.size());
                targetModel->abnormal = false;
            }
            // 创建装甲板实例
            Armors armors(all_Armor_Position_Pose.size());
            for (int i = 0; i < all_Armor_Position_Pose.size(); i++)
            {
                armors[i].position = Eigen::Vector3d(all_Armor_Position_Pose[i][0].x(),all_Armor_Position_Pose[i][0].y(),all_Armor_Position_Pose[i][0].z());
                armors[i].rpy = Eigen::Vector3d(all_Armor_Position_Pose[i][1].x(),all_Armor_Position_Pose[i][1].y(),all_Armor_Position_Pose[i][1].z());
            }
            // 先验状态预测
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = targetModel->ekf->predict(armors);
            // cout << "输出状态矩阵:" << endl;
            // cout << "///////////////////////////////" << endl;
            // cout << prior.first << endl;
            // cout << "///////////////////////////////" << endl;
            // 创建运动模型实例
            std::shared_ptr<KinematicModel> prior_status;
            if (targetModel->m_model_type == KinematicModel::STANDARD)
            {
                prior_status = std::make_shared<StandardModel>(prior.first);
            }
            else if (targetModel->m_model_type == KinematicModel::BALANCE)
            {
                prior_status = std::make_shared<BalanceModel>(prior.first);
            }
            else if (targetModel->m_model_type == KinematicModel::OUTPOST)
            {
                prior_status = std::make_shared<OutpostModel>(prior.first);
            }
            // 数据关联匹配(计算检测到的装甲板与预测的装甲板之间的匹配度,在得到评分矩阵后为每个检测到的装甲板找到一个最合适的预测装甲板对应,形成一个最优匹配)
            Eigen::MatrixXd score = getScoreMat(armors,prior_status->getArmors(0));
            // cout << "输出分数:" << endl;
            // cout << score << endl;
            std::map<int, int> armor_match = getMatch(score, m_score_tolerance,targetModel->count);
            /*
            正常的数据关联匹配结果：
            1.一一对应关系:
                每个检测装甲板索引对应唯一的目标装甲板索引
                没有重复的匹配关系
            2. 索引范围合理
            3.得分低于阈值
                所有匹配对的得分都低于m_score_tolerance
                得分越低表示匹配度越高
            1.绿色点(预测)和黑色点(预测)是否在空间上接近
            2.匹配的装甲板在图像上是否对齐
            */
            // debug
            // for (auto it : armor_match)
            // {
            //     cout << "装甲板序列: " << it.first  << "目标装甲板索引: " << it.second << endl;
            //     debugdate.data = it.first;
            //     debugdate1.data = it.second;
            // } 
            int current_index = targetModel->m_status->index;
            targetModel->m_status = targetModel->ekf->update(armors, prior.first, prior.second,armor_match);
            targetModel->m_status->index = current_index;
            // debug (用于调试ekf)
            Eigen::VectorXd current_state = targetModel->m_status->getState();
            // prior_status->print("预测的数据");
            // debugdate1.data = current_state(0);                                    // rqt预测x车中心
            // debugdate1.data = targetModel->m_status->getArmors(0)[0].position.x(); // rqt打印预测第一块装甲板x坐标
            // debugdate1.data = current_state(2);                                    // rqt预测y车中心
            // debugdate.data = current_state[4];                                     // rqt预测z左侧
            // debugdate1.data = current_state[5];                                    // rqt预测z右侧
            // debugdate1.data = current_state(8) / 3.14 * 180;                       // rqt预测yaw      // debugdate.data = current_state(6);                                     // rqt打印半径左侧
            // debugdate1.data = current_state(7);                                    // rqt打印半径右侧
            // 可视化
            projectCameraPointsToImage(targetModel->m_status->getArmors(0),all_Armor_Position_Pose);
        }
        // 无检测目标仅预测+更新目标对象
        else
        {
            // 用于debug
            ROS_WARN("无装甲板仅预测");
            // 执行卡尔曼滤波器(传入空装甲板列表)
            Armors empty_armors;
            std::pair<Eigen::MatrixXd, Eigen::MatrixXd> prior = targetModel->ekf->predict(empty_armors);
            std::map<int, int> armor_match;
            armor_match.clear();
            // 后验状态更新
            targetModel->m_status = targetModel->ekf->update(empty_armors, prior.first, prior.second,armor_match);
            // 可视化
            Armors no_predict_armos = targetModel->m_status->getArmors(0);
            std::vector<std::vector<Eigen::Vector3d>> empty_Armor_Position_Pose;
            projectCameraPointsToImage(no_predict_armos,empty_Armor_Position_Pose);
            return;
        }
    }
    /***
     * 工具函数
     * 用于实时计算R(协方差)
     *
    */
    // 用于计算R值
    double max_num = INT_MIN;
    double min_num = INT_MAX;
    double calculate_R(double num) {
        if(Switch_Armor){
            min_num = INT_MAX;
            max_num = INT_MIN;
        }
        // 计算R值的函数
        if (num > max_num) {
            max_num = num;
        }
        if (num < min_num) {
            min_num = num;
        }
        double sqrt_R = (max_num - min_num) * 0.5;
        double R = sqrt_R * sqrt_R;
        cout << "协方差: " << R << endl;
        return R;
    }
    /***
     * 工具函数
     * 用于计算单帧之间的最大角度差值
    */
    // 用于计算R值
    double last_yaw_test = 0;
    double max_between_yaw = 0;
    int cnt = 0;
    double calculate_max_yaw(double yaw) {
        if(cnt == 0){
            max_between_yaw = 0;
            last_yaw_test = yaw;
        }
        double yaw_temp = abs(yaw - last_yaw_test);
        if (yaw_temp > max_between_yaw) {
            max_between_yaw = yaw_temp;
        }
        last_yaw_test = yaw;
        cnt++;
        cout << "两帧之间最大角度差: " << max_between_yaw << endl;
        return max_between_yaw;
    }
    /**
     * @brief 基于熵权法计算装甲板匹配得分矩阵
     * @param detect_armors 检测到的装甲板集合
     * @param standard_armors 预测的标准装甲板集合
     * @return Eigen::MatrixXd 匹配得分矩阵(行:检测装甲板, 列:预测装甲板)
     */
    Eigen::MatrixXd getScoreMat(const Armors &detect_armors, const Armors &standard_armors)
    {
        int m = detect_armors.size();
        int n = standard_armors.size();
        // 计算两组装甲板之间的坐标差和角度差两个负向指标
        Eigen::Matrix<double, Eigen::Dynamic, 2> negative_score;
        negative_score.resize(m * n, 2);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                Eigen::Vector3d d1(detect_armors[i].position.x(),detect_armors[i].position.y(),detect_armors[i].position.z());
                Eigen::Vector3d d2(standard_armors[j].position.x(),standard_armors[j].position.y(),standard_armors[j].position.z());
                negative_score(i * n + j, 0) = (d1 - d2).norm();
                negative_score(i * n + j, 1) = abs(_std_radian(detect_armors[i].rpy.z() - standard_armors[j].rpy.z()));
            }
        }
        // 数据标准化
        Eigen::Matrix<double, Eigen::Dynamic, 2> regular_score;
        regular_score.resize(m * n, 2);
        for (int i = 0; i < regular_score.rows(); i++)
        {
            regular_score(i, 0) = (negative_score.col(0).maxCoeff() - negative_score(i, 0)) / (negative_score.col(0).maxCoeff() - negative_score.col(0).minCoeff());
            regular_score(i, 1) = (negative_score.col(1).maxCoeff() - negative_score(i, 1)) / (negative_score.col(1).maxCoeff() - negative_score.col(1).minCoeff());
        }
        // 计算样本值占指标的比重
        Eigen::Matrix<double, Eigen::Dynamic, 2> score_weight;
        score_weight.resize(m * n, 2);
        Eigen::VectorXd col_sum = regular_score.colwise().sum();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            score_weight(i, 0) = regular_score(i, 0) / col_sum(0);
            score_weight(i, 1) = regular_score(i, 1) / col_sum(1);
        }
        // 计算每项指标的熵值
        Eigen::Vector2d entropy = Eigen::Vector2d::Zero();
        for (int i = 0; i < score_weight.rows(); i++)
        {
            if (score_weight(i, 0) != 0)
                entropy(0) -= score_weight(i, 0) * log(score_weight(i, 0));
            if (score_weight(i, 1) != 0)
                entropy(1) -= score_weight(i, 1) * log(score_weight(i, 1));
        }
        entropy /= log(score_weight.rows());
        // 计算权重
        Eigen::Vector2d weight = (Eigen::Vector2d::Ones() - entropy) / (2 - entropy.sum());
        // 计算匹配得分矩阵(综合评分)
        Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> score;
        score.resize(m, n);
        for (int i = 0; i < m; i++)
        {
            for (int j = 0; j < n; j++)
            {
                if (i < detect_armors.size() && j < standard_armors.size())
                {
                    score(i, j) = negative_score.row(i * standard_armors.size() + j) * weight;
                }
            }
        }
        return score;
    }
    /**
    * @brief 执行装甲板匹配的核心函数
    * @param matrix 得分矩阵（rows:检测到的装甲板, cols:目标装甲板）
    * @param score_max 允许的最大匹配得分阈值
    * @param m 目标装甲板数量
    * @return std::map<int, int> 匹配结果（key:检测装甲板索引, value:目标装甲板索引）
    */
    std::map<int, int> getMatch(Eigen::MatrixXd matrix, double score_max, int m)
    {
        //初始化最终结果
        this->row_col.clear();
        for (int i = 0; i < matrix.rows(); i++) {
            this->row_col[i] = -1;
        }
        //初始化min
        for (int i = 0; i < matrix.rows() && i < m; i++)
        {
            this->min += matrix(i, i);//初始化最小总得分
            this->row_col[i] = i;
        }
        //初始化行数列
        this->row.clear();
        for (int i = 0; i < matrix.rows(); i++)
        {
            // 有多少行，行数列就有多少个数
            this->row.emplace_back(i);
        }
        /// 初始化列数列，n块装甲板，因此列为n
        this->col.clear();
        for (int i = 0; i < m; i++)
        {
            this->col.emplace_back(i);
        }
        this->tmp_v.clear();
        this->result.clear();
        this->nAfour.clear();
        this->fourAfour.clear();
        //计算C(n,m)
        if (matrix.rows() <= m)
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, matrix.rows());
        else
            this->getCombinationsNumbers(this->row, this->tmp_v, this->result, 0, m);
        //使用上一步的结果计算A(m,m),最终得到A(n,m)
        for (auto it = this->result.begin(); it != this->result.end(); it++)
        {
            do{
                this->nAfour.emplace_back(*it);
            } while (next_permutation(it->begin(), it->end()));
        }
        //计算A(m,m)
        do{
            fourAfour.push_back(col);
        } while (next_permutation(col.begin(), col.end())); // stl自带全排列函数
        
        //使用A(n,m)和A(m,m)求出可能选择的所有行列的组合的结果
        for (auto it = this->nAfour.begin(); it != this->nAfour.end(); it++)
        {
            for (auto it2 = this->fourAfour.begin(); it2 != this->fourAfour.end(); it2++){
                this->tmp = 0;
                for (int i = 0; i < it->size(); i++)
                    this->tmp += matrix((*it)[i], (*it2)[i]);
                if (this->tmp < this->min){
                    this->min = this->tmp;
                    this->row_col.clear();
                    for (int i = 0; i < it->size(); i++){
                        this->row_col[it->at(i)] = it2->at(i);
                    }
                }
            }
        }
        //删除-1的键值对
        for (auto it = this->row_col.begin(); it != this->row_col.end();){
            if (it->second == -1 || (it->second != -1 && matrix(it->first, it->second) > score_max)){
                it = this->row_col.erase(it);
            }
            else
                ++it;
        }
        return this->row_col;
    }
    // 辅助函数：递归生成组合
    void getCombinationsNumbers(const std::vector<int>& input, std::vector<int>& tmp_v, std::vector<std::vector<int>>& result, int start, int k)
    {
        for (int i = start; i < input.size(); ++i) {
            tmp_v.emplace_back(input[i]);
            if (tmp_v.size() == k) {
                result.emplace_back(tmp_v);
                tmp_v.pop_back();
                continue;
            }
            else {
                getCombinationsNumbers(input, tmp_v, result, i + 1, k);
            }
            tmp_v.pop_back();
        }
    }
    /**
     * @brief 根据目标类型切换运动模型和观测器，并调整跟踪参数
     * @param type 目标运动模型类型（STANDARD/BALANCE/OUTPOST）
     */
    void setModelType(KinematicModel::Type type)
    {
        // 设置目标模型类型
        targetModel->m_model_type = type;
        /
*2.运动模型与观测器初始化*
/
        if (type == KinematicModel::STANDARD)
        {
            targetModel->m_status = std::make_shared<StandardModel>();   //创建标准模型
            targetModel->ekf = std::make_shared<StandardObserver>(config_path_);
        }
        else if (type == KinematicModel::BALANCE)
        {
            targetModel->m_status = std::make_shared<BalanceModel>();    //创建平衡步兵模型
            targetModel->ekf = std::make_shared<BalanceObserver>(config_path_);
        }
        else if (type == KinematicModel::OUTPOST)
        {
            targetModel->m_status = std::make_shared<OutpostModel>();    //创建哨兵模型
            targetModel->ekf = std::make_shared<OutpostObserver>(config_path_);
        }
    }
};
先大致看一下这段代码
这段代码是一个自瞄系统中的跟踪器（Tracker）类。它使用ROS框架，从相机获取图像和装甲板检测信息，通过卡尔曼滤波等算法对目标进行跟踪和预测，并计算出云台需要转动的角度，最终通过ROS话题发布控制指令。
由于代码量很大，我将主要对代码的结构和功能进行分析，并指出可能存在的问题和改进建议。
代码结构：
类Tracker：模板类，模板参数是跟踪时间（TimeSpan）。
私有成员：包括ROS相关的发布订阅、图像处理相关变量、坐标变换工具、跟踪控制参数、时间戳管理、标志位等。
公有成员：构造函数、析构函数、回调函数（设置串口信息、图像、装甲板）、主跟踪循环、发布角度等。
主要功能：
通过ROS订阅串口信息、图像和装甲板检测结果。
对装甲板进行决策（选择跟踪目标）、预测（使用扩展卡尔曼滤波）、补偿（重力补偿、安装误差补偿）。
根据预测结果计算云台需要转动的角度，并发布控制指令。
可能存在的问题和改进建议：
a. 代码可读性：
- 代码中使用了大量的注释，但有些注释是中文，有些是英文，建议统一。
- 有些代码块较长，可以进一步拆分为更小的函数，提高可读性。
b. 性能：
- 在updateEKFState函数中，有一个帧率控制的延时（sleep_for）。这可能会影响实时性。建议考虑使用其他方式控制帧率，比如固定频率的循环。
- 在查询TF变换时，使用了固定的延时等待（ros::Duration(1)），这可能会在TF变换不可用时导致程序阻塞。建议使用更短的超时时间，并处理异常。
c. 异常处理：
- 在setParam函数中，加载配置文件时如果出现异常，会记录错误并返回，但程序可能继续运行，这可能导致后续使用未初始化的参数。建议在加载失败时采取更严格的措施，比如抛出异常或终止程序。
- 在TF查询时，如果查询失败，会设置targetModel->abnormal = true，但后续的处理中，有些地方会直接返回，有些地方会继续执行。需要确保异常状态被正确处理。
d. 资源管理：
- 在构造函数中，使用new分配了Calculater和CoordinateTransformer，并在析构函数中删除。建议使用智能指针（如std::unique_ptr）来管理这些资源，避免内存泄漏。
- 同样，targetModel和m_MPC使用了智能指针，但cal和coorConverter没有。
e. 线程安全：
- 该类在ROS回调函数中设置成员变量（如RmSerialData、m_armors等），并在Track函数中读取。这些操作可能发生在不同的线程中（ROS默认情况下为每个订阅回调创建单独的线程）。因此，需要确保线程安全。可以考虑使用互斥锁（如std::mutex）保护这些共享数据。
f. 代码冗余：
- 在reconstruction_choose_compensation函数中，有两次计算m_target_pose的代码块（分别对应锁中心和不锁中心），它们非常相似，可以考虑提取为单独的函数。
g. 魔法数字：
- 代码中存在一些魔法数字，比如0.025、0.11等。建议将这些数字定义为常量，并加上注释说明。
h. 配置参数：
- 配置文件通过YAML加载，但代码中有些参数没有在配置文件中，比如BulletVector的初始值。建议将更多的参数放到配置文件中。
i. 可视化：
- 代码中有多处可视化（如projectCameraPointsToImage、projectMapPointsToImage），这些可视化在调试时很有用，但在实际部署时可能会影响性能。建议通过条件编译或运行时标志来控制是否启用可视化。
j. 跟踪逻辑：
- 在armorUpdate函数中，丢失时间的计算是基于Now_Time_armor和Track_Time_armor的差值。但是，Track_Time_armor只在检测到目标并更新时才会更新。如果长时间没有检测到目标，丢失时间会很大，从而触发重置。但是，对于前哨站（ID=6）的丢失时间阈值是普通目标的3倍。这个逻辑是合理的，但需要注意，如果目标短暂丢失又出现，可能会被当作新目标重新初始化，这可能导致跟踪不稳定。
k. 数据关联：
- 数据关联部分使用了熵权法计算匹配得分，并使用组合优化寻找最佳匹配。这部分代码比较复杂，且计算量可能较大。如果装甲板数量较多，可能会影响性能。可以考虑使用更简单的匹配方法，比如最近邻匹配，或者使用匈牙利算法。
l. 扩展卡尔曼滤波：
- 扩展卡尔曼滤波的实现封装在targetModel->ekf中。需要确保其实现正确，并且噪声参数设置合理。
m. 重力补偿：
- 在getAngle函数中，重力补偿的计算假设了子弹速度恒定，且只考虑了垂直方向的重力。实际上，子弹速度可能会衰减，且空气阻力等因素也会影响弹道。如果需要更精确的补偿，可以考虑使用更复杂的弹道模型。
n. 安装误差补偿：
- 在getAngle函数中，直接加上了pitch_compensation和yaw_compensation。这些补偿量需要通过标定得到。
o. 前哨站模型：
- 前哨站（ID=6）的模型与其他模型不同，具有3块装甲板。在代码中，对于前哨站的处理有一些特殊之处，比如丢失时间阈值更大。需要确保模型和观测器与前哨站的运动特性匹配。
总结：
这段代码实现了一个完整的自瞄跟踪系统，涉及图像处理、目标跟踪、控制等多个方面。代码结构较为清晰，但仍有改进空间，特别是在线程安全、性能、异常处理等方面。建议进行进一步的测试和优化。
由于代码量很大，无法逐行分析，以上是对整体结构和一些关键点的分析。如果需要更详细的修改，请指出具体部分。
这是一个完整的RoboMaster自瞄系统中的跟踪器（Tracker）类实现，让我分析一下它的主要结构和功能：
一、总体架构
这是一个基于ROS的自瞄系统跟踪器，主要功能包括：
目标跟踪与预测：使用扩展卡尔曼滤波（EKF）预测目标运动
云台控制：计算并发布云台转动角度
弹道补偿：考虑重力、延迟等因素进行弹道补偿
数据关联：匹配观测到的装甲板与预测的装甲板
二、核心组件
## 1. 状态管理
支持多种目标模型：
STANDARD：标准模型（英雄、步兵）
OUTPOST：前哨站模型
BALANCE：平衡步兵模型（代码中提及但未完全实现）
## 2. 主要功能模块
决策模块 (decision)：选择最佳跟踪目标
预测模块 (updateEKFState)：EKF预测与更新
补偿模块 (getAngle)：弹道重力补偿
控制模块 (reconstruction_choose_compensation)：云台控制决策
## 3. 可视化工具
projectCameraPointsToImage：相机系投影
projectMapPointsToImage：地图系投影
draw_simulated_bullets：弹道模拟
三、核心算法流程
text
接收数据（串口+图像+装甲板）
装甲板决策（选择目标）
EKF预测
选板与补偿
发布控制指令
四、关键技术点
## 1. 数据关联
使用熵权法计算匹配得分矩阵，解决观测装甲板与预测装甲板的对应关系。
## 2. 时间同步
TF变换获取精确时间戳
时间戳比对避免重复处理同一帧
电控延迟补偿
## 3. 弹道补偿
考虑：
重力加速度
子弹飞行时间
电控延迟（COMMAND_TIMESPAN）
曝光时间（eTime）
## 4. 控制模式
普通模式：直接角度计算
MPC模式：模型预测控制（可选）
锁中心模式：跟随旋转中心
五、可优化点
## 1. 代码结构
cpp
// 建议将大类拆分为多个小类：- ArmorTracker：装甲板跟踪
- BallisticSolver：弹道解算
- Controller：云台控制
- Visualizer：可视化
## 2. 性能优化
cpp
// 当前问题：1. TF查询阻塞（ros::Duration(1)）过长
## 2. 组合数计算可能耗时（getCombinationsNumbers）
## 3. 频繁的内存分配/释放

// 建议：1. 使用固定大小容器避免动态分配
## 2. 预计算组合数
## 3. 异步TF查询
## 3. 线程安全
cpp
// 当前：回调函数和Track()可能在多线程环境// 建议：添加互斥锁保护共享数据
std::mutex data_mutex_;
## 4. 配置管理
cpp
// 当前：YAML配置+硬编码混合// 建议：统一配置接口class ConfigManager {static TrackerConfig load(const std::string& path);};
六、使用建议
## 1. 参数调优重点
m_score_tolerance：数据关联阈值
m_switch_threshold：装甲板切换阈值
COMMAND_TIMESPAN：电控延迟（关键！）
安装误差补偿（m_pitchOffset, m_yawOffset）
## 2. 调试技巧
使用rqt_plot查看角度跟随
可视化投影检查坐标系转换
弹道模拟验证补偿效果
## 3. 常见问题排查
TF异常：检查坐标系发布频率
预测发散：调整EKF的Q/R矩阵
延迟过大：优化时间戳管理
这是一个相当完整的自瞄系统实现，涵盖了从感知到控制的完整链条。代码结构清晰，但复杂度较高，建议在实际使用中逐步调试各个模块。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        double k = 1;   // 空气阻力系数
        // 计算水平位移
        double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time) -> void {
        const long long fire_interval = 500;  // 发射间隔：200毫秒
        if (current_time - this->last_fire_time >= fire_interval) {
            if (bullets.size() < 20) {        // 最多显示10颗子弹
                bullets.push_back(IdProj {
                    next_id++,
                    // ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
namespace tools{
    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点
    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };
    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };
    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };
    /*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    
/
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char
 str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };
    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };
    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置
    };
    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };
    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };
    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {}
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t) const -> HitCircle;
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t) const -> HitPos;
        // 获取开火时间
        auto get_fire_t() const -> double;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };
    // 子弹位置信息
    struct IdPos {
        int id;
        Eigen::Vector3d pos;
    };
    // 子弹投影圆信息
    struct IdCircle {
        int id;
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };
    // 子弹模拟器封装
    struct IdProj {
        int id;
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time) -> std::vector<IdCircle>;
        auto update_bullet(long long current_time) -> void;
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;
        long long next_id = 0;
        long long last_fire_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
    };
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time);
}
#endif // TRAJECTORY_VISUALIZER_HPP
可以帮我看一下可能是那里导致弹道复现不对吗?
我们首先需要分析弹道复现不对的可能原因。弹道复现主要依赖于两个部分：子弹弹道的物理模拟和坐标变换。
在ProjectileSimulator类中，get_pos_by_t函数根据时间计算子弹在枪管坐标系中的位置，然后通过坐标变换转换到相机坐标系，再转换到图像坐标系。
可能的问题点：
物理模型：子弹弹道的物理模型是否正确？目前使用的模型是考虑空气阻力的，但是空气阻力系数k=1是否合理？另外，公式是否正确？
坐标变换：从枪管坐标系到相机坐标系的变换是否正确？在get_pos_by_t函数中，我们首先将目标点从相机坐标系转换到枪管坐标系，然后以此方向为基准，计算子弹在枪管坐标系中的偏移（水平位移w和高度h），再转换回相机坐标系。这里需要注意的是，我们假设弹道始终在由枪管指向目标点的垂直平面内，这个假设是合理的。
时间基准：开火时间fire_t是毫秒时间戳，而get_pos_by_t函数的参数t也是毫秒时间戳吗？从代码看，get_pos_by_t函数中计算时间差时，直接使用t - this->fire_t，这里单位应该是毫秒，但公式中时间单位应该是秒。所以需要检查时间单位是否统一。
参数传递：在AimCorrector的update_bullet函数中，创建ProjectileSimulator时传入的fire_t是current_time，而实际上子弹发射时间应该考虑延迟（如曝光时间、电控延迟等）。在之前的代码中，我们看到自瞄代码中计算击中时间时加了m_time_off（包括COMMAND_TIMESPAN + eTime + 0.025）。但是在这个弹道复现中，我们可能只是为了可视化，所以直接用了当前时间。不过，如果是为了复现实际弹道，那么应该考虑这些延迟。
子弹速度：在ShootParam中，v0是子弹初速度，这个值是否准确？是否与自瞄代码中使用的子弹速度一致？
仰角：在ShootParam中，aim_angle是发射仰角，这个角度应该是云台pitch角加上补偿角。在自瞄代码中，我们计算出的m_target_pose.pitch是云台需要转动的角度（已经包含重力补偿），那么这里传入的aim_angle应该是这个值吗？注意，在自瞄代码中，我们发布的角度是经过补偿的，但是云台实际转动的角度可能因为机械误差而有所不同。
目标点：在ShootParam中，target_xyz_i_camera是相机坐标系下的目标点。在自瞄代码中，我们传入的是coorConverter->map2Cam(abs_target_armor.position)，这个转换是否正确？需要检查map2Cam函数。
图像投影：在get_circle_by_t函数中，我们计算子弹在图像上的投影圆。这里先通过map2Cam将世界坐标转换到相机坐标，然后通过cam2Map将相机坐标转换回世界坐标？这里似乎有问题。实际上，我们已经有子弹在相机坐标系下的坐标（由get_pos_by_t返回的bullet_xyz_i_camera），那么投影到图像上应该使用相机内参和畸变系数。但是在这段代码中，我们却使用了coorConverter->map2pixel，这个函数可能是将世界坐标（地图坐标）投影到像素坐标。这可能是错误的。
仔细看get_circle_by_t函数：
cpp
Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
注意，这里的bullet.pos是get_pos_by_t返回的HitPos中的pos，这个pos是相机坐标系下的坐标吗？看get_pos_by_t函数，它返回的pos是bullet_xyz_i_camera，即相机坐标系下的坐标。那么，map2Cam函数是将世界坐标转换到相机坐标，但这里传入的是相机坐标，所以可能不对。
再看coorConverter->map2Cam的实现，它应该是将世界坐标系（map）下的点转换到相机坐标系（cam）。所以，这里我们需要的是世界坐标系下的点，而不是相机坐标系下的点。
所以，问题可能出现在这里：在get_pos_by_t函数中，我们计算的是子弹在相机坐标系下的位置，但是get_circle_by_t函数却把它当作世界坐标系下的点，用map2Cam转换，这显然不对。
应该改为：在get_pos_by_t函数中，我们计算子弹在相机坐标系下的位置，然后直接使用相机内参和畸变系数投影到图像上。但是，在ProjectileSimulator类中，我们没有相机内参和畸变系数。所以，我们需要在ProjectileSimulator中保存相机内参和畸变系数，或者通过coorConverter提供相机投影功能。
但是，看CoordinateTransformer类，它可能已经封装了相机投影。我们查看coorConverter->map2pixel函数，它可能是将世界坐标（map）投影到像素坐标。所以，如果我们有子弹在世界坐标系下的位置，那么就可以用map2pixel得到像素坐标。
那么，我们需要的是子弹在世界坐标系下的位置，而不是相机坐标系下的位置。所以，在get_pos_by_t函数中，我们应该计算子弹在世界坐标系下的位置。
因此，我们需要重新考虑弹道模拟的坐标系。
实际上，弹道模拟应该在世界坐标系（惯性系）中进行。我们已知：
发射时刻云台的位置和姿态（即相机坐标系到世界坐标系的变换）
子弹在相机坐标系（或枪管坐标系）中的运动轨迹，然后通过云台的外参转换到世界坐标系。
所以，步骤应为：
在发射时刻，云台（相机）的外参已知（即世界坐标系到相机坐标系的变换矩阵）。
子弹在相机坐标系（或枪管坐标系）中的运动轨迹，通过外参转换到世界坐标系。
然后，将世界坐标系下的子弹位置投影到图像上。
但是，在ProjectileSimulator中，我们只有coorConverter，它提供了坐标系之间的变换。我们来看get_pos_by_t函数：
它首先将目标点从相机坐标系转换到枪管坐标系（target_xyz_i_barrel），然后以枪管坐标系下的目标方向为基准，计算子弹在枪管坐标系中的位置（bullet_xyz_i_barrel），然后转换回相机坐标系（bullet_xyz_i_camera）。
但是，我们最终需要的是世界坐标系下的位置。所以，我们需要将bullet_xyz_i_camera转换到世界坐标系。这可以通过coorConverter->cam2Map来实现。
所以，在get_pos_by_t函数中，我们应该返回的是世界坐标系下的位置。我们可以这样修改：
cpp
const Eigen::Vector3d bullet_xyz_i_map = this->coorConverter->cam2Map(bullet_xyz_i_camera);return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(), bullet_xyz_i_map };
然后，在get_circle_by_t函数中，我们就可以使用map2Cam将世界坐标转换到相机坐标（实际上，这一步在投影中可能不需要，因为map2pixel可能已经包含了这个过程）。但是，我们注意到在get_circle_by_t函数中，我们使用map2Cam将世界坐标转换到相机坐标，然后为了计算半径，又使用cam2Map转换回世界坐标，这似乎绕路了。实际上，我们可以直接使用世界坐标计算像素坐标。
我们来看coorConverter->map2pixel函数，它可能是将世界坐标直接投影到像素坐标。如果是这样，我们可以：
cpp
cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
但是，计算半径时，我们需要在图像上找到一个边缘点。原来代码中，我们是在相机坐标系中，沿着与光心连线垂直的方向偏移一个物理半径（0.0085米）得到边缘点，然后投影到图像上。这个边缘点也需要在世界坐标系中表示，然后通过map2pixel得到像素坐标。
所以，我们需要将边缘点也从相机坐标系转换到世界坐标系。原来的代码是：
cpp
Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
这里，xyz_c是相机坐标系下的点，edge_xyz_c也是相机坐标系下的点，然后通过cam2Map转换到世界坐标系。这样是可以的。
但是，现在bullet.pos已经是世界坐标系下的点，那么我们可以：
cpp
Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);   // 世界坐标->相机坐标
Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
这样，我们得到了图像上子弹中心点和边缘点的像素坐标，然后计算距离作为半径。
所以，总结一下，我们需要修改get_pos_by_t函数，使其返回世界坐标系下的位置。然后，在get_circle_by_t函数中，使用这个世界坐标进行投影。
另外，我们还需要检查时间单位。在物理公式中，时间单位应该是秒，所以需要将毫秒转换为秒。
我们修改get_pos_by_t函数：
cpp
auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {double k = 1;   // 空气阻力系数double delta_t = (t - this->fire_t) / 1000.0;   // 将毫秒转换为秒// 计算水平位移double w = delta_t * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);// 计算高度double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;// 弹道轨迹仅取决于目标点(理想弹道)const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 计算基准方向const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();const Eigen::Vector3d h_norm = { 0., 0., 1. };const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);// 将子弹位置转换到世界坐标系const Eigen::Vector3d bullet_xyz_i_map = this->coorConverter->cam2Map(bullet_xyz_i_camera);const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),bullet_xyz_i_barrel(1, 0) };const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),target_xyz_i_barrel(1, 0) };return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(), bullet_xyz_i_map};}
同时，在get_circle_by_t函数中，我们使用修改后的bullet.pos（现在是世界坐标）进行投影。
另外，我们还需要检查coorConverter->map2pixel函数是否正确，它应该将世界坐标投影到像素坐标。
如果以上步骤都正确，那么弹道复现应该正确。
但是，如果还是不正确，我们可以通过以下步骤调试：
检查目标点在世界坐标系中的位置是否合理。
检查子弹在世界坐标系中的位置是否合理。
检查投影到图像上的位置是否合理。
我们可以在get_circle_by_t函数中打印一些中间变量，比如bullet.pos（世界坐标），以及投影后的像素坐标。
此外，还需要注意：在AimCorrector的update_bullet函数中，我们创建ProjectileSimulator时传入的fire_t是current_time，而实际上，子弹发射时间可能不是当前帧的时间，而是未来某个时间（考虑延迟）。在自瞄代码中，我们计算击中时间时加了延迟，那么这里为了模拟实际弹道，也应该加上相同的延迟。所以，我们需要将自瞄代码中的延迟时间传递给AimCorrector。但是，在draw_simulated_bullets函数中，我们直接使用了当前时间，这可能是不准确的。
因此，我们需要修改draw_simulated_bullets函数，使其能够考虑延迟。我们可以将延迟时间作为参数传入，或者在ShootParam中加入延迟时间。
由于问题可能较多，我们首先修改坐标系的问题，确保物理模型和坐标变换正确，然后再调整时间延迟。
由于代码较长，我们只修改了关键部分。如果问题仍然存在，可能需要进一步调试。
由于我们无法直接运行代码，所以只能提供这些修改建议。请根据实际情况调整。
如果还有问题，请提供更多的错误信息，例如弹道投影在图像上的位置与预期偏差多大，等等。
看了你的弹道可视化代码，我发现了几个可能导致弹道复现不正确的问题：
主要问题分析
## 1. 弹道模型公式错误
在 get_pos_by_t 函数中，弹道模型公式看起来有问题：
cpp
double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;
这个公式看起来很复杂且可能不正确。考虑空气阻力的弹道方程通常使用数值积分求解，而不是解析解。
## 2. 空气阻力系数 k=1 不合理
cpp
double k = 1;   // 空气阻力系数
空气阻力系数通常远小于1（如 0.01-0.1 量级），k=1 意味着空气阻力非常大。
## 3. 坐标变换链不完整
在计算子弹位置时，坐标变换路径是：
text
相机系 -> 枪管系 -> 计算弹道 -> 枪管系 -> 相机系
但缺少从世界坐标系到相机坐标系的完整变换链。
## 4. 时间单位混淆
cpp
const long long fire_interval = 500;  // 发射间隔：200毫秒
注释说是200ms，但实际值是500，存在混淆。
## 5. 子弹半径参数不准确
cpp
Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
17mm子弹的半径应该是0.0085m（8.5mm）吗？应该是直径17mm，半径8.5mm。
建议修正方案
## 1. 简化弹道模型（不考虑空气阻力）
cpp
auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {double delta_t = (t - this->fire_t) / 1000.0;  // 转换为秒// 简化的弹道方程（无空气阻力）double vx = this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);double vy0 = this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle);double x = vx * delta_t;  // 水平位移double y = vy0 * delta_t - 0.5 * this->g * delta_t * delta_t;  // 垂直位移// 从枪管系到相机系的变换// 需要确保shoot_param.target_xyz_i_camera是相机坐标系下的目标点const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 基准方向（目标在枪管系中的水平方向）
    Eigen::Vector3d target_dir_xy = target_xyz_i_barrel;
    target_dir_xy.z() = 0;  // 只取水平分量if (target_dir_xy.norm() < 1e-6) {
        target_dir_xy = Eigen::Vector3d(1, 0, 0);  // 默认方向} else {
        target_dir_xy.normalize();}// 子弹在枪管系中的位置
    Eigen::Vector3d bullet_xyz_i_barrel = target_dir_xy * x + Eigen::Vector3d(0, 0, 1) * y;// 转换回相机系
    Eigen::Vector3d bullet_xyz_i_camera = this->coorConverter->gun2Cam(bullet_xyz_i_barrel);// 判断是否击中：水平距离是否超过目标距离double bullet_horizontal_dist = bullet_xyz_i_barrel.head<2>().norm();double target_horizontal_dist = target_xyz_i_barrel.head<2>().norm();bool hit = (bullet_horizontal_dist >= target_horizontal_dist);return HitPos{hit, bullet_xyz_i_camera};}
## 2. 修正坐标变换逻辑
确保你传入的 shoot_param.target_xyz_i_camera 是正确的：
在自瞄代码中应该是：
cpp
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(目标世界坐标);
## 3. 添加调试输出
cpp
auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {// ... 计算过程 ...// 调试输出
    std::cout << "弹道计算 - 时间: " << (t - fire_t)/1000.0 
              << "s, 水平位移: " << x << "m, 垂直位移: " << y << "m, 目标水平距离: " << target_horizontal_dist << "m, 击中: " << hit << std::endl;return HitPos{hit, bullet_xyz_i_camera};}
## 4. 修正时间管理
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 固定发射间隔：200毫秒if (current_time - this->last_fire_time >= fire_interval) {if (bullets.size() < 10) {  // 限制显示数量// 注意：这里应该使用实际的发射时间，考虑电控延迟long long actual_fire_time = current_time;  // 可以加上延迟
            bullets.push_back(IdProj{
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, actual_fire_time)});this->last_fire_time = current_time;}}}
## 5. 检查坐标变换器
确保 CoordinateTransformer 正确实现了以下方法：
map2Cam：世界坐标系 → 相机坐标系
cam2Gun：相机坐标系 → 枪管坐标系
gun2Cam：枪管坐标系 → 相机坐标系
cam2Map：相机坐标系 → 世界坐标系
map2pixel：世界坐标系 → 像素坐标系
快速诊断步骤
打印关键坐标值：
cpp
// 在draw_simulated_bullets中添加
std::cout << "目标点相机坐标: " << shoot_param.target_xyz_i_camera.transpose() << std::endl;
std::cout << "子弹初速: " << shoot_param.v0 << "m/s" << std::endl;
std::cout << "发射仰角: " << shoot_param.aim_angle << "rad" 
          << " (" << shoot_param.aim_angle * 180/M_PI << "度)" << std::endl;
可视化检查：
弹道点应该在图像中形成一条平滑的抛物线
第一个点应该在枪口位置附近
最后一个点应该在目标装甲板位置
验证坐标变换：
cpp
// 测试坐标变换
Eigen::Vector3d test_world(3.0, 0.0, 0.0);  // 正前方3米
Eigen::Vector3d test_cam = coorConverter->map2Cam(test_world);
Eigen::Vector3d test_back = coorConverter->cam2Map(test_cam);
std::cout << "原始: " << test_world.transpose() 
          << " -> 相机: " << test_cam.transpose()<< " -> 返回: " << test_back.transpose() << std::endl;
最可能的问题是弹道模型公式错误和坐标变换链不完整。建议先用简化的无空气阻力模型，确保基础正确后再考虑空气阻力。
// 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
现在已经确定了没有进入这个for循环,为什么没有进入这个for循环?
我们注意到，在 update_bullet 函数中，我们向 bullets 添加了新的子弹模拟器，但是在 get_circles 函数中，我们使用 now_time 和子弹的发射时间进行比较，如果 now_time 小于发射时间，则跳过。
问题可能出现在 update_bullet 函数中，我们传入的 fire_t 是 current_time，而 current_time 就是 now_time（在draw函数中传入的当前时间）。
因此，当我们添加子弹时，子弹的发射时间被设置为当前时间，那么在同一个 now_time 下，now_time 可能等于 fire_t，所以不会跳过。
但是，注意在 get_circles 函数中，我们判断的是 now_time < it->proj.get_fire_t()，如果相等，则不会跳过，会计算弹道。
所以，如果子弹被添加，那么在同一帧中，now_time 等于 fire_t，不会跳过，应该会进入计算。
但是，如果 update_bullet 没有被调用，或者添加子弹的条件不满足，那么 bullets 可能为空，就不会进入循环。
因此，我们需要检查：
update_bullet 是否被正确调用，并且添加了子弹。
添加子弹的条件：current_time - this->last_fire_time >= fire_interval 是否满足，并且 bullets.size() < 20。
另外，注意 update_bullet 函数中，我们使用 last_fire_time 记录上一次发射时间，初始值为当前时间（构造函数中初始化为当前时间）。所以第一帧时，current_time - last_fire_time 可能为0，小于200，所以不会添加子弹。
为了测试，我们可以将 fire_interval 设置为0，确保每次调用 update_bullet 都会添加子弹。
此外，还需要检查 draw_simulated_bullets 函数中，我们创建了 AimCorrector 对象，然后调用了 update_bullet 和 get_circles。但是注意，AimCorrector 对象是局部变量，每次调用 draw_simulated_bullets 都会重新创建，所以之前添加的子弹会被清空。
这可能是问题所在：每次绘制都是重新开始，子弹列表被清空，然后只添加一颗子弹（如果满足条件的话），并且由于时间间隔，可能只添加了一颗。
为了解决这个问题，我们需要将 AimCorrector 对象作为全局变量或静态变量，或者作为类成员，以便持续跟踪子弹状态。
在Tracker类中，我们可以将 AimCorrector 作为成员变量，并在每次更新时传入当前的射击参数和当前时间。
但是，在当前的代码中，draw_simulated_bullets 函数是每次独立创建的，所以子弹列表无法保持。
因此，我建议将 AimCorrector 作为Tracker的成员变量，然后在Track函数中更新子弹模拟器，并在绘制函数中直接使用这个成员变量来获取子弹圆圈。
修改步骤：
在Tracker类中添加成员变量：AimCorrector aim_corrector_;，并在构造函数中初始化（需要CoordinateTransformer和初始射击参数）。
在Track函数中，当计算出射击参数时，更新aim_corrector_的射击参数，并调用update_bullet。
在绘制弹道时，使用aim_corrector_.get_circles获取当前子弹圆圈。
但是，注意：AimCorrector 的构造函数需要CoordinateTransformer和ShootParam，而ShootParam是每次计算都可能变化的。
所以，我们可以在Tracker的构造函数中初始化aim_corrector_，然后每次更新射击参数，并调用update_bullet。
然而，在Tracker类中，我们已经在reconstruction_choose_compensation函数中计算了射击参数，并调用了draw_simulated_bullets。我们可以将射击参数存储下来，然后更新到aim_corrector_中。
但是，由于弹道模拟是连续的，我们需要在每一帧都更新子弹状态（即使没有新子弹发射，也要更新已有子弹的位置）。
所以，我们可以这样调整：
在Tracker类中：
添加成员变量：ShootParam current_shoot_param_; 和 AimCorrector aim_corrector_;
在构造函数中，用初始参数初始化aim_corrector_（可能为空参数，等待更新）
在reconstruction_choose_compensation函数中，计算出的射击参数赋值给current_shoot_param_，然后调用aim_corrector_.update_bullet(当前时间)，并更新aim_corrector_的射击参数（因为AimCorrector类中需要更新射击参数以创建新的子弹模拟器）
在绘制函数中，使用aim_corrector_.get_circles(当前时间)获取子弹圆圈。
但是，注意AimCorrector类中的update_bullet函数会创建新的子弹，而我们需要在每一帧都更新所有子弹的位置（在get_circles中已经根据当前时间计算位置）。
另外，AimCorrector类中需要更新射击参数，因为每次发射的子弹参数可能不同（比如目标点变化，仰角变化）。所以，我们可能需要在每次创建新子弹时更新射击参数。
因此，我们修改AimCorrector类，添加一个更新射击参数的函数，并在每次创建新子弹前更新。
但是，在当前的AimCorrector实现中，子弹的创建只依赖于固定的射击参数（构造函数传入的），所以如果我们不更新，那么新子弹会使用旧的射击参数。
所以，我们需要在AimCorrector类中添加一个成员函数来更新射击参数，然后在Tracker中每次计算完新的射击参数后，更新到AimCorrector中。
修改AimCorrector类：
添加一个成员变量ShootParam shoot_param_，并在构造函数中初始化。
添加一个成员函数void update_shoot_param(const ShootParam& shoot_param)，用于更新射击参数。
在update_bullet函数中，创建新子弹时使用当前的shoot_param_。
然后，在Tracker类中，每次计算出新的射击参数，就调用aim_corrector_.update_shoot_param(current_shoot_param_)。
这样，每次发射的新子弹都会使用最新的射击参数。
但是，注意：在draw_simulated_bullets函数中，我们创建了一个新的AimCorrector对象，这是不对的。我们应该使用Tracker的成员变量aim_corrector_。
所以，我们需要将draw_simulated_bullets函数改为使用Tracker的成员变量aim_corrector_。
但是，draw_simulated_bullets函数是一个独立的函数，它需要访问Tracker的私有成员。我们可以将其改为Tracker的成员函数。
因此，我建议将draw_simulated_bullets函数移到Tracker类中，作为私有成员函数，并直接使用aim_corrector_。
总结修改步骤：
在Tracker类中添加私有成员：
ShootParam current_shoot_param_;
AimCorrector aim_corrector_;
在Tracker构造函数中初始化aim_corrector_，需要传入coorConverter和初始射击参数（可以默认为0）。
在reconstruction_choose_compensation函数中，计算出的射击参数赋值给current_shoot_param_，然后调用aim_corrector_.update_shoot_param(current_shoot_param_)，并调用aim_corrector_.update_bullet(当前时间)。
将draw_simulated_bullets函数改为Tracker的成员函数，并修改其实现，直接使用aim_corrector_获取子弹圆圈并绘制到图像上。
在Track函数中，调用draw_simulated_bullets时，传入当前图像和当前时间。
由于修改较大，下面给出部分代码示例：
在Tracker.h中：
// 在私有成员中添加
ShootParam current_shoot_param_;
AimCorrector aim_corrector_;
// 私有成员函数
void draw_simulated_bullets(cv::Mat& img, long long now_time);
在Tracker.cpp构造函数中：
// 初始化aim_corrector_，需要coorConverter和初始射击参数
aim_corrector_ = AimCorrector(coorConverter, ShootParam{});
在reconstruction_choose_compensation函数中，计算射击参数后：
current_shoot_param_ = shoot_param; // shoot_param是计算出的当前帧的射击参数
aim_corrector_.update_shoot_param(current_shoot_param_);
long long now_time = chrono::time_point_castchrono::milliseconds(chrono::system_clock::now()).time_since_epoch().count();
aim_corrector_.update_bullet(now_time);
然后，在需要绘制弹道的地方，调用成员函数draw_simulated_bullets。
注意：由于弹道模拟是连续的，我们需要在每一帧都更新子弹状态，所以每一帧都要调用aim_corrector_.update_bullet(now_time)，这样子弹列表才会更新（包括新发射的子弹和旧子弹的移除）。
但是，注意update_bullet函数会检查发射间隔，所以只有满足间隔条件时才会添加新子弹。而旧子弹的移除是在get_circles中进行的（当子弹击中目标后会被移除）。
这样修改后，子弹列表会持续存在，直到击中目标。
但是，由于我们没有在每一帧都调用get_circles，所以击中判断可能不及时。因此，我们可以在update_bullet中也进行子弹的更新和击中判断，或者保证每一帧都调用get_circles。
在当前的修改中，我们每一帧都会调用draw_simulated_bullets，而在这个函数中，我们会调用get_circles，所以子弹的更新和移除是及时的。
所以，现在回到最初的问题：为什么没有进入for循环？
在修改之后，子弹列表是持续存在的，并且每一帧都会更新，所以应该会进入for循环。
如果还是不能进入，请检查：
是否成功添加了子弹？即update_bullet函数中是否满足添加条件。
子弹的发射时间是否小于等于当前时间？注意我们传入的发射时间是当前时间，所以同一帧中，发射时间等于当前时间，不会跳过。
子弹列表是否为空？如果为空，则不会进入循环。
为了调试，你可以在get_circles函数中添加日志，输出子弹列表的大小，以及每个子弹的发射时间。
另外，注意时间单位：now_time和fire_t都是毫秒时间戳，所以比较时是相同的单位。
希望这些修改能解决问题。如果还有问题，请继续提供详细信息。
根据代码分析，没有进入for循环的原因主要有以下几个可能：
## 1. bullets列表为空
这是最常见的原因。检查以下几个方面：
在update_bullet中添加日志：
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 500;  // 发射间隔：200毫秒
    std::cout << "update_bullet called: current_time=" << current_time << ", last_fire_time=" << this->last_fire_time << ", diff=" << (current_time - this->last_fire_time) 
              << ", bullets.size()=" << bullets.size() << std::endl;if (current_time - this->last_fire_time >= fire_interval) {
        std::cout << "条件满足，尝试添加子弹..." << std::endl;if (bullets.size() < 20) {        // 最多显示10颗子弹
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;
            std::cout << "成功添加子弹，id=" << (next_id-1) 
                      << ", fire_t=" << current_time << std::endl;} else {
            std::cout << "子弹数量已达上限20" << std::endl;}} else {
        std::cout << "发射间隔未满足" << std::endl;}}
## 2. 时间初始化问题
在AimCorrector构造函数中：
cpp
AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {this->shoot_param = shoot_param;// 这里没有初始化 last_fire_time，使用的是成员变量默认初始化}
last_fire_time默认初始化可能是一个很大的值，导致条件判断不正确。
修正构造函数：
cpp
AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) 
    : coorConverter(coorConverter), 
      shoot_param(shoot_param),last_fire_time(0),  // 明确初始化为0next_id(0) {}
## 3. 时间单位问题
检查传入的current_time和last_fire_time是否使用相同的时间单位。在你的代码中：
cpp
// 在Tracker中使用long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
但在AimCorrector中：
cpp
long long last_fire_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
两个时间单位应该一致（都是毫秒）。
## 4. 添加调试信息到get_circles
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::cout << "get_circles called: now_time=" << now_time << ", bullets.size()=" << bullets.size() << std::endl;
    std::vector<IdCircle> res;if (bullets.empty()) {
        std::cout << "bullets列表为空，直接返回" << std::endl;return res;}int i = 0;for (auto it = this->bullets.begin(); it != this->bullets.end();) {
        std::cout << "处理子弹 " << i << ": id=" << it->id << ", fire_t=" << it->proj.get_fire_t() 
                  << ", now_time=" << now_time << std::endl;if (now_time < it->proj.get_fire_t()) {
            std::cout << "子弹还未发射，跳过" << std::endl;++it; 
            continue;}// ... 其余代码 ...++i;}return res;}
## 5. 在draw_simulated_bullets中添加调试
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time){
    std::cout << "=== draw_simulated_bullets开始 ===" << std::endl;
    std::cout << "传入时间: " << now_time << std::endl;
    std::cout << "子弹速度: " << shoot_param.v0 << std::endl;
    std::cout << "瞄准角度: " << shoot_param.aim_angle << std::endl;
    FlaskStream flask_aim;
    FlaskStream flask_map;
    flask_aim.clear();
    flask_map.clear();
    AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);// 更新子弹序列
    std::cout << "调用update_bullet前..." << std::endl;
    aim_corrector.update_bullet(now_time);
    std::cout << "调用update_bullet后..." << std::endl;
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
    std::cout << "获取到的子弹数量: " << bullets.size() << std::endl;// ... 其余代码 ...
    std::cout << "=== draw_simulated_bullets结束 ===" << std::endl;}
## 6. 可能的原因总结
原因A：条件判断不成立
cpp
// 如果last_fire_time初始值很大，这个条件永远不会成立if (current_time - this->last_fire_time >= fire_interval)
解决方案：在构造函数中初始化last_fire_time = 0
原因B：子弹发射时间比当前时间晚
cpp
// 如果子弹的fire_t比now_time大，会跳过if (now_time < it->proj.get_fire_t())
检查：在ProjectileSimulator构造函数中传入的fire_t是否正确
原因C：子弹立即被判定为击中
如果子弹一发射就被判定为击中，会立即从列表中删除，导致列表为空。
检查：在get_pos_by_t中，击中判定的逻辑是否过于宽松。
## 7. 快速诊断测试
添加这个测试函数来诊断问题：
cpp
void test_bullet_creation() {
    CoordinateTransformer* dummy_converter = nullptr; // 实际使用时需要真实对象
    ShootParam param;
    param.v0 = 20.0;
    param.aim_angle = 0.1;long long now_time = 1000; // 测试时间
    AimCorrector corrector(dummy_converter, param);// 第一次调用
    corrector.update_bullet(now_time);// 等待一段时间后再次调用
    std::this_thread::sleep_for(std::chrono::milliseconds(600));long long new_time = now_time + 600;
    corrector.update_bullet(new_time);// 检查子弹auto circles = corrector.get_circles(new_time);
    std::cout << "最终子弹数量: " << circles.size() << std::endl;}
最可能的问题是时间初始化不正确或发射间隔条件判断错误。添加上述调试输出后，应该能快速定位问题所在。
log file: /home/rm/.ros/log/cbda0f3a-dffc-11f0-8565-791c79470aa4/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [10756]
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
^C子弹列表大小: 0
子弹列表大小: 0
[tracker-5] killing on exit
[identify-4] killing on exit
[serial-3] killing on exit
[hikcamera-2] killing on exit
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
子弹列表大小: 0
[rosout-1] killing on e
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        cout << "子弹列表大小: " << bullets.size() << endl;
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
现在已经确定了这个子弹列表的大小是零
if (current_time - this->last_fire_time >= fire_interval) {现在已经确定了是没有进入这个if语句,所以这段代码为什么会出错,我应该怎么实现?
我们需要确定为什么条件不满足。从日志来看，子弹列表大小始终为0，说明没有成功添加子弹。而条件current_time - this->last_fire_time >= fire_interval不满足，可能的原因有：
current_time和last_fire_time的初始值问题，导致时间差不够大。
fire_interval的值设置得太大，导致条件难以满足。
时间单位不一致，导致计算的时间差不是预期的毫秒数。
查看代码中last_fire_time的初始化：
在AimCorrector类中，last_fire_time在构造函数中没有显式初始化，而是在类定义中初始化为当前时间（毫秒）：
cpp
long long last_fire_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
而在update_bullet函数中，我们传入的current_time也是通过同样的方式获取的。但是，由于last_fire_time在构造函数中被初始化为创建AimCorrector对象时的时间，而current_time是调用update_bullet时的时间。如果两次时间间隔小于fire_interval，条件就不满足。
另外，注意在draw_simulated_bullets函数中，我们创建了一个新的AimCorrector对象，然后立即调用update_bullet。这意味着last_fire_time被初始化为创建对象的时间，而传入的now_time是同一个函数被调用时的时间。由于这两者几乎是同时的，所以时间差几乎为0，小于fire_interval（200或500毫秒），因此条件不满足。
因此，我们需要调整逻辑，使得第一次调用update_bullet时就能添加一颗子弹。可以考虑将last_fire_time初始化为一个很早的时间，比如0，这样第一次调用时时间差就会很大，条件满足。
修改AimCorrector构造函数，将last_fire_time初始化为0：
cpp
AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) 
    : coorConverter(coorConverter), 
      shoot_param(shoot_param),last_fire_time(0),   // 初始化为0，这样第一次调用update_bullet时，条件就会满足next_id(0) {}
同时，在类定义中，移除last_fire_time的默认初始化，改为在构造函数初始化列表中初始化。
注意：在头文件中，last_fire_time的声明应该去掉默认初始化，即：
cpp
long long last_fire_time;  // 在构造函数中初始化
而不是：
cpp
long long last_fire_time = chrono::time_point_cast<chrono::milliseconds>(chrono::system_clock::now()).time_since_epoch().count();
这样修改后，第一次调用update_bullet时，current_time（当前时间）减去0会很大，满足条件，从而添加子弹。
另外，还需要注意fire_interval的值，你代码中注释写的是200毫秒，但实际是500。根据你的需求调整。
此外，还需要注意，在update_bullet中，我们添加子弹时传入的发射时间是current_time，而在get_circles中，我们检查子弹是否已发射的条件是now_time < it->proj.get_fire_t()，这意味着如果当前时间小于子弹的发射时间，子弹就不会被绘制。由于我们传入的current_time是当前时间，所以子弹的发射时间就是当前时间，而get_circles被调用时传入的now_time可能比current_time稍晚，所以子弹应该会被绘制。但是，如果get_circles在update_bullet之前被调用，那么子弹就不会被绘制，因为子弹列表是空的。
在draw_simulated_bullets函数中，我们先调用update_bullet，再调用get_circles，所以顺序是正确的。
总结修改步骤：
修改AimCorrector构造函数，将last_fire_time初始化为0。
修改头文件中last_fire_time的声明，去掉默认初始化。
确保fire_interval的值符合预期（比如200毫秒）。
修改后，应该就能正常添加子弹了。
如果仍然不行，请添加更多调试信息，比如在update_bullet中打印current_time、last_fire_time和它们的差值，以及fire_interval的值。
根据日志显示子弹列表大小始终为0，说明update_bullet函数中的条件判断没有进入。最可能的原因是时间比较逻辑有问题。让我分析并提供修复方案：
问题分析
## 1. 时间比较问题
cpp
if (current_time - this->last_fire_time >= fire_interval)
last_fire_time 可能在构造函数中被初始化为当前时间
每次调用 update_bullet 时，current_time 几乎等于 last_fire_time
导致时间差非常小，永远不会超过 fire_interval
## 2. 调试发现
你在日志中打印了子弹列表大小，但没有打印时间相关的调试信息。添加以下调试信息：
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 200毫秒
    std::cout << "update_bullet调试信息:" << std::endl;
    std::cout << "  current_time: " << current_time << std::endl;
    std::cout << "  last_fire_time: " << this->last_fire_time << std::endl;
    std::cout << "  时间差: " << (current_time - this->last_fire_time) << std::endl;
    std::cout << "  fire_interval: " << fire_interval << std::endl;
    std::cout << "  条件是否满足: " << ((current_time - this->last_fire_time) >= fire_interval) << std::endl;if (current_time - this->last_fire_time >= fire_interval) {
        std::cout << "  进入if语句，添加子弹..." << std::endl;if (bullets.size() < 10) {  // 减少到10颗，更容易观察
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;
            std::cout << "  添加子弹成功，id=" << (next_id-1) 
                      << ", 当前子弹数量: " << bullets.size() << std::endl;}} else {
        std::cout << "  条件未满足，不添加子弹" << std::endl;}}
根本原因和解决方案
方案1：初始化last_fire_time为0
这是最简单的解决方案：
cpp
// 修改构造函数AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param) 
    : coorConverter(coorConverter), 
      shoot_param(shoot_param),last_fire_time(0),  // 初始化为0，确保第一次就能进入ifnext_id(0) {}
方案2：修改条件判断逻辑
使用 > 而不是 >=，并确保初始值合理：
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 200毫秒// 如果是第一次调用或者时间差足够大if (this->last_fire_time == 0 || (current_time - this->last_fire_time) > fire_interval) {if (bullets.size() < 10) {
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;}}}
方案3：强制第一次调用时添加子弹
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 200毫秒// 如果还没有子弹，或者时间间隔满足，就添加新子弹bool should_add = bullets.empty() || 
                     (current_time - this->last_fire_time) >= fire_interval;if (should_add && bullets.size() < 10) {
        bullets.push_back(IdProj {
            next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;}}
完整修复代码
cpp
// trajectory_visualizer.hpp 中的类定义修改class AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param);// ... 其他成员函数private:
    std::list<IdProj> bullets;
    CoordinateTransformer* coorConverter;
    ShootParam shoot_param;long long next_id = 0;long long last_fire_time = 0;  // 初始化为0，不是当前时间};
cpp
// trajectory_visualizer.cpp 中的实现修改AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param) 
    : coorConverter(coorConverter), 
      shoot_param(shoot_param),last_fire_time(0),  // 明确初始化为0next_id(0) {}auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒// 调试信息
    std::cout << "update_bullet调试:" 
              << "当前时间=" << current_time << ", 上次发射时间=" << this->last_fire_time << ", 时间差=" << (current_time - this->last_fire_time) 
              << ", 子弹数量=" << bullets.size() << std::endl;// 如果满足发射间隔条件if (current_time - this->last_fire_time >= fire_interval) {if (bullets.size() < 10) {  // 限制最大子弹数量
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;
            std::cout << "添加子弹成功，id=" << (next_id-1) << std::endl;}}}
额外的调试建议
在 draw_simulated_bullets 函数中添加更多调试信息：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, cv::Mat& img, long long now_time) {
    std::cout << "\n=== draw_simulated_bullets 开始 ===" << std::endl;
    std::cout << "当前帧时间: " << now_time << std::endl;
    FlaskStream flask_aim;
    flask_aim.clear();
    AimCorrector aim_corrector = AimCorrector(coorConverter, shoot_param);// 第一次更新
    aim_corrector.update_bullet(now_time);// 等待一小段时间模拟long long later_time = now_time + 250;  // 250毫秒后
    std::cout << "\n250毫秒后..." << std::endl;
    aim_corrector.update_bullet(later_time);// 获取子弹
    std::vector<IdCircle> bullets = aim_corrector.get_circles(later_time);
    std::cout << "最终子弹数量: " << bullets.size() << std::endl;// ... 绘制代码 ...
    std::cout << "=== draw_simulated_bullets 结束 ===\n" << std::endl;}
最关键的是将 last_fire_time 初始化为 0 而不是当前时间，这样第一次调用时时间差会非常大，条件判断就会成立。
if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111111111111111111111111111111");
                ++it;
                continue;
            }
上面的问题解决了,这次我发现程序没有进入这个if语句,为什么?
我们进入get_circles函数时，子弹列表bullets不为空，但是却没有进入if (now_time < it->proj.get_fire_t())这个条件，这说明对于列表中的每个子弹，当前时间now_time都大于等于子弹的发射时间fire_t。
可能的原因：
子弹的发射时间fire_t设置不正确，可能设置成了过去的时间，导致now_time总是大于fire_t。
时间单位问题，now_time和fire_t的单位不一致。
由于我们之前在update_bullet中创建子弹时，将fire_t设置为current_time，而current_time就是调用update_bullet时的当前时间。在get_circles中传入的now_time是当前帧的时间。在正常的程序流程中，我们期望在某一帧创建子弹，然后在后续帧中更新子弹的位置。
但是，如果创建子弹的帧和更新子弹的帧是同一帧，那么now_time可能等于fire_t，这样就不会进入这个if语句。
另外，如果我们在创建子弹后，在下一帧调用get_circles，那么now_time应该大于fire_t，也不会进入这个if语句。
所以，这个if语句只有在子弹的发射时间晚于当前时间（即子弹还没有发射）的情况下才会进入。在我们当前的逻辑中，子弹一旦被创建，就认为是已经发射了（因为fire_t被设置为当前时间），所以不会进入这个if语句。
但是，我们之前为了模拟子弹发射间隔，在update_bullet中设置了发射间隔，并且只有在满足间隔条件时才创建子弹。那么，当我们创建子弹时，我们设置的fire_t就是当前时间，而now_time也是当前时间，所以now_time等于fire_t，不满足now_time < fire_t，因此不会进入这个if语句。
那么，这个if语句的设计意图是什么？看起来是想跳过那些尚未发射的子弹。但是，在我们的设计中，子弹一旦被创建，就认为是已经发射了。所以，这个if语句可能永远不会进入，除非我们在创建子弹时，将fire_t设置为未来的某个时间（比如考虑击发延迟）。
实际上，在真实的射击过程中，从按下射击键到子弹出膛有一个击发延迟，我们可能需要在fire_t上加上这个延迟，这样子弹在出膛前的一段时间内不会被绘制。
但是，根据你的代码，ProjectileSimulator的构造函数中传入的fire_t就是current_time，即子弹的发射时间就是创建模拟器的时间。那么，在绘制第一帧时，子弹已经发射，所以不会进入这个if语句。
所以，问题可能在于：我们是否需要模拟击发延迟？如果需要，那么应该将fire_t设置为未来某个时间（比如current_time + 延迟时间）。这样，在子弹出膛前，now_time < fire_t，就会进入if语句，跳过这颗子弹。
然而，在你的代码中，我注意到在Tracker的reconstruction_choose_compensation函数中，你计算了m_time_off = COMMAND_TIMESPAN + eTime + 0.025，并用于计算击中时间。但是，在创建子弹模拟器时，你并没有考虑这个延迟。
在Tracker中，你创建ProjectileSimulator时使用的是当前时间：
cpp
ProjectileSimulator(coorConverter, this->shoot_param, current_time)
但是，在真实的射击过程中，子弹的出膛时间应该是当前时间加上击发延迟。所以，你可能需要将fire_t设置为current_time + m_time_off。
然而，在draw_simulated_bullets函数中，你创建AimCorrector并调用update_bullet时，并没有传入这个延迟。所以，你需要修改AimCorrector的设计，使其能够考虑击发延迟。
但是，请注意，AimCorrector类目前只有一个ShootParam参数，没有延迟参数。你可能需要将延迟参数作为ShootParam的一部分，或者在AimCorrector的构造函数中增加一个延迟参数。
另一种思路是：在Tracker中，你已经计算了击中时间，那么你可以将子弹的发射时间设置为当前时间，然后子弹的飞行时间就是击中时间减去当前时间。但是，在子弹模拟器中，我们是从发射开始模拟的，所以发射时间就是当前时间加上延迟。
由于你的代码中已经有一个固定的延迟COMMAND_TIMESPAN + eTime + 0.025，我建议在创建子弹模拟器时，将这个延迟加上。
修改AimCorrector的update_bullet函数，将子弹的发射时间设置为当前时间加上延迟。但是，这个延迟需要从外部传入。
由于AimCorrector是在draw_simulated_bullets函数中创建的，而draw_simulated_bullets是在Tracker的reconstruction_choose_compensation函数中调用的，并且Tracker中已经计算了m_time_off，所以你可以将m_time_off作为参数传递给draw_simulated_bullets，然后再传递给AimCorrector。
但是，这可能需要修改多个函数。另一种简单的方法是：在ShootParam结构体中增加一个fire_delay字段，然后在创建子弹模拟器时使用。
由于时间关系，我建议先修改ShootParam结构体，增加延迟字段，然后在创建子弹模拟器时加上这个延迟。
步骤：
在ShootParam结构体中增加一个成员变量，比如fire_delay，表示击发延迟（单位：毫秒）。
在Tracker中设置shoot_param时，将m_time_off赋值给fire_delay（注意单位转换，m_time_off是秒，而fire_delay可能是毫秒）。
在AimCorrector的update_bullet函数中，创建子弹模拟器时，将fire_t设置为current_time + shoot_param.fire_delay。
这样，子弹的发射时间就是当前时间加上延迟，那么在第一帧时，如果延迟大于0，那么now_time可能小于fire_t，就会进入if语句，跳过这颗子弹，直到时间超过延迟，子弹才真正发射。
但是，请注意，你的get_circles函数中，对于每一颗子弹，如果now_time < fire_t，就跳过，这意味着在延迟期间，这颗子弹不会被绘制。这符合预期吗？
另外，你的get_pos_by_t函数中，计算子弹位置时使用的t是传入的时间，它应该大于等于fire_t。如果t < fire_t，那么子弹还没有发射，应该返回什么位置？可能是枪口位置？但是，根据你的代码，get_pos_by_t函数没有处理t < fire_t的情况，所以可能返回错误的位置。
因此，我建议在get_pos_by_t函数中，如果t < fire_t，则返回枪口位置（即初始位置）。但是，你的代码中，子弹的初始位置是哪里？在get_pos_by_t函数中，当t等于fire_t时，子弹应该刚好在枪口。
由于你的弹道模型是从枪口开始计算的，所以当t < fire_t时，子弹应该还在枪膛内，我们可能不想绘制它。所以，在get_circles函数中跳过t < fire_t的子弹是正确的。
但是，为了确保get_pos_by_t函数在t < fire_t时不会出现异常（比如对数函数中真数为负数），我们可以在get_pos_by_t函数中判断，如果t < fire_t，则返回一个特殊的HitPos，其中hit为false，位置为枪口位置（或者零位置）。不过，由于我们在get_circles中已经跳过了，所以可以假设get_pos_by_t不会被用于t < fire_t的情况。
综上所述，修改方案：
在ShootParam中增加fire_delay（单位：毫秒）。
在Tracker中，设置shoot_param.fire_delay = (COMMAND_TIMESPAN + eTime + 0.025) * 1000; // 转换为毫秒
在AimCorrector的update_bullet中，创建子弹时，将发射时间设置为当前时间加上延迟：
cpp
bullets.push_back(IdProj {
    next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time + this->shoot_param.fire_delay)});
这样，在子弹延迟期间，now_time < fire_t，就会进入if语句，跳过这颗子弹。
但是，请注意，你的get_circles函数中，now_time是调用时传入的当前时间，而子弹的fire_t是发射时间。这样，当now_time小于fire_t时，子弹不会显示，直到now_time大于等于fire_t。
另外，你的ProjectileSimulator的get_pos_by_t函数中，计算弹道时使用的t是绝对时间（毫秒），而fire_t也是绝对时间（毫秒）。所以，当t大于等于fire_t时，计算的是子弹发射后的位置。
这样修改后，你应该能看到子弹在延迟一段时间后才出现。
但是，请注意，你的AimCorrector的update_bullet函数中，每次满足发射间隔时就会创建一颗子弹，而每颗子弹都有相同的延迟。这样，第一颗子弹会在current_time + fire_delay时发射，第二颗子弹会在current_time + fire_interval + fire_delay时发射，以此类推。
现在，让我们回到最初的问题：为什么没有进入if语句？因为之前我们没有加延迟，所以fire_t等于current_time，而now_time是当前帧的时间，可能等于或大于fire_t。现在加上延迟后，fire_t大于current_time，而now_time可能小于fire_t，这样就会进入if语句。
但是，请注意，在draw_simulated_bullets函数中，你创建AimCorrector后立即调用update_bullet，然后调用get_circles，传入的now_time是同一个时间。所以，如果now_time小于fire_t，那么子弹不会被绘制。
为了测试，你可以将fire_delay设置为一个较小的值，比如50毫秒，然后观察子弹是否在50毫秒后出现。
由于你的代码中，draw_simulated_bullets每帧都会被调用，所以你可以看到子弹在一段时间后出现。
但是，请注意，你的update_bullet函数中，发射间隔是500毫秒，所以每500毫秒才会创建一颗子弹。而第一颗子弹的延迟是fire_delay，所以第一颗子弹会在程序开始后的fire_delay毫秒后出现，然后每500毫秒出现一颗。
另外，你需要确保fire_delay小于500毫秒，否则第二颗子弹可能会在第一颗子弹之前出现（因为第二颗子弹的发射时间是current_time + fire_interval + fire_delay，而第一颗是current_time + fire_delay）。
最后，由于你是在Tracker的每一帧中调用draw_simulated_bullets，所以每帧都会创建新的AimCorrector对象，这样之前的状态（子弹列表）就会丢失。你可能希望AimCorrector对象能够持续存在，而不是每帧都新建。这可能需要你将AimCorrector作为Tracker的成员变量，而不是在函数内部创建。
但是，根据你的需求，如果你只是想在每一帧中绘制当前帧的子弹弹道，那么每帧新建也是可以的，只不过这样子弹列表会被重置，你只能看到当前帧发射的子弹（而且由于发射间隔，可能很多帧都不会有子弹）。
所以，为了保持子弹的状态，你应该将AimCorrector作为Tracker的成员变量，并在Tracker的构造函数中初始化它，然后在每一帧中更新和绘制。
由于这是一个较大的改动，你需要根据你的项目需求来决定。
如果你只是想调试弹道，那么每帧新建可能问题不大，但是要注意，这样你只能看到当前帧发射的子弹，而且由于发射间隔，你可能需要等待一段时间才能看到子弹。
为了快速看到效果，你可以暂时将发射间隔fire_interval设置得小一些，比如100毫秒。
总结一下，要解决进入if语句的问题，你需要：
在ShootParam中增加fire_delay。
在Tracker中设置shoot_param.fire_delay为击发延迟（毫秒）。
在创建子弹模拟器时，将发射时间设置为当前时间加上延迟。
这样，在延迟时间内，now_time会小于fire_t，就会进入if语句跳过。
但是，请注意，如果你的目的是为了模拟真实的子弹发射（包括击发延迟），那么这样做是正确的。如果你的目的只是为了可视化弹道，不考虑击发延迟，那么你可以不加延迟，而是直接将发射时间设置为当前时间，这样就不会进入if语句，子弹会立即被绘制。
所以，是否进入if语句取决于你的需求。如果你希望子弹立即被绘制，那么就不要加延迟，这样now_time等于fire_t，不会进入if语句。如果你希望模拟击发延迟，那么加上延迟，这样在延迟期间，now_time小于fire_t，就会进入if语句跳过。
根据你的问题，你发现没有进入if语句，那么说明你没有加延迟，而且now_time大于等于fire_t。如果你希望进入if语句，那么就需要加上延迟。
但是，请记住，get_circles函数中，对于每一颗子弹，如果now_time < fire_t，就会跳过，这颗子弹就不会被绘制。所以，如果你加了延迟，那么在延迟期间，子弹是不可见的。
所以，请根据你的需求决定。
由于你之前的问题已经解决（子弹列表不为空），现在的问题是没有进入if语句，那么你可以选择是否要加延迟。如果你不需要延迟，那么你可以删除这个if语句，或者将条件改为now_time <= fire_t，这样在发射时间等于当前时间时也会跳过（但实际上，子弹在发射瞬间就应该被绘制，所以可能不应该跳过）。
我认为，在子弹发射的瞬间，我们应该绘制子弹，所以当now_time == fire_t时，不应该跳过。所以，你可以将条件改为now_time < fire_t，这样在发射时间等于当前时间时，就会绘制子弹。
所以，如果你不想加延迟，而且希望子弹在创建后立即被绘制，那么你应该确保fire_t小于等于now_time。在创建子弹时，将fire_t设置为当前时间，那么now_time等于fire_t，不会进入if语句，子弹会被绘制。
因此，你的代码中，如果没有加延迟，那么fire_t等于current_time，而now_time是同一帧的时间，所以相等，不会进入if语句。
所以，现在你可以选择：
不加延迟，并且希望子弹立即绘制：保持原样，不用进入if语句。
加延迟，模拟击发延迟：按照上述修改，这样在延迟期间，子弹不会绘制，直到延迟结束。
根据你的需求，如果你想模拟击发延迟，那么请按照上述步骤修改。如果不想，那么你可以删除这个if语句，或者将其条件改为now_time <= fire_t，这样在发射时间等于当前时间时也会跳过（但这样发射瞬间的子弹就不会被绘制）。
但是，我认为在发射瞬间，子弹应该被绘制在枪口位置。所以，我建议不要跳过now_time == fire_t的情况。
所以，你可以将条件改为now_time < fire_t，这样在等于的时候就会绘制。
这样，即使你不加延迟，子弹也会在创建的第一帧被绘制。
所以，现在的问题是你希望子弹在什么时候被绘制？如果你希望子弹在创建的第一帧就被绘制，那么你应该确保fire_t小于等于now_time。由于你创建子弹时使用的是current_time，而now_time是同一帧的时间，所以相等，不会进入if语句，会被绘制。
因此，你的代码目前的行为是：子弹在创建的第一帧就会被绘制。如果你希望这样，那么没有问题。
如果你希望子弹在创建后延迟一段时间再绘制，那么就需要加延迟。
根据你的问题描述，你发现没有进入if语句，这是因为你的fire_t设置成了当前时间，而now_time等于当前时间。所以，如果你希望进入if语句，就需要让fire_t大于now_time，即设置一个未来的发射时间。
所以，请根据你的需求决定是否修改。
最后，请注意，你的get_pos_by_t函数在t等于fire_t时，计算出的子弹位置应该是枪口位置。所以，在第一帧绘制时，子弹在枪口。
希望这能帮助你理解问题所在。
这个问题说明当前时间 now_time 已经大于等于子弹的发射时间 fire_t，也就是说子弹已经发射了。让我们分析一下为什么会出现这种情况：
原因分析
## 1. 时间创建顺序
在你的代码中，子弹的创建和获取发生在同一帧：
cpp
// 在draw_simulated_bullets函数中：
aim_corrector.update_bullet(now_time);      // 创建子弹，fire_t = now_time
std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);  // 同一时间查询
这意味着：
update_bullet(now_time) 创建子弹，fire_t = now_time
get_circles(now_time) 查询子弹，now_time == fire_t
所以 now_time < fire_t 为 false
## 2. 时间精度问题
即使你希望子弹稍后发射，但如果你使用的是毫秒级时间戳，在同一函数调用中，now_time 几乎不可能小于 fire_t。
解决方案
方案1：添加发射延迟
这是最合理的解决方案。子弹应该有发射延迟（考虑击发时间）：
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒const long long fire_delay = 50;      // 发射延迟：50毫秒if (current_time - this->last_fire_time >= fire_interval) {if (bullets.size() < 10) {// 添加发射延迟：子弹会在50毫秒后才真正发射long long actual_fire_time = current_time + fire_delay;
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, actual_fire_time)});this->last_fire_time = current_time;}}}
方案2：在Tracker中添加延迟
在Tracker的调用中添加延迟更符合实际情况：
cpp
// 在Tracker的reconstruction_choose_compensation函数中：
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);// 添加发射延迟（电控延迟+击发延迟）double m_time_off = COMMAND_TIMESPAN + eTime + 0.025;  // 你之前计算的总延迟long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(
    chrono::system_clock::now()).time_since_epoch().count();// 子弹将在未来发射，所以使用未来的时间long long fire_time = draw_visual_now_time + static_cast<long long>(m_time_off * 1000);
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_, fire_time);
方案3：修改判断逻辑
如果你想让子弹立即显示，可以修改判断逻辑：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 修改为：如果子弹还没到发射时间，使用发射时间进行计算long long effective_time = now_time;if (now_time < it->proj.get_fire_t()) {// 子弹还未发射，但我们仍然可以显示（在枪口位置）
            effective_time = it->proj.get_fire_t();}
        HitCircle hit_circle = it->proj.get_circle_by_t(effective_time);if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
完整的调试代码
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;ROS_INFO("get_circles调用 - now_time: %lld, 子弹数量: %zu",             now_time, bullets.size());int bullet_index = 0;for (auto it = this->bullets.begin(); it != this->bullets.end(); ++bullet_index) {long long fire_t = it->proj.get_fire_t();ROS_INFO("子弹[%d] - id: %d, fire_t: %lld, now_time: %lld, now_time < fire_t: %s",                 bullet_index, it->id, fire_t, now_time, 
                 (now_time < fire_t) ? "true" : "false");if (now_time < fire_t) {ROS_WARN("子弹[%d]还未发射，跳过", bullet_index);++it; 
            continue;}// 获取子弹在当前时刻的投影圆
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {ROS_INFO("子弹[%d]已击中，删除", bullet_index);
            it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果ROS_INFO("子弹[%d]未击中，添加到结果", bullet_index);
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}ROS_INFO("get_circles返回 - 有效子弹数量: %zu", res.size());return res;}
推荐的解决方案
最佳方案是方案1 + 方案2的组合：
在AimCorrector中添加固定的发射延迟：
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 200;  // 发射间隔const long long fire_delay = 100;     // 发射延迟100msif (current_time - this->last_fire_time >= fire_interval) {if (bullets.size() < 10) {
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param,                                   current_time + fire_delay)  // 延迟发射});this->last_fire_time = current_time;}}}
在draw_simulated_bullets中确保使用正确的时间：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time) {// 注意：now_time应该是当前图像的时间戳
    AimCorrector aim_corrector = AimCorrector(coorConverter, shoot_param);// 更新子弹（可能创建新子弹）
    aim_corrector.update_bullet(now_time);// 获取子弹在当前时刻的状态
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);// ... 绘制代码 ...}
这样，子弹会有100ms的发射延迟，在子弹创建后的前100ms内，now_time < fire_t 会成立，你就能看到111111...的警告信息了。100ms后子弹才会开始运动并被绘制。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        double k = 1;   // 空气阻力系数
        // 计算水平位移
        double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111111111111111111111111111111");
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 200;  // 发射间隔：200毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)
                    // ProjectileSimulator(coorConverter, this->shoot_param, current_time)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time,eTime,COMMAND_TIMESPAN);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
#include "ros/ros.h"
namespace tools{
    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点
    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };
    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };
    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };
    /*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    
/
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char
 str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };
    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };
    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置
    };
    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };
    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };
    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {}
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t) const -> HitCircle;
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t) const -> HitPos;
        // 获取开火时间
        auto get_fire_t() const -> double;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };
    // 子弹位置信息
    struct IdPos {
        int id;
        Eigen::Vector3d pos;
    };
    // 子弹投影圆信息
    struct IdCircle {
        int id;
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };
    // 子弹模拟器封装
    struct IdProj {
        int id;
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time) -> std::vector<IdCircle>;
        auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;
        long long next_id = 0;
        long long last_fire_time = 0;
    };
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN);
}
#endif // TRAJECTORY_VISUALIZER_HPP
rm@rm-NUC11PAHi7:~/ws_glut_vison$ catkin_make
Base path: /home/rm/ws_glut_vison
Source space: /home/rm/ws_glut_vison/src
Build space: /home/rm/ws_glut_vison/build
Devel space: /home/rm/ws_glut_vison/devel
Install space: /home/rm/ws_glut_vison/install
Running command: "make cmake_check_build_system" in "/home/rm/ws_glut_vison/build"
Running command: "make -j8 -l8" in "/home/rm/ws_glut_vison/build"
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target geometry_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target geometry_msgs_generate_messages_eus
[  0%] Built target geometry_msgs_generate_messages_cpp
[  0%] Built target std_msgs_generate_messages_eus
[  5%] Built target hikcamera
[  5%] Built target std_msgs_generate_messages_lisp
[  5%] Built target geometry_msgs_generate_messages_lisp
[  5%] Built target std_msgs_generate_messages_nodejs
[  5%] Built target _rm_msgs_generate_messages_check_deps_Debug
[  5%] Built target _rm_msgs_generate_messages_check_deps_ArmorArray
[  5%] Built target _rm_msgs_generate_messages_check_deps_Armor
[  5%] Built target geometry_msgs_generate_messages_nodejs
[  5%] Built target _rm_msgs_generate_messages_check_deps_RmSerial
[  7%] Building CXX object rm_serial/CMakeFiles/serial.dir/src/serial.cpp.o
[ 20%] Built target rm_msgs_generate_messages_py
[ 30%] Built target rm_msgs_generate_messages_cpp
[ 40%] Built target rm_msgs_generate_messages_lisp
[ 55%] Built target rm_msgs_generate_messages_eus
[ 57%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/math.cpp.o
[ 60%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/CoorConverter.cpp.o
[ 60%] Building CXX object rm_identify/CMakeFiles/identify.dir/src/identify.cpp.o
[ 62%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o
[ 65%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/MPC.cpp.o
[ 75%] Built target rm_msgs_generate_messages_nodejs
[ 77%] Building CXX object rm_identify/CMakeFiles/identify_test.dir/src/identify_test.cpp.o
[ 77%] Built target rm_msgs_generate_messages
[ 80%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o
In file included from /home/rm/ws_glut_vison/src/rm_serial/src/serial.cpp:2:
/home/rm/ws_glut_vison/src/rm_serial/include/Serial.h: In member function ‘long long int SerialPort::timestamp(unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char, unsigned char)’:
/home/rm/ws_glut_vison/src/rm_serial/include/Serial.h:545:28: warning: left shift count >= width of type [-Wshift-count-overflow]
  545 |   temp = (uint32_t)h_32 << 32 | (uint32_t) l_32;
      |                            ^~
/home/rm/ws_glut_vison/src/rm_serial/src/serial.cpp: At global scope:
/home/rm/ws_glut_vison/src/rm_serial/src/serial.cpp:15:32: warning: ISO C++ forbids converting a string constant to ‘char*’ [-Wwrite-strings]
   15 | constexpr char* SerialPrefix = "/dev/";
      |                                ^~~~~~~
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:81:10: error: no declaration matches ‘void tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
   81 |     auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
      |          ^~~~~~~~~~~~
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: note: candidate is: ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:155:11: note: ‘class tools::AimCorrector’ defined here
  155 |     class AimCorrector {
      |           ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp: In function ‘void tools::draw_simulated_bullets(CoordinateTransformer*, const tools::ShootParam&, cv::Mat&, long long int, long long int, long long int)’:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:230:45: error: no matching function for call to ‘tools::AimCorrector::update_bullet(long long int&)’
  230 |         aim_corrector.update_bullet(now_time);
      |                                             ^
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: note: candidate: ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: note:   candidate expects 3 arguments, 1 provided
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:232:98: error: no matching function for call to ‘tools::AimCorrector::get_circles(long long int&, long long int&, long long int&)’
  232 |         std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time,eTime,COMMAND_TIMESPAN);
      |                                                                                                  ^
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:50:10: note: candidate: ‘std::vector<tools::IdCircle> tools::AimCorrector::get_circles(long long int)’
   50 |     auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
      |          ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:50:10: note:   candidate expects 1 argument, 3 provided
[ 82%] Linking CXX executable /home/rm/ws_glut_vison/devel/lib/rm_serial/serial
[ 82%] Built target serial
In file included from /home/rm/ws_glut_vison/src/rm_identify/include/new_Rmidentify.hpp:9,
                 from /home/rm/ws_glut_vison/src/rm_identify/src/identify_test.cpp:2:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h: In member function ‘float Light::cal_ang(cv::Point2f, cv::Point2f, cv::Point2f)’:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:246:13: warning: NULL used in arithmetic [-Wpointer-arith]
  246 |    if (B != NULL)
      |             ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h: In member function ‘std::vector<cv::Point_<float> > Light::refine_the_armor(cv::Mat, int)’:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:19: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                   ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:37: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                     ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:55: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                                       ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:73: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                                                         ^~~~
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:132：rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o] 错误 1
make[2]: *** 正在等待未完成的任务....
In file included from /home/rm/ws_glut_vison/src/rm_identify/include/RmIdentify.hpp:10,
                 from /home/rm/ws_glut_vison/src/rm_identify/src/identify.cpp:2:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h: In member function ‘float Light::cal_ang(cv::Point2f, cv::Point2f, cv::Point2f)’:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:246:13: warning: NULL used in arithmetic [-Wpointer-arith]
  246 |    if (B != NULL)
      |             ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h: In member function ‘std::vector<cv::Point_<float> > Light::refine_the_armor(cv::Mat, int)’:
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:19: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                   ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:37: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                     ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:55: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                                       ^~~~
/home/rm/ws_glut_vison/src/rm_identify/include/Light.h:367:73: warning: NULL used in arithmetic [-Wpointer-arith]
  367 |     if (angle1 != NULL && angle2 != NULL && angle3 != NULL && angle4 != NULL)
      |                                                                         ^~~~
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/RmTracker.hpp:34,
                 from /home/rm/ws_glut_vison/src/rm_tracker/src/tracker.cpp:3:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
In file included from /home/rm/ws_glut_vison/src/rm_identify/include/new_Rmidentify.hpp:12,
                 from /home/rm/ws_glut_vison/src/rm_identify/src/identify_test.cpp:2:
/home/rm/ws_glut_vison/src/rm_identify/include/new_detector.h: In function ‘float new_Identify::cal_ang(cv::Point2f, cv::Point2f, cv::Point2f)’:
/home/rm/ws_glut_vison/src/rm_identify/include/new_detector.h:568:13: warning: NULL used in arithmetic [-Wpointer-arith]
  568 |    if (B != NULL)
      |             ^~~~
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:76：rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:2125：rm_tracker/CMakeFiles/tracker.dir/all] 错误 2
make[1]: *** 正在等待未完成的任务....
[ 85%] Linking CXX executable /home/rm/ws_glut_vison/devel/lib/rm_identify/identify
/usr/bin/ld: warning: libopencv_imgcodecs.so.4.2, needed by /opt/ros/noetic/lib/libcv_bridge.so, may conflict with libopencv_imgcodecs.so.407
/usr/bin/ld: warning: libopencv_features2d.so.4.2, needed by /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0, may conflict with libopencv_features2d.so.407
/usr/bin/ld: warning: libopencv_imgproc.so.407, needed by /usr/local/lib/libopencv_imgcodecs.so.4.7.0, may conflict with libopencv_imgproc.so.4.2
/usr/bin/ld: warning: libopencv_core.so.407, needed by /usr/local/lib/libopencv_imgcodecs.so.4.7.0, may conflict with libopencv_core.so.4.2
[ 90%] Built target identify
[ 92%] Linking CXX executable /home/rm/ws_glut_vison/devel/lib/rm_identify/identify_test
/usr/bin/ld: warning: libopencv_imgcodecs.so.4.2, needed by /opt/ros/noetic/lib/libcv_bridge.so, may conflict with libopencv_imgcodecs.so.407
/usr/bin/ld: warning: libopencv_features2d.so.4.2, needed by /usr/lib/x86_64-linux-gnu/libopencv_calib3d.so.4.2.0, may conflict with libopencv_features2d.so.407
/usr/bin/ld: warning: libopencv_imgproc.so.407, needed by /usr/local/lib/libopencv_imgcodecs.so.4.7.0, may conflict with libopencv_imgproc.so.4.2
/usr/bin/ld: warning: libopencv_core.so.407, needed by /usr/local/lib/libopencv_imgcodecs.so.4.7.0, may conflict with libopencv_core.so.4.2
[ 97%] Built target identify_test
make: *** [Makefile:146：all] 错误 2
Invoking "make -j8 -l8" failed
rm@rm-NUC11PAHi7:~/ws_glut_vison$ catkin_make
Base path: /home/rm/ws_glut_vison
Source space: /home/rm/ws_glut_vison/src
Build space: /home/rm/ws_glut_vison/build
Devel space: /home/rm/ws_glut_vison/devel
Install space: /home/rm/ws_glut_vison/install
Running command: "make cmake_check_build_system" in "/home/rm/ws_glut_vison/build"
Running command: "make -j8 -l8" in "/home/rm/ws_glut_vison/build"
[  0%] Built target geometry_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_cpp
[  5%] Built target hikcamera
[  5%] Built target geometry_msgs_generate_messages_cpp
[  5%] Built target std_msgs_generate_messages_eus
[  5%] Built target geometry_msgs_generate_messages_eus
[  5%] Built target _rm_msgs_generate_messages_check_deps_RmSerial
[  5%] Built target _rm_msgs_generate_messages_check_deps_ArmorArray
[  5%] Built target geometry_msgs_generate_messages_lisp
[  5%] Built target std_msgs_generate_messages_nodejs
[  5%] Built target std_msgs_generate_messages_lisp
[  5%] Built target geometry_msgs_generate_messages_nodejs
[  5%] Built target _rm_msgs_generate_messages_check_deps_Armor
[  5%] Built target _rm_msgs_generate_messages_check_deps_Debug
[ 15%] Built target rm_msgs_generate_messages_cpp
[ 20%] Built target serial
[ 32%] Built target rm_msgs_generate_messages_eus
[ 45%] Built target rm_msgs_generate_messages_py
[ 55%] Built target identify
[ 65%] Built target identify_test
[ 85%] Built target rm_msgs_generate_messages_nodejs
[ 85%] Built target rm_msgs_generate_messages_lisp
[ 85%] Built target rm_msgs_generate_messages
[ 87%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o
[ 90%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:81:10: error: no declaration matches ‘void tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
   81 |     auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
      |          ^~~~~~~~~~~~
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: note: candidate is: ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:155:11: note: ‘class tools::AimCorrector’ defined here
  155 |     class AimCorrector {
      |           ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp: In function ‘void tools::draw_simulated_bullets(CoordinateTransformer*, const tools::ShootParam&, cv::Mat&, long long int, long long int, long long int)’:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:230:68: error: use of ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’ before deduction of ‘auto’
  230 |         aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
      |                                                                    ^
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:232:98: error: no matching function for call to ‘tools::AimCorrector::get_circles(long long int&, long long int&, long long int&)’
  232 |         std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time,eTime,COMMAND_TIMESPAN);
      |                                                                                                  ^
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:50:10: note: candidate: ‘std::vector<tools::IdCircle> tools::AimCorrector::get_circles(long long int)’
   50 |     auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
      |          ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:50:10: note:   candidate expects 1 argument, 3 provided
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:132：rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o] 错误 1
make[2]: *** 正在等待未完成的任务....
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/RmTracker.hpp:34,
                 from /home/rm/ws_glut_vison/src/rm_tracker/src/tracker.cpp:3:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:76：rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:2125：rm_tracker/CMakeFiles/tracker.dir/all] 错误 2
make: *** [Makefile:146：all] 错误 2
Invoking "make -j8 -l8" failed
这里为什么报错?
根据编译错误信息，有以下几个问题需要解决：
头文件中的函数声明与实现不匹配：
在头文件 trajectory_visualizer.hpp 中，AimCorrector 类的 update_bullet 成员函数声明为三个参数，但是实现时（在 trajectory_visualizer.cpp 中）却定义了一个参数的版本（get_circles 也有类似问题，但错误信息显示是调用时参数不匹配，实际上get_circles在头文件中声明为一个参数，但实现时也是一个参数，但是在draw_simulated_bullets中调用时却传了三个参数）。
多余的作用域限定符：
在头文件中，函数声明时使用了 auto AimCorrector::update_bullet(...); 这种形式，这是错误的。在类内部声明成员函数时，不需要加上类作用域。
函数调用时的参数数量不匹配：
在 draw_simulated_bullets 函数中，调用 aim_corrector.update_bullet 和 aim_corrector.get_circles 时传递的参数数量与函数声明不一致。
让我们逐一修正：
修正步骤：
修正头文件中的函数声明（trajectory_visualizer.hpp）
在类 AimCorrector 中，声明函数时不要加类作用域，并且确保参数列表与实现一致。
原头文件中的错误声明：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
应改为（在类内部）：
cpp
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
同时，get_circles 在头文件中声明为一个参数，但是我们在调用时却传了三个参数。根据实现，get_circles 确实只需要一个参数 now_time。因此，我们需要修改调用处，使其只传递一个参数。
修正实现文件中的函数定义（trajectory_visualizer.cpp）
在实现文件中，函数定义应该与头文件中的声明一致。
对于 update_bullet，我们已经在头文件中改为三个参数，因此实现部分也需要相应修改（实际上在实现部分已经是三个参数，但函数定义前的类作用域写法有误）。
在实现文件中，定义成员函数时应该使用类作用域，但不要加 auto 关键字在返回类型前（除非使用尾置返回类型，这里我们使用了尾置返回类型，所以保留 auto，但要去掉头文件中声明时的类作用域）。
修正函数调用（trajectory_visualizer.cpp 中的 draw_simulated_bullets）
在 draw_simulated_bullets 函数中，我们调用 aim_corrector.update_bullet 时应该传递三个参数，而调用 aim_corrector.get_circles 时只传递一个参数。
具体修改：
修改头文件（trajectory_visualizer.hpp）：
cpp
// 在 AimCorrector 类中，修改 update_bullet 的声明auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;// get_circles 的声明保持不变（一个参数）auto get_circles(long long now_time) -> std::vector<IdCircle>;
修改实现文件（trajectory_visualizer.cpp）：
cpp
// 修改 update_bullet 函数定义，去掉类作用域（但实现时要用类作用域）auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {// 函数体}// get_circles 函数定义保持不变auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{// 函数体}
修改 draw_simulated_bullets 函数中的调用：
cpp
// 更新子弹序列，传入三个参数
aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);// 获取子弹，只传入一个参数
std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
注意：
头文件中的函数声明不需要加 AimCorrector::，因为这是在类内部。
实现文件中的函数定义需要加 AimCorrector:: 以指明这是类的成员函数。
函数调用时，参数数量必须与函数声明一致。
根据以上分析，我们需要修改头文件和实现文件中的相关部分，并确保调用时传递正确数量的参数。
由于错误信息还指出 draw_simulated_bullets 函数调用 get_circles 时传递了三个参数，但函数只接受一个参数，因此需要修改调用方式。
另外，在头文件中，我们声明了 update_bullet 和 get_circles，但是它们的参数列表需要和实现一致。根据你的代码，update_bullet 需要三个参数（current_time, eTime, COMMAND_TIMESPAN），而 get_circles 只需要一个参数（now_time）。
因此，在 draw_simulated_bullets 中，调用 get_circles 时只传递一个参数，即当前时间 now_time。
修改后的代码示例：
trajectory_visualizer.hpp 中的 AimCorrector 类：
cpp
class AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param);// 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆auto get_circles(long long now_time) -> std::vector<IdCircle>;auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;private:// ... 其他成员};
trajectory_visualizer.cpp 中的函数定义：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)// ProjectileSimulator(coorConverter, this->shoot_param, current_time)});this->last_fire_time = current_time;}}}auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{// 函数体不变}
draw_simulated_bullets 函数中的调用：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){// ... 其他代码
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);  // 只传递一个参数// ... 绘制代码}
经过以上修改，应该可以解决编译错误。请注意，在修改头文件后，需要确保包含该头文件的源文件重新编译。因此，建议进行完整的重新构建（clean 然后 build）。
根据编译错误，有多个问题需要解决。让我逐一分析并给出修复方案：
主要错误分析
## 1. 类作用域限定符错误
cpp
// 错误：在类声明内部使用了作用域限定符auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);// 正确：应该在类声明内部直接声明auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
## 2. 函数参数不匹配
get_circles 声明为一个参数，但调用时传入三个参数
函数签名不一致
修复方案
## 1. 修改头文件 (trajectory_visualizer.hpp)
cpp
#ifndef TRAJECTORY_VISUALIZER_HPP#define TRAJECTORY_VISUALIZER_HPP#include "math.hpp"#include "CoorConverter.hpp"#include <opencv2/opencv.hpp>#include "GimbalPos.hpp"#include "ros/ros.h"namespace tools{const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点// 点绘制参数struct FlaskPoint {FlaskPoint(const cv::Point2d& pt,const cv::Scalar& color,const int& radius,const int& thickness):pt(pt),color(color),radius(radius),thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色int radius;       // 半径int thickness;    // 线宽};struct FlaskLine {FlaskLine(const std::pair<cv::Point2f, cv::Point2f>& pt_pair,const cv::Scalar& color,const int& thickness):pt_pair(pt_pair),color(color),thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;int thickness;};// 文本绘制参数struct FlaskText {FlaskText(const std::string& str,const cv::Point2d& pt,const cv::Scalar& color,const double& scale):str(str),pt(pt),color(color),scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色double scale;     // 字体大小};/*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    */class FlaskStream {public:
        FlaskStream& operator<<(const char* str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);void clear();private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;};// 用于复现的瞄准参数// 移植代码的时候将这段代码移植到自瞄那里struct ShootParam {double v0 = 0.;                                                 // 子弹初速度double aim_angle = 0.;                                          // 发射仰角// Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点};// 子弹命中位置信息struct HitPos {bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置            };// 子弹图像投影信息struct HitCircle {bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆};// 匹配代价评估struct CaughtCost {bool caught;    // 是否满足匹配条件double cost;    // 匹配代价(越小越好)};// 子弹弹道物理模拟器class ProjectileSimulator {public:ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {}// 子弹在图像平面上的投影计算auto get_circle_by_t(const double& t) const -> HitCircle;// 计算在指定时间t的子弹位置auto get_pos_by_t(const double& t) const -> HitPos;// 获取开火时间auto get_fire_t() const -> double;private:const double g { 9.8 };const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;};// 子弹位置信息struct IdPos {int id;        Eigen::Vector3d pos;};// 子弹投影圆信息struct IdCircle {   
        int id;        math::CircleF circle; // 子弹在图像平面上的投影圆};// 子弹模拟器封装struct IdProj {int id;        ProjectileSimulator proj;  // 子弹物理模拟器实例};// 自动瞄准误差校准(目前仅用来复现理想弹道)class AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);// 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆auto get_circles(long long now_time) -> std::vector<IdCircle>;auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器          
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;      
        long long next_id = 0;           
        long long last_fire_time = 0;};
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(const Eigen::Vector3d& pos,const cv::Scalar& color,const int& radius,const int& thickness);// 绘制模拟发射的子弹void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                               cv::Mat& img,long long now_time, 
                               long long eTime, 
                               long long COMMAND_TIMESPAN);}#endif // TRAJECTORY_VISUALIZER_HPP
## 2. 修改实现文件 (trajectory_visualizer.cpp)
cpp
#include "trajectory_visualizer.hpp"#include <chrono>namespace tools{auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);// 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();// 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);double radius = math::get_dis(edge_xy_u, center_xy_u);return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };}auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {double k = 1;   // 空气阻力系数// 计算水平位移double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);// 计算高度double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;// 弹道轨迹仅取决于目标点(理想弹道)const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 计算基准方向const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();const Eigen::Vector3d h_norm = { 0., 0., 1. };const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),bullet_xyz_i_barrel(1, 0) };const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),target_xyz_i_barrel(1
Invoking "make -j8 -l8" failed
rm@rm-NUC11PAHi7:~/ws_glut_vison$ catkin_make
Base path: /home/rm/ws_glut_vison
Source space: /home/rm/ws_glut_vison/src
Build space: /home/rm/ws_glut_vison/build
Devel space: /home/rm/ws_glut_vison/devel
Install space: /home/rm/ws_glut_vison/install
Running command: "make cmake_check_build_system" in "/home/rm/ws_glut_vison/build"
Running command: "make -j8 -l8" in "/home/rm/ws_glut_vison/build"
[  0%] Built target std_msgs_generate_messages_py
[  0%] Built target geometry_msgs_generate_messages_py
[  0%] Built target std_msgs_generate_messages_cpp
[  0%] Built target std_msgs_generate_messages_eus
[  0%] Built target std_msgs_generate_messages_lisp
[  0%] Built target geometry_msgs_generate_messages_eus
[  0%] Built target _rm_msgs_generate_messages_check_deps_ArmorArray
[  5%] Built target hikcamera
[  5%] Built target _rm_msgs_generate_messages_check_deps_Armor
[  5%] Built target _rm_msgs_generate_messages_check_deps_RmSerial
[  5%] Built target geometry_msgs_generate_messages_cpp
[  5%] Built target _rm_msgs_generate_messages_check_deps_Debug
[  5%] Built target std_msgs_generate_messages_nodejs
[  5%] Built target geometry_msgs_generate_messages_nodejs
[  5%] Built target geometry_msgs_generate_messages_lisp
[ 17%] Built target rm_msgs_generate_messages_py
[ 27%] Built target rm_msgs_generate_messages_cpp
[ 37%] Built target identify_test
[ 47%] Built target identify
[ 52%] Built target serial
[ 65%] Built target rm_msgs_generate_messages_eus
[ 75%] Built target rm_msgs_generate_messages_nodejs
[ 85%] Built target rm_msgs_generate_messages_lisp
[ 85%] Built target rm_msgs_generate_messages
[ 87%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o
[ 90%] Building CXX object rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:81:10: error: no declaration matches ‘void tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
   81 |     auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
      |          ^~~~~~~~~~~~
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:1:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: note: candidate is: ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:155:11: note: ‘class tools::AimCorrector’ defined here
  155 |     class AimCorrector {
      |           ^~~~~~~~~~~~
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp: In function ‘void tools::draw_simulated_bullets(CoordinateTransformer*, const tools::ShootParam&, cv::Mat&, long long int, long long int, long long int)’:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.cpp:230:68: error: use of ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’ before deduction of ‘auto’
  230 |         aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
      |                                                                    ^
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:132：rm_tracker/CMakeFiles/tracker.dir/include/trajectory_visualizer.cpp.o] 错误 1
make[2]: *** 正在等待未完成的任务....
In file included from /home/rm/ws_glut_vison/src/rm_tracker/include/RmTracker.hpp:34,
                 from /home/rm/ws_glut_vison/src/rm_tracker/src/tracker.cpp:3:
/home/rm/ws_glut_vison/src/rm_tracker/include/trajectory_visualizer.hpp:162:14: error: extra qualification ‘tools::AimCorrector::’ on member ‘update_bullet’ [-fpermissive]
  162 |         auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
      |              ^~~~~~~~~~~~
make[2]: *** [rm_tracker/CMakeFiles/tracker.dir/build.make:76：rm_tracker/CMakeFiles/tracker.dir/src/tracker.cpp.o] 错误 1
make[1]: *** [CMakeFiles/Makefile2:2125：rm_tracker/CMakeFiles/tracker.dir/all] 错误 2
make: *** [Makefile:146：all] 错误 2
Invoking "make -j8 -l8" failed
rm@rm-NUC11PAHi7:~/ws_glut_vison$   我已经修改了上面的代码,这里为什么报错? 不要生成一大堆代码
我们之前修改了头文件，但是在头文件中的类声明里，我们仍然错误地使用了作用域限定符。
错误出现在头文件的第162行：auto AimCorrector::update_bullet(...);
在类定义内部，我们声明成员函数时不应该加上类名限定。应该直接写函数声明。
让我们检查一下头文件中的AimCorrector类的声明部分。根据错误信息，头文件第162行是：
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
这行代码在类AimCorrector的内部，这是错误的。应该改为：
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
另外，我们还需要确保在类声明中，这个函数的声明与实现文件的函数签名一致。
同时，我们注意到在实现文件中，函数定义时也错误地加上了作用域限定符，但是是在类外部，这是正确的。不过，由于我们修改了头文件，实现文件的函数定义应该与头文件中的声明匹配。
但是，错误信息显示实现文件中的函数定义与头文件中的声明不匹配。实际上，我们刚才在头文件中修改了，但是实现文件中的函数定义也需要修改，因为之前我们在实现文件中定义函数时，函数签名可能不一致。
让我们重新整理：
头文件中AimCorrector类的声明：
class AimCorrector {
public:
AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
auto get_circles(long long now_time) -> std::vector<IdCircle>;
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void; // 修改这一行，去掉类名限定
private:
...
};
实现文件中，函数定义应该为：
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
... // 函数体
}
但是，我们注意到错误信息中还有另一个问题：在draw_simulated_bullets函数中，调用update_bullet时，我们传递了三个参数，但是现在update_bullet函数已经修改为三个参数，所以调用方式应该是正确的。
然而，错误信息还提示“use of ‘auto tools::AimCorrector::update_bullet(...)’ before deduction of ‘auto’”。这通常是因为函数声明为auto，但定义时没有正确指定返回类型（或者定义和声明不匹配）。我们已经在声明和定义中都指定了返回类型为void（通过尾返回类型），所以这个错误可能是由于之前的声明错误导致的。
另外，我们还需要检查头文件中的函数声明是否与实现文件中的定义一致。特别是，我们修改了头文件，但实现文件中的定义可能还是旧的。
根据你提供的错误信息，头文件第162行仍然是错误的，说明头文件没有修改成功。请确保你已经将头文件中的那一行修改为：
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
如果头文件已经修改，但错误仍然存在，请检查是否保存了头文件，或者是否在多个地方有重复的声明。
由于我们无法直接修改你的文件，请按照上述步骤修改头文件和实现文件。
如果问题仍然存在，请提供修改后的头文件和实现文件的相关部分，以便进一步分析。
另外，错误信息中还提到在draw_simulated_bullets函数中调用update_bullet时使用了三个参数，而update_bullet函数现在已经接受三个参数，所以这里应该没有问题。但是，注意在draw_simulated_bullets函数中，我们调用的是：
aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
这没有问题。
但是，我们注意到在实现文件中的draw_simulated_bullets函数中，我们调用了aim_corrector.get_circles(now_time,eTime,COMMAND_TIMESPAN)，但是get_circles函数只接受一个参数。这可能是导致另一个错误的原因。请检查：
在头文件中，get_circles函数声明为：
auto get_circles(long long now_time) -> std::vector<IdCircle>;
所以它只能接受一个参数。但是在draw_simulated_bullets函数中，我们传递了三个参数。这显然是错误的。
因此，我们需要修改draw_simulated_bullets函数中调用get_circles的方式，只传递一个参数。
总结需要修改的地方：
头文件中AimCorrector类的update_bullet成员函数声明，去掉类名限定。
确保update_bullet函数在头文件中声明为三个参数，在实现文件中定义也为三个参数。
修改draw_simulated_bullets函数中调用get_circles的方式，只传递一个参数（now_time）。
请按照上述步骤修改，然后重新编译。
如果还有问题，请提供修改后的代码。
错误依然存在，主要问题是头文件中第162行的类成员函数声明格式错误。让我直接指出并修复这个问题：
问题所在
在你的头文件中第162行：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
这个写法是错误的，因为在类声明内部，你不能使用作用域限定符 AimCorrector::。
修复方案
## 1. 修改头文件 (trajectory_visualizer.hpp)
找到第162行，修改为：
cpp
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
完整的类声明应该如下：
cpp
// 自动瞄准误差校准(目前仅用来复现理想弹道)class AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);// 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆auto get_circles(long long now_time) -> std::vector<IdCircle>;// 修正这里：移除"AimCorrector::"auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;private:
    std::list<IdProj> bullets;             // 活跃子弹容器模拟器          
    CoordinateTransformer* coorConverter;  // 坐标变换器
    std::string config_path_;              // 存储配置路径
    ShootParam shoot_param;      
    long long next_id = 0;           
    long long last_fire_time = 0;};
## 2. 更新实现文件 (trajectory_visualizer.cpp)
确保实现文件中的函数定义与头文件声明一致：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() < 10) {  // 限制子弹数量
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)});this->last_fire_time = current_time;}}}
## 3. 检查draw_simulated_bullets函数调用
在实现文件中，确保调用的参数正确：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {
    FlaskStream flask_aim;
    flask_aim.clear();
    AimCorrector aim_corrector = AimCorrector(coorConverter, shoot_param);// 更新子弹序列
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);  // 注意：这里只传一个参数for (auto& bullet: bullets) {
        flask_aim << FlaskPoint(
            bullet.circle.center,{ 0, 0, 255 },
            bullet.circle.r,2);
        flask_aim << FlaskText(
            std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
    flask_aim >> img;}
## 4. 检查Tracker中的调用
在Tracker中调用draw_simulated_bullets时，确保参数正确：
cpp
// 在Tracker的reconstruction_choose_compensation函数中：
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time, 
                             static_cast<long long>(eTime * 1000),  // 转换为毫秒static_cast<long long>(COMMAND_TIMESPAN * 1000));  // 转换为毫秒
快速修复步骤
打开头文件，找到第162行，将：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
改为：
cpp
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
检查draw_simulated_bullets函数声明，确保在头文件中声明为：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN);
清理并重新编译：
bash
rm -rf build devel
catkin_make
主要问题就是头文件中的类成员函数声明格式错误。在类声明内部，只需要写函数名，不能写作用域限定符。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        double k = 1;   // 空气阻力系数
        // 计算水平位移
        double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111111111111111111111111111111");
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 200;  // 发射间隔：200毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)
                    // ProjectileSimulator(coorConverter, this->shoot_param, current_time)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
#include "ros/ros.h"
namespace tools{
    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点
    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };
    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };
    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };
    /*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    
/
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char
 str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };
    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };
    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置
    };
    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };
    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };
    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {}
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t) const -> HitCircle;
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t) const -> HitPos;
        // 获取开火时间
        auto get_fire_t() const -> double;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };
    // 子弹位置信息
    struct IdPos {
        int id;
        Eigen::Vector3d pos;
    };
    // 子弹投影圆信息
    struct IdCircle {
        int id;
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };
    // 子弹模拟器封装
    struct IdProj {
        int id;
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time) -> std::vector<IdCircle>;
        auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;
        long long next_id = 0;
        long long last_fire_time = 0;
    };
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN);
}
#endif // TRAJECTORY_VISUALIZER_HPP
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111111111111111111111111111111");
                ++it;
                continue;
            }
现在这段代码的问题是没有进入这段代码
根据编译错误信息，问题出现在头文件中第162行：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN);
在类声明内部，不能使用作用域限定符。应该直接写函数声明。
但是，从你给出的代码中，我看到头文件中的第162行是：
cpp
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
这看起来是正确的。然而，编译器报错信息却显示它看到了一个带有作用域限定符的版本。
这可能是因为编译器在编译时读取的头文件版本与你展示的版本不一致。请确保你修改并保存了头文件，并且编译器正在使用这个最新版本。
另外，错误信息中还提到：
cpp
error: use of ‘auto tools::AimCorrector::update_bullet(long long int, long long int, long long int)’ before deduction of ‘auto’
这是因为在实现文件中，函数定义使用了auto作为返回类型，但是函数定义在头文件中已经声明了返回类型为void，所以应该将实现文件中的函数定义改为返回void，或者确保返回类型一致。
在实现文件中，我们看到：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void
这里使用了尾置返回类型，并且返回类型为void，这与头文件中的声明一致，所以没有问题。
但是，请注意，在头文件中，函数声明和类定义在同一文件中，而实现文件是分开编译的。如果头文件没有正确更新，编译器可能会使用旧的头文件，导致声明不匹配。
解决方案：
确保头文件已正确修改并保存。检查头文件中第162行，确保它是：
cpp
auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
清理并重新编译。由于可能使用了旧的编译缓存，建议清理构建目录并重新编译。
如果问题仍然存在，检查是否有其他头文件包含了旧版本的头文件，或者头文件被多次包含导致宏重定义等问题。
关于没有进入if (now_time < it->proj.get_fire_t())这段代码，可能是因为条件不满足。也就是说，当前时间now_time已经大于等于子弹的发射时间fire_t。这可能是由于你设置的发射时间延迟不够大，或者当前时间戳now_time增长过快。
在update_bullet函数中，你设置了子弹的发射时间为：
cpp
current_time + eTime + 0.025 + COMMAND_TIMESPAN
注意，这里的eTime和COMMAND_TIMESPAN是long long类型，你传入的是毫秒吗？在Tracker中调用时，你传入的eTime和COMMAND_TIMESPAN是毫秒吗？如果不是，需要转换为毫秒。
另外，在draw_simulated_bullets函数中，你传入的now_time是当前时间，而子弹的发射时间是在当前时间的基础上加上了延迟。因此，当now_time小于发射时间时，子弹不会发射，所以不会进入那个条件分支。
如果你希望看到子弹发射，可以尝试增大延迟，或者确保在子弹发射后再调用get_circles。你也可以在update_bullet函数中打印调试信息，看看是否成功添加了子弹，以及子弹的发射时间是多少。
总结步骤：
修改头文件，确保类声明中的函数没有作用域限定符。
清理并重新编译整个项目。
检查时间单位，确保延迟参数的单位一致（都是毫秒）。
添加调试信息，打印子弹的发射时间和当前时间，以判断条件是否满足。
如果编译问题解决了，但逻辑问题（没有进入if）仍然存在，那么请关注时间参数和延迟设置，确保子弹发射时间在当前时间之后足够长，以便在调用get_circles时，now_time仍然小于发射时间。
根据你提供的代码和问题描述，你已经解决了编译错误，但现在遇到了逻辑问题：程序没有进入 if (now_time < it->proj.get_fire_t()) 这段代码。
问题分析
原因：
子弹的发射时间 fire_t 已经等于或早于当前时间 now_time。
在你的代码中，子弹的发射时间计算为：
cpp
ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 0.025 + COMMAND_TIMESPAN)
但是，这里的 eTime 和 COMMAND_TIMESPAN 是 long long 类型，而它们应该是毫秒值。如果 eTime 和 COMMAND_TIMESPAN 是秒（double类型），那么你需要将它们转换为毫秒：
修复方案
## 1. 确保时间单位一致
所有时间应该使用相同的单位（毫秒）：
cpp
auto AimCorrector::update_bullet(long long current_time, double eTime_sec, double command_timespan_sec) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime_sec * 1000);long long command_timespan_ms = static_cast<long long>(command_timespan_sec * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() < 10) {  // 限制子弹数量// 计算发射时间（添加延迟）long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;
            std::cout << "创建子弹: current_time=" << current_time << ", fire_time=" << fire_time << ", 延迟=" << (eTime_ms + additional_delay + command_timespan_ms) << "ms" << std::endl;
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
## 2. 修改调用方式
在Tracker中调用时，传入秒值：
cpp
// 在Tracker的reconstruction_choose_compensation函数中：
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(
    chrono::system_clock::now()).time_since_epoch().count();// 传入秒值，函数内部会转换为毫秒
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             eTime,  // 秒
                             COMMAND_TIMESPAN);  // 秒
## 3. 添加调试信息
在get_circles函数中添加详细的时间调试信息：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;
    std::cout << "\n=== get_circles ===" << std::endl;
    std::cout << "当前时间(now_time): " << now_time << std::endl;
    std::cout << "子弹数量: " << bullets.size() << std::endl;int bullet_index = 0;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();
        std::cout << "子弹[" << bullet_index << "] - id: " << it->id << ", fire_t: " << fire_t << ", now_time < fire_t? " << (now_time < fire_t) 
                  << ", 差值: " << (fire_t - now_time) << "ms" << std::endl;if (now_time < fire_t) {ROS_WARN("子弹[%d]还未发射，fire_t=%lld, now_time=%lld, 还有%lldms",                     bullet_index, fire_t, now_time, (fire_t - now_time));++it; 
            continue;}
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}++bullet_index;}
    std::cout << "返回有效子弹数量: " << res.size() << std::endl;return res;}
## 4. 修改draw_simulated_bullets函数
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           double eTime_sec, 
                           double command_timespan_sec) {
    std::cout << "\n=== draw_simulated_bullets ===" << std::endl;
    std::cout << "现在时间: " << now_time << std::endl;
    std::cout << "eTime: " << eTime_sec << "s" << std::endl;
    std::cout << "COMMAND_TIMESPAN: " << command_timespan_sec << "s" << std::endl;
    FlaskStream flask_aim;
    flask_aim.clear();
    AimCorrector aim_corrector(coorConverter, shoot_param);// 更新子弹序列
    aim_corrector.update_bullet(now_time, eTime_sec, command_timespan_sec);// 获取子弹
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
    std::cout << "获取到的可绘制子弹数量: " << bullets.size() << std::endl;for (auto& bullet: bullets) {
        flask_aim << FlaskPoint(
            bullet.circle.center,{ 0, 0, 255 },
            bullet.circle.r,2);
        flask_aim << FlaskText(
            std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
    flask_aim >> img;
    std::cout << "=== draw_simulated_bullets结束 ===\n" << std::endl;}
## 5. 检查时间参数值
在Tracker中检查你的时间参数：
cpp
// 在Tracker构造函数中
cout << "eTime: " << eTime << " seconds" << endl;  // 应该是0.002左右
cout << "COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << " seconds" << endl;  // 应该是0.11左右// 在调用draw_simulated_bullets之前
cout << "调用draw_simulated_bullets前: now_time=" << draw_visual_now_time << ", eTime=" << eTime << "s, COMMAND_TIMESPAN=" << COMMAND_TIMESPAN << "s" << endl;
关键检查点
时间单位转换：确保所有时间都是毫秒，或者正确转换
延迟计算：eTime + 0.025 + COMMAND_TIMESPAN 应该至少是125毫秒（如果eTime=2ms, COMMAND_TIMESPAN=110ms）
当前时间：确保draw_visual_now_time是当前时间的毫秒值
最可能的问题是时间单位不匹配。如果 eTime 和 COMMAND_TIMESPAN 是秒（如0.002和0.11），那么你需要将它们转换为毫秒才能在 update_bullet 函数中使用。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        double k = 1;   // 空气阻力系数
        // 计算水平位移
        double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ++it;
                continue;
            }
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 200;  // 发射间隔：200毫秒
        // 将秒转换为毫秒
        long long eTime_ms = static_cast<long long>(eTime * 1000);
        long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
        long long additional_delay = 25;  // 0.025秒 = 25毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime_ms + additional_delay + command_timespan_ms)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
#include "ros/ros.h"
namespace tools{
    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点
    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };
    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };
    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };
    /*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    
/
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char
 str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };
    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };
    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置
    };
    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };
    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };
    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {}
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t) const -> HitCircle;
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t) const -> HitPos;
        // 获取开火时间
        auto get_fire_t() const -> double;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };
    // 子弹位置信息
    struct IdPos {
        int id;
        Eigen::Vector3d pos;
    };
    // 子弹投影圆信息
    struct IdCircle {
        int id;
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };
    // 子弹模拟器封装
    struct IdProj {
        int id;
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time) -> std::vector<IdCircle>;
        auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;
        long long next_id = 0;
        long long last_fire_time = 0;
    };
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN);
}
#endif // TRAJECTORY_VISUALIZER_HPP
现在的问题是程序可以运行了,但是子弹没有打印重投影出来
我们注意到在代码中，子弹的绘制是通过draw_simulated_bullets函数进行的，但是子弹没有显示出来。可能的原因有：
子弹的发射时间计算有误，导致子弹在创建后立即被判断为已发射（即now_time可能大于等于fire_t），从而进入了后续处理，但可能因为弹道计算问题导致绘制的位置不对，或者没有绘制。
子弹的弹道计算有误，导致投影到图像上的位置超出了图像范围，或者半径太小看不见。
绘制子弹的代码没有正确执行，可能因为get_circles返回的子弹列表为空，或者绘制函数没有被正确调用。
我们需要逐步排查。
首先，在get_circles函数中，我们添加一些调试信息，看看子弹是否被正确创建，以及子弹的发射时间和当前时间的关系。
修改get_circles函数，添加打印信息：
cpp
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{// 初始化结果向量
        std::vector<IdCircle> res;// 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表ROS_INFO("Bullets list size: %zu", bullets.size());for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 检查子弹是否已发射// 当前图像时间 < 子弹发射时间// 是 -> 子弹还未发射,跳过// 否 -> 子弹已发射,继续处理ROS_INFO("Bullet id: %d, fire_t: %lld, now_time: %lld", it->id, it->proj.get_fire_t(), now_time);if (now_time < it->proj.get_fire_t()) {ROS_WARN("Bullet not fired yet, skipping.");++it; 
                continue;}// 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {ROS_INFO("Bullet hit, erasing.");
                it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果,迭代器ROS_INFO("Bullet not hit, adding to result.");
                res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
同时，在update_bullet函数中也添加一些调试信息，确保子弹被创建：
cpp
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒ROS_INFO("update_bullet: current_time=%lld, last_fire_time=%lld, interval=%lld", current_time, last_fire_time, fire_interval);if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {ROS_INFO("Condition met, trying to add bullet.");if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;ROS_INFO("Adding bullet with fire_time: %lld (current_time=%lld, delay=%lld)", fire_time, current_time, eTime_ms+additional_delay+command_timespan_ms);
                bullets.push_back(IdProj {
                    next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
另外，在draw_simulated_bullets函数中，添加打印信息，看看获取到的子弹数量：
cpp
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);// 更新子弹序列// 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);ROS_INFO("Number of bullets to draw: %zu", bullets.size());for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,{ 0, 0, 255 },
                bullet.circle.r,2);
            flask_aim << FlaskText(
                std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
        flask_aim >> img;}
运行程序，观察日志输出。根据日志输出，我们可以判断：
子弹是否被创建（update_bullet中是否打印添加子弹的信息）
子弹的发射时间是否正确（fire_time是否大于当前时间now_time）
在get_circles中，子弹是否因为未发射而被跳过，或者是否被判断为击中而删除。
如果子弹被创建，但是get_circles返回的子弹列表为空，那么可能是以下原因：
子弹的发射时间设置得太晚，导致now_time < fire_t始终为真，子弹一直处于未发射状态，被跳过。
子弹的发射时间设置得太早，导致子弹一创建就被判断为已发射，但可能因为弹道计算问题，子弹立即被判断为击中（hit为真）而被删除。
根据你的描述，我们之前的问题是没有进入if (now_time < it->proj.get_fire_t())，现在可能已经进入了，但是子弹没有显示，所以我们需要进一步检查弹道计算和击中判断。
在ProjectileSimulator::get_pos_by_t函数中，我们计算了子弹的位置，并判断是否击中。这里的击中判断是：bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm()，即子弹在枪管坐标系中的水平距离是否大于等于目标点的水平距离。如果是，则认为击中。
但是，请注意，这个判断可能过于严格，因为子弹可能一开始就大于目标距离（比如目标距离为0，或者子弹的初始位置就不对）。此外，弹道计算可能有问题，导致子弹位置计算错误。
我们可以添加调试信息，打印子弹位置和目标位置，以及击中判断的结果。
修改ProjectileSimulator::get_pos_by_t函数：
cpp
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {double k = 1;   // 空气阻力系数// 计算水平位移double w = (t - this->fire_t) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);// 计算高度double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;// 弹道轨迹仅取决于目标点(理想弹道)const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 计算基准方向const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();const Eigen::Vector3d h_norm = { 0., 0., 1. };const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),bullet_xyz_i_barrel(1, 0) };const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),target_xyz_i_barrel(1, 0) };bool hit = bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm();ROS_INFO("ProjectileSimulator: t=%f, fire_t=%lld, bullet_xy_i_barrel.norm()=%f, target_xy_i_barrel.norm()=%f, hit=%d",                 t, fire_t, bullet_xy_i_barrel.norm(), target_xy_i_barrel.norm(), hit);return HitPos { hit, bullet_xyz_i_camera};}
同时，在get_circle_by_t函数中，我们也可以打印投影后的像素坐标，看看是否在图像范围内：
cpp
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->map2Cam(bullet.pos);// 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();// 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型ROS_INFO("ProjectileSimulator: center_xy_u=(%f,%f), radius=%f", center_xy_u.x, center_xy_u.y, radius);return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };}
运行程序，观察日志输出。通过日志，我们可以判断：
子弹在枪管坐标系中的水平距离和目标水平距离，以及击中判断。
子弹投影到图像上的像素坐标和半径。
如果子弹的像素坐标不在图像范围内（比如负数或大于图像宽度/高度），那么绘制可能不会显示。如果半径为0或负数，也不会显示。
另外，请注意，在draw_simulated_bullets函数中，我们创建了一个局部的AimCorrector对象，然后调用update_bullet和get_circles。但是，每次调用draw_simulated_bullets时，都会创建一个新的AimCorrector对象，这意味着之前的状态（子弹列表）不会被保留。也就是说，每一帧都是重新开始模拟子弹，这可能导致子弹还没有发射（因为发射延迟）就被丢弃了（因为局部对象在函数结束时销毁）。
你可能需要将AimCorrector对象作为全局变量或者作为某个对象的成员变量，以便在多次调用draw_simulated_bullets时保持子弹的状态。
根据你的代码，draw_simulated_bullets是在Tracker的每一帧中被调用的，而Tracker是一个持续运行的对象。因此，你应该将AimCorrector作为Tracker的成员变量，而不是在每次调用draw_simulated_bullets时都创建一个新的。
修改Tracker类，添加一个成员变量：
cpp
class Tracker{private:// ... 其他成员变量
    tools::AimCorrector aim_corrector;  // 子弹模拟器};
并在Tracker的构造函数中初始化它（可能需要传递参数）。然后在draw_simulated_bullets函数中，使用这个成员变量，而不是局部变量。
但是，请注意，draw_simulated_bullets函数目前是独立函数，它创建了一个局部的AimCorrector。我们需要修改这个设计。
由于时间关系，我们先尝试在draw_simulated_bullets函数中增加子弹状态的持续性，我们可以将AimCorrector声明为静态变量，这样它的生命周期会持续到程序结束，但这样会导致所有调用共享同一个AimCorrector实例，可能会产生冲突。
更好的方法是，将AimCorrector作为Tracker的成员，然后在Tracker的reconstruction_choose_compensation函数中调用aim_corrector.update_bullet和aim_corrector.get_circles，并将子弹绘制到图像上。
由于修改较大，我们这里先采用静态局部变量的方法，看看子弹是否能够持续存在并显示。修改draw_simulated_bullets函数：
cpp
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){static AimCorrector aim_corrector(coorConverter, shoot_param);  // 静态变量，保持状态
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();// 更新子弹序列// 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);ROS_INFO("Number of bullets to draw: %zu", bullets.size());for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,{ 0, 0, 255 },
                bullet.circle.r,2);
            flask_aim << FlaskText(
                std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
        flask_aim >> img;}
注意：静态局部变量只会在第一次调用时初始化，后续调用都会使用同一个实例。这样，子弹的状态就会在多次调用中保持。
但是，这种方法有一个问题：如果coorConverter指针发生变化，或者shoot_param发生变化，那么静态变量在后续调用中就不会更新这些参数。所以，我们需要在每次调用时更新aim_corrector中的这些参数。我们可以在AimCorrector类中添加设置函数，或者修改update_bullet函数，同时更新这些参数。
考虑到shoot_param可能会变化，我们可以将shoot_param作为update_bullet的参数，而不是在构造函数中传入。但是，这样需要修改AimCorrector的设计。
由于时间有限，我们先假设shoot_param不变，而coorConverter也不变。如果变化，我们需要重新设计。
另一种方法是，将AimCorrector的创建放在Tracker的成员函数中，作为Tracker的成员变量，这样Tracker可以控制其生命周期，并在每次需要时更新参数。
由于修改较大，我们先尝试静态局部变量的方法，看看子弹是否能显示。如果子弹能够显示，说明问题在于之前每次调用都创建新的AimCorrector导致状态丢失。
如果仍然不能显示，请根据日志输出进一步分析。
如果日志显示子弹被创建，但是get_circles返回的子弹列表为空，那么可能是子弹在第一次调用get_circles时就被判断为击中而删除了。我们可以通过日志查看击中判断的情况。
请运行添加了日志的程序，并观察输出。
根据你提供的代码，我发现几个关键问题导致子弹没有被重投影出来。让我逐一分析并提供解决方案：
主要问题分析
## 1. 子弹立即被判定为击中
在 get_pos_by_t 函数中，击中判断逻辑有问题：
cpp
return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(), bullet_xyz_i_camera };
问题：子弹创建后立即判断，水平距离可能已经大于等于目标距离，导致子弹立即被标记为"击中"而被删除。
## 2. 时间单位问题
get_pos_by_t 函数参数 t 是毫秒，但弹道计算中直接使用 (t - this->fire_t) 作为秒来计算，导致时间差错误。
## 3. 弹道公式错误
空气阻力系数 k=1 不合理，而且弹道公式复杂且可能有误。
## 4. 缺少调试信息
无法知道子弹的状态和位置。
修复方案
## 1. 修正弹道模型（简化无空气阻力）
cpp
auto ProjectileSimulator::get_pos_by_t(const double& t_millis) const -> HitPos {// 将毫秒转换为秒double delta_t = (t_millis - this->fire_t) / 1000.0;if (delta_t <= 0) {// 子弹还未发射，返回初始位置（枪口位置）
        Eigen::Vector3d gun_pos = Eigen::Vector3d::Zero(); // 枪口在枪管系中的位置
        Eigen::Vector3d bullet_xyz_i_camera = this->coorConverter->gun2Cam(gun_pos);return HitPos { false, bullet_xyz_i_camera };}// 简化的弹道模型（无空气阻力）double v0 = this->shoot_param.v0;double theta = this->shoot_param.aim_angle;// 水平位移double x = v0 * cos(theta) * delta_t;// 垂直位移（考虑重力）double z = v0 * sin(theta) * delta_t - 0.5 * this->g * delta_t * delta_t;// 获取目标在枪管系中的方向const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 水平方向（目标方向）
    Eigen::Vector2d target_dir_2d(target_xyz_i_barrel.x(), target_xyz_i_barrel.y());if (target_dir_2d.norm() < 1e-6) {
        target_dir_2d = Eigen::Vector2d(1, 0); // 默认向前}
    target_dir_2d.normalize();// 子弹在枪管系中的位置
    Eigen::Vector3d bullet_xyz_i_barrel(
        target_dir_2d.x() * x,
        target_dir_2d.y() * x,
        z);// 转换到相机系
    Eigen::Vector3d bullet_xyz_i_camera = this->coorConverter->gun2Cam(bullet_xyz_i_barrel);// 击中判断：子弹在枪管系中的水平距离是否超过目标距离double bullet_distance = bullet_xyz_i_barrel.head<2>().norm();double target_distance = target_xyz_i_barrel.head<2>().norm();bool hit = (bullet_distance >= target_distance) && (delta_t > 0.1); // 添加最小飞行时间return HitPos { hit, bullet_xyz_i_camera };}
## 2. 添加调试输出
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;ROS_INFO("get_circles: now_time=%lld, bullet_count=%zu", now_time, bullets.size());int index = 0;for (auto it = this->bullets.begin(); it != this->bullets.end();) {double fire_t = it->proj.get_fire_t();ROS_INFO("  Bullet[%d]: id=%d, fire_t=%.0f, now_time=%lld, delta=%.0fms",                 index, it->id, fire_t, now_time, now_time - fire_t);if (now_time < fire_t) {ROS_WARN("    Not fired yet");++it; 
            ++index;continue;}
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);if (hit_circle.hit) {ROS_INFO("    Hit! Removing bullet");
            it = this->bullets.erase(it);} else {ROS_INFO("    Not hit, drawing at (%.1f, %.1f) radius=%.1f",                     hit_circle.circle.center.x, hit_circle.circle.center.y, hit_circle.circle.r);
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}++index;}ROS_INFO("  Returning %zu bullets to draw", res.size());return res;}
## 3. 修正update_bullet中的时间延迟
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒// 计算总延迟long long total_delay = eTime_ms + additional_delay + command_timespan_ms;ROS_INFO("update_bullet: current_time=%lld, last_fire_time=%lld, total_delay=%lldms",             current_time, last_fire_time, total_delay);if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() < 10) {  // 减少数量便于观察long long fire_time = current_time + total_delay;ROS_INFO("  Creating bullet id=%d, fire_time=%lld (current_time=%lld + delay=%lld)",                     next_id, fire_time, current_time, total_delay);
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
## 4. 在draw_simulated_bullets中添加参数检查
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {ROS_INFO("=== draw_simulated_bullets ===");ROS_INFO("now_time: %lld", now_time);ROS_INFO("shoot_param: v0=%.2f, aim_angle=%.4f rad (%.2f deg)",             shoot_param.v0, shoot_param.aim_angle, shoot_param.aim_angle * 180.0 / M_PI);ROS_INFO("target_xyz_i_camera: [%.3f, %.3f, %.3f]",             shoot_param.target_xyz_i_camera.x(),
             shoot_param.target_xyz_i_camera.y(),
             shoot_param.target_xyz_i_camera.z());
    FlaskStream flask_aim;
    flask_aim.clear();
    AimCorrector aim_corrector = AimCorrector(coorConverter, shoot_param);
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);ROS_INFO("Bullets to draw: %zu", bullets.size());for (auto& bullet: bullets) {// 检查坐标是否在图像范围内if (bullet.circle.center.x >= 0 && bullet.circle.center.x < img.cols &&
            bullet.circle.center.y >= 0 && bullet.circle.center.y < img.rows) {
            flask_aim << FlaskPoint(
                bullet.circle.center,{ 0, 0, 255 },
                bullet.circle.r,2);
            flask_aim << FlaskText(
                std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);ROS_INFO("  Drawing bullet %d at (%.1f, %.1f)",                     bullet.id, bullet.circle.center.x, bullet.circle.center.y);} else {ROS_WARN("  Bullet %d out of bounds: (%.1f, %.1f), image size: %dx%d",                     bullet.id, bullet.circle.center.x, bullet.circle.center.y,                     img.cols, img.rows);}}
    flask_aim >> img;ROS_INFO("=== End draw_simulated_bullets ===\n");}
## 5. 确保ProjectileSimulator::get_fire_t返回正确类型
cpp
auto ProjectileSimulator::get_fire_t() const -> double {return static_cast<double>(this->fire_t);  // 确保返回double}
关键测试点
检查时间延迟是否正确：
eTime 应该是秒（如 0.002）
COMMAND_TIMESPAN 应该是秒（如 0.11）
总延迟应该约为 137ms (2+25+110)
检查目标坐标：
shoot_param.target_xyz_i_camera 应该在相机系中有合理的值
如果目标在3米外，x坐标应该在3.0左右
检查子弹参数：
子弹速度 v0 应该合理（如 20-30 m/s）
瞄准角度 aim_angle 应该合理（正角度表示向上）
快速诊断
添加以下代码到Tracker中，检查传入参数：
cpp
// 在调用draw_simulated_bullets之前ROS_INFO("Calling draw_simulated_bullets with:");ROS_INFO("  BulletVector: %.2f m/s", BulletVector);ROS_INFO("  Pitch angle: %.4f rad (%.2f deg)", m_target_pose.pitch, m_target_pose.pitch * 180/M_PI);ROS_INFO("  Target position in map: (%.3f, %.3f, %.3f)",         abs_target_armor.position.x(),         abs_target_armor.position.y(),         abs_target_armor.position.z());
Eigen::Vector3d target_in_cam = coorConverter->map2Cam(abs_target_armor.position);ROS_INFO("  Target in camera: (%.3f, %.3f, %.3f)",         target_in_cam.x(), target_in_cam.y(), target_in_cam.z());
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = target_in_cam;
主要问题很可能是：
子弹立即被判断为击中（修改击中判断逻辑）
弹道计算错误（使用简化弹道模型）
时间单位不匹配（确保毫秒/秒转换正确）
建议先用简化弹道模型，确保基础功能正常后再优化。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        ROS_WARN("22222222222222222222222222222222");
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->cam2Map(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        ROS_WARN("33333333333333333333333333333333333333333333333");
        // 将毫秒转换成毫秒
        double t_s = static_cast<double>(t * 0.001);
        double fire_t_s = static_cast<double>(this->fire_t * 0.001);
        double k = 1;   // 空气阻力系数
        // 计算水平位移
        double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        cout << "w: " << w << endl;
        cout << "h: " << h << endl;
        ROS_WARN("444444444444444444444444444444444444");
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111");
                ++it;
                continue;
            }
            ROS_WARN("66666666666666666666666666666666");
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 200;  // 发射间隔：200毫秒
        // 将秒转换为毫秒
        long long eTime_ms = static_cast<long long>(eTime * 1000);
        long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
        long long additional_delay = 25;  // 0.025秒 = 25毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() < AIM_CORRECTOR_BULLETS_MAX_SZ) {        // 最多显示10颗子弹
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime_ms + additional_delay + command_timespan_ms)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111");
                ++it;
                continue;
            }
发现这段代码执行不到continue的下面,也就是说没有打印66666666666666666666，为什么?
根据你的代码和调试信息，你已经解决了子弹列表为空的问题，但现在发现程序进入了if (now_time < it->proj.get_fire_t())这个条件，然后执行了continue，所以没有打印后面的"6666666666"。这意味着当前时间now_time小于子弹的发射时间fire_t，所以子弹还没有发射，因此跳过了后续处理。
为什么now_time会小于fire_t呢？让我们分析一下：
在update_bullet中，你创建子弹时传入的发射时间（fire_t）为：
cpp
current_time + eTime_ms + additional_delay + command_timespan_ms
这意味着你给子弹设置了一个未来的发射时间。然后，在get_circles中，你用当前时间now_time（注意：这个now_time就是调用draw_simulated_bullets时传入的now_time，与创建子弹时的current_time是同一个值）去和子弹的发射时间比较。
由于发射时间被设置为未来（当前时间加上延迟），所以now_time（即当前时间）小于fire_t（未来时间），因此子弹被判定为未发射，跳过了后续的弹道计算和绘制。
但是，你希望的是：子弹在创建后，经过一段延迟（发射时间）才被发射，而在发射之后，你希望绘制出子弹的弹道。
然而，在draw_simulated_bullets函数中，你创建子弹和检查子弹状态使用的是同一个时间now_time（即当前帧的时间）。这样，在同一帧中，你创建了子弹（设置发射时间为未来），然后立即检查子弹状态，由于子弹的发射时间还在未来，所以它不会被绘制。
你想要的效果可能是：子弹在未来的某个时间点发射，然后随着时间的推移，在后续的帧中绘制出子弹的飞行轨迹。但是，在当前的实现中，draw_simulated_bullets每次被调用时都会创建一个新的AimCorrector对象（局部变量），然后更新子弹（可能创建新的子弹），并立即获取当前帧的子弹状态。由于子弹是同一帧创建的，发射时间在未来，所以不会被绘制。
为了看到子弹的飞行轨迹，你需要让AimCorrector对象持续存在，而不是每次调用draw_simulated_bullets都新建一个。这样，子弹列表才能跨帧保持。然后，随着时间的推移，子弹的发射时间会达到（即now_time会大于等于fire_t），然后子弹就会被绘制。
所以，问题在于你的AimCorrector对象是局部变量，每次调用draw_simulated_bullets都会新建，子弹列表被重置。你应该将AimCorrector对象作为类成员变量，在Tracker中持续存在，这样子弹才能跨帧存在。
但是，从你的代码来看，draw_simulated_bullets函数是静态的，每次调用都会新建AimCorrector，然后更新子弹（创建子弹）并获取子弹状态。这只能显示在同一帧内创建的子弹，而且由于发射时间在未来，所以同一帧内不会绘制。
为了测试，你可以暂时将子弹的发射时间设置为当前时间（即不添加延迟），看看子弹是否会被绘制。或者，你可以修改AimCorrector，让它成为一个持续存在的对象（例如，在Tracker中作为成员变量），这样子弹就可以跨帧存在。
不过，从你的使用场景来看，你可能是想在同一帧内模拟子弹的飞行轨迹（比如，从枪口到目标点的弹道）。如果是这样，那么你不需要延迟发射，而应该让子弹的发射时间就是当前时间，然后计算从当前时间开始，经过一段时间后的子弹位置（比如，每帧计算子弹从发射到当前帧的时间差，然后计算位置）。但是，你的代码中，子弹的发射时间被设置为未来（当前时间+延迟），而计算子弹位置时使用的是当前时间，这会导致子弹还没有发射（因为当前时间小于发射时间），所以你可能需要重新考虑时间逻辑。
根据你的需求，我建议：
如果你想要模拟的是子弹从发射到击中目标的整个过程，那么你应该让子弹的发射时间就是当前时间（即不添加延迟），然后计算从发射到当前时间（可能是未来某个时间）的子弹位置。这样，在子弹发射后，随着时间的推移，子弹位置会不断更新。
如果你想要模拟的是子弹在未来的发射（比如，考虑电控延迟），那么你应该将子弹的发射时间设置为未来，然后在未来的时间点（当now_time大于等于fire_t）才开始计算子弹位置。
但是，在每一帧中，你都需要根据当前的时间now_time来计算子弹的位置。所以，在get_circles函数中，只有当now_time大于等于fire_t时，才计算子弹位置（即子弹已经发射）。否则，子弹还没有发射，不进行绘制。
现在，你的代码中，子弹的发射时间被设置为未来（current_time + 延迟），而draw_simulated_bullets被调用时传入的now_time就是current_time（同一帧的时间），所以now_time小于fire_t，子弹不会发射。
为了看到效果，你可以尝试将发射时间设置为当前时间（即不加延迟）：
cpp
ProjectileSimulator(coorConverter, this->shoot_param, current_time)  // 不加延迟
这样，在同一帧中，now_time等于fire_t，条件now_time < fire_t为假，就会进入弹道计算和绘制。
或者，如果你希望延迟发射，那么你需要让draw_simulated_bullets在未来的某一帧被调用（即传入的now_time大于子弹的发射时间）。但是，由于你的AimCorrector对象是局部变量，每次调用都会新建，子弹列表不会保存到下一帧。所以，你需要将AimCorrector对象持久化。
根据你的需求，我建议修改代码结构，将AimCorrector作为Tracker的成员变量，这样子弹列表可以跨帧存在。然后在每一帧的Tracker::Track()函数中，调用draw_simulated_bullets时，传入当前时间，并更新子弹状态。
但是，从你提供的代码中，我看到draw_simulated_bullets是静态函数，而且没有使用持久化的AimCorrector。所以，如果你只是临时测试弹道计算是否正确，可以先将发射时间设置为当前时间（不加延迟），看看弹道绘制是否正确。
另外，你的弹道计算函数get_pos_by_t中有一些潜在问题：
时间单位：t和fire_t是毫秒，你将其转换为秒，这是正确的。
弹道公式：你使用了空气阻力模型，但是空气阻力系数k=1可能过大，导致弹道异常。你可以先使用无空气阻力的模型测试，或者调整k值。
击中判断：bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm()，这个条件可能在子弹发射后很快就满足（特别是当目标距离很近时），导致子弹很快被标记为击中并从列表中删除。
所以，你可以先简化问题，使用无空气阻力模型，并确保目标距离足够远，以便观察弹道。
综上所述，要解决当前问题，你可以：
修改发射时间，暂时去掉延迟，使用当前时间作为发射时间，以便在同一帧中看到子弹。
将AimCorrector对象持久化，使其跨帧存在，这样子弹可以在未来的帧中被绘制。
根据你的实际情况选择修改方案。如果你只是测试弹道计算，可以选择方案1。如果你需要模拟真实的延迟发射，并希望看到子弹飞行过程，那么需要方案2。
由于你没有提供Tracker中调用draw_simulated_bullets的完整上下文，我假设你是在每一帧中调用这个函数。那么，为了方案2，你需要在Tracker类中添加一个成员变量：
cpp
tools::AimCorrector aim_corrector;
然后在构造函数中初始化它（需要传入coorConverter和shoot_param）。但是，shoot_param是每一帧都可能变化的（因为目标位置和云台角度在变），所以你可能需要在每一帧更新aim_corrector的shoot_param。或者，你可以将aim_corrector作为局部变量，但是使用静态变量来保存子弹列表（不推荐，因为这样会使得子弹列表在多次调用之间共享，可能不是你想要的）。
鉴于你的代码结构，我建议先使用方案1测试弹道计算是否正确，然后再考虑如何将子弹模拟与你的跟踪系统整合。
修改update_bullet函数中创建子弹的部分，去掉延迟：
cpp
bullets.push_back(IdProj {
    next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time)  // 使用当前时间，不加延迟});
这样，子弹的发射时间就是当前时间，那么在get_circles中，now_time（同一帧的时间）等于fire_t，条件now_time < fire_t为假，就会执行后面的弹道计算和绘制。
试试看，这样应该能打印出"66666666666666666666"，并进入弹道计算。
根据你的调试信息，程序进入了 if (now_time < it->proj.get_fire_t()) 这个条件判断，但是没有打印"6666666666"，这说明子弹的发射时间 fire_t 大于当前时间 now_time，所以子弹还没有发射，直接 continue 跳过了。
让我分析一下原因：
问题分析
## 1. 时间延迟过大
在你的代码中，子弹的发射时间被设置为：
cpp
current_time + eTime_ms + additional_delay + command_timespan_ms
假设：
current_time = 当前时间（毫秒）
eTime = 0.002秒 = 2毫秒
additional_delay = 25毫秒
COMMAND_TIMESPAN = 0.11秒 = 110毫秒
总延迟 = 2 + 25 + 110 = 137毫秒
这意味着：
你创建子弹时，设置它的发射时间是 当前时间 + 137毫秒
在创建子弹的同一帧，你立即调用 get_circles(now_time)
由于 now_time 小于 fire_t（差137毫秒），所以子弹被认为是"未发射"状态
## 2. 实时性考虑
这个逻辑在实时系统中是合理的：子弹需要时间才能发射出去（击发延迟、电控延迟等）。
但是，对于弹道可视化，你可能希望看到的是：
子弹已经发射，正在飞行中
或者你想预测未来的弹道轨迹
解决方案
方案1：调整时间逻辑（适用于实时弹道显示）
如果你想显示已经发射的子弹，可以修改时间判断：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 方法1：稍微放宽条件，允许子弹"提前"显示long long fire_t = it->proj.get_fire_t();// 如果子弹将在未来50ms内发射，也显示它（作为"即将发射"的视觉效果）if (now_time < fire_t - 50) {ROS_WARN("子弹未发射: now_time=%lld, fire_t=%lld, 差值=%lldms",                     now_time, fire_t, fire_t - now_time);++it; 
            continue;}ROS_WARN("子弹已发射或即将发射");// 获取子弹在当前时刻的投影圆
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
方案2：修改子弹创建逻辑（测试用）
如果你想立即看到子弹，可以去掉延迟：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 200;  // 发射间隔：200毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() < 10) {// 测试：使用当前时间作为发射时间，立即显示子弹long long fire_time = current_time;  // 去掉延迟
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;ROS_WARN("创建子弹: id=%d, fire_time=%lld (无延迟)", next_id-1, fire_time);}}}
方案3：最佳方案：分阶段处理
这是最合理的方法，区分子弹的"发射前"和"飞行中"状态：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();// 子弹发射前：显示在枪口位置if (now_time < fire_t) {// 可以显示"准备发射"的标记，比如在枪口显示一个小点// 或者跳过不显示ROS_WARN("子弹[%d]准备发射: 还有%lldms", it->id, fire_t - now_time);++it;continue;}// 子弹已发射：计算弹道并显示ROS_WARN("子弹[%d]已发射: 已飞行%lldms", it->id, now_time - fire_t);
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);if (hit_circle.hit) {ROS_WARN("子弹[%d]已击中目标", it->id);
            it = this->bullets.erase(it);} else {
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
方案4：在Tracker中调整调用时机
如果你希望看到预测的弹道，而不是实际发射的子弹，可以在Tracker中使用未来的时间：
cpp
// 在Tracker的reconstruction_choose_compensation函数中：long long draw_visual_now_time = chrono::time_point_cast<chrono::milliseconds>(
    chrono::system_clock::now()).time_since_epoch().count();// 使用未来的时间（比如100ms后）来显示弹道预测long long future_time = draw_visual_now_time + 100;  // 100毫秒后
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             future_time,  // 传入未来时间static_cast<long long>(eTime * 1000),static_cast<long long>(COMMAND_TIMESPAN * 1000));
快速测试方案
为了立即看到效果，最简单的方法是：
修改 update_bullet 函数：
cpp
// 将发射时间设置为当前时间，去掉延迟long long fire_time = current_time;  // 而不是 current_time + 延迟
或者修改 get_circles 函数：
cpp
// 放宽条件，让子弹"提前"显示if (now_time < it->proj.get_fire_t() - 50) {  // 50ms的容差++it;continue;}
根本原因总结
你的代码逻辑是正确的，但不符合你的期望：
实际逻辑：子弹有发射延迟（137ms），所以在创建后的137ms内，子弹处于"未发射"状态
你的期望：子弹创建后立即显示飞行轨迹
选择哪个方案取决于你想要的效果：
实时弹道模拟：显示实际发射的子弹轨迹 → 保持当前逻辑，等待137ms后就能看到子弹
弹道预测显示：显示如果现在发射，子弹会如何飞行 → 使用方案4（传入未来时间）
即时测试：立即看到弹道效果 → 使用方案2（去掉延迟）
建议先使用方案2进行测试，确认弹道计算正确后，再根据实际需求选择合适的方案。
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        ROS_WARN("22222222222222222222222222222222");
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->cam2Map(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        ROS_WARN("33333333333333333333333333333333333333333333333");
        // 将毫秒转换成毫秒
        double t_s = static_cast<double>(t * 0.001);
        double fire_t_s = static_cast<double>(this->fire_t * 0.001);
        double k = 0.1;   // 空气阻力系数
        // 计算水平位移
        double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        cout << "w: " << w << endl;
        cout << "h: " << h << endl;
        ROS_WARN("444444444444444444444444444444444444");
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111");
                ++it;
                continue;
            }
            ROS_WARN("66666666666666666666666666666666");
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 50;  // 发射间隔：50毫秒
        // 将秒转换为毫秒
        long long eTime_ms = static_cast<long long>(eTime * 1000);
        long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
        long long additional_delay = 25;  // 0.025秒 = 25毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime_ms + additional_delay + command_timespan_ms)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
我现在是这么实现的,每50ms往发射队列放一颗子弹,表示视觉将要发射子弹,接着在各种延迟(曝光,开火延迟,自瞄程序运行耗时等影响下) 作为子弹的发射时间，  当当前时间小于子弹发射时间的时候表示子弹已经发射，当当前时间小于子弹发射时间的时候,表示子弹还没有发射,当当前时间超过子弹发射时间的时候表示子弹已经发射,此时开始计算这颗子弹的坐标  
e1766498503009
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.009312748]: 111111111111111111111
now_time1766498503032
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.032824389]: 111111111111111111111
now_time1766498503058
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.058838090]: 111111111111111111111
now_time1766498503082
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.082508863]: 111111111111111111111
now_time1766498503107
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.107763172]: 111111111111111111111
now_time1766498503133
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.133792247]: 111111111111111111111
now_time1766498503159
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.159381971]: 111111111111111111111
now_time1766498503182
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.182102586]: 111111111111111111111
^Cnow_time1766498503207
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.207126988]: 111111111111111111111
now_time1766498503232
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.232881250]: 111111111111111111111
now_time1766498503257
it->proj.get_fire_t(): 1.7665e+12
[ WARN] [1766498503.257718857]: 111111111111111111111
[tracker-5] killing on exit
[identify-4] killing on exit
[serial-3] killing on exit
[hikcamera-2] killing on exit
now_time1766498503282
it->proj.get_fire_t(): 1.7665e+12
now_time1766498503306
it->proj.get_fire_t(): 1.7665e+12
now_time1766498503333
it->proj.get_fire_t(): 1.7665e+12
now_time1766498503357
it->proj.get_fire_t(): 1.7665e+12
now_time1766498503382
it->proj.get_fire_t(): 1.7665e+12
now_time1766498503408
it->proj.get_fire_t(): 1.7665e+12
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor com
这里打印出来的如何打印出来数据而不是科学计数法
我们注意到在日志中，it->proj.get_fire_t() 打印出来是科学计数法（例如1.7665e+12），这不利于我们直观地比较时间。我们需要将其转换为普通的十进制整数格式来打印。
另外，从日志中可以看到，now_time 打印的是普通整数（例如1766498503009），而 it->proj.get_fire_t() 打印的是科学计数法，这是因为 get_fire_t() 返回的是 double 类型，而 now_time 是 long long 类型。
我们可以通过以下两种方式解决：
在打印时，将 double 类型的 fire_t 转换为 long long 类型，然后使用 %lld 格式化输出。
或者，使用 std::fixed 和 std::setprecision(0) 来避免科学计数法。
由于 fire_t 本身是 long long 类型，但在 ProjectileSimulator 中存储为 double，我们可以修改 get_fire_t() 返回 long long 类型，或者保持返回 double 但在打印时进行转换。
根据代码，fire_t 在 ProjectileSimulator 中定义为 const long long fire_t，但是 get_fire_t() 返回 double。我们可以将 get_fire_t() 的返回类型改为 long long，这样打印时就可以直接使用 %lld。
但是，注意到在 get_pos_by_t 函数中，我们使用了 this->fire_t 并转换为秒（除以1000），所以返回 long long 也是合理的。
修改步骤：
在头文件中，将 get_fire_t() 的返回类型改为 long long。
在实现文件中，将 get_fire_t() 的返回语句改为 return this->fire_t;，因为 fire_t 已经是 long long 类型。
同时，在 get_circles 函数中，我们打印时使用 %lld 来打印 fire_t，而 now_time 也是 long long，这样就能正常显示。
但是，我们注意到在 get_circles 函数中，我们使用了 ROS_WARN 打印，而 ROS_WARN 使用的是类似于 printf 的格式，所以我们需要确保格式匹配。
由于 fire_t 是 long long，我们使用 %lld 来打印。
我们修改 ProjectileSimulator 类的 get_fire_t() 函数，返回 long long。
另外，在 get_circles 函数中，我们打印 now_time 和 fire_t 时，使用 %lld。
但是，注意：在 get_circles 函数中，我们调用 it->proj.get_fire_t() 并赋值给 long long fire_t，然后打印。
修改后的 get_circles 函数中，我们将 fire_t 定义为 long long 类型，然后使用 %lld 打印。
同时，我们修改头文件中 get_fire_t() 的声明，返回 long long。
修改如下：
在 trajectory_visualizer.hpp 中，修改 ProjectileSimulator 类的 get_fire_t() 声明：
cpp
auto get_fire_t() const -> long long;
在 trajectory_visualizer.cpp 中，修改 get_fire_t() 的实现：
cpp
auto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}
在 get_circles 函数中，我们打印时使用 %lld，注意 now_time 也是 long long，所以我们可以这样打印：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();if (now_time < fire_t) {ROS_WARN("now_time: %lld, fire_t: %lld, 子弹未发射", now_time, fire_t);++it; 
            continue;}// ... 其他代码}return res;}
这样修改后，打印出来的时间就是整数形式，而不是科学计数法了。
但是，注意：在 get_pos_by_t 函数中，我们将 fire_t 转换为秒时，需要先将 fire_t 转换为 double，然后再除以1000。因为 fire_t 是毫秒，而 t 也是毫秒，我们在计算时转换为秒。
所以，在 get_pos_by_t 函数中，我们这样转换：
cpp double t_s = static_cast<double>(t) * 0.001; double fire_t_s = static_cast<double>(this->fire_t) * 0.001;
这样修改后，应该能正常打印和计算。
另外，我们注意到在 get_circle_by_t 函数中，我们使用 coorConverter->cam2Map 将子弹位置转换到地图坐标系，然后计算半径。但是，我们传入的 bullet.pos 是相机坐标系下的坐标，而 cam2Map 是将相机坐标系转换到地图坐标系，这是正确的。
但是，我们计算半径时，使用的是地图坐标系下的点投影到像素坐标，这依赖于 coorConverter->map2pixel 函数。请确保这个函数能正确工作。
最后，我们修改代码以打印出整数时间，并检查时间逻辑。
由于你的日志显示 now_time 和 fire_t 非常接近，但 now_time 一直小于 fire_t，所以一直打印未发射。这可能是因为你的 fire_t 设置得比当前时间大（即延迟发射），所以你需要等待一段时间（比如137毫秒）后，now_time 才会大于等于 fire_t。
如果你希望立即看到子弹发射，可以调整 update_bullet 中的延迟，比如将延迟设置为0，或者将发射时间设置为当前时间。
但是，根据你的需求，你是想模拟实际延迟，那么就需要等待。你可以通过打印 fire_t - now_time 来查看还有多少毫秒发射。
修改 get_circles 中的打印，以显示时间差：
text

if (now_time < fire_t) {
    ROS_WARN("now_time: %lld, fire_t: %lld, 还有 %lld 毫秒发射", now_time, fire_t, fire_t - now_time);
    ++it; 
    continue;
}
这样，你可以看到每颗子弹还需要多久发射。
根据你的日志，now_time 和 fire_t 都是大约1.7665e+12，也就是1766500000000，而你的 now_time 打印为1766498503009，相差大约1.5秒。注意，你的 fire_t 是科学计数法，而 now_time 是整数，它们可能相差不大，但需要精确比较。
修改后，我们期望打印出整数，这样就能清楚看到时间差。
另外，注意你的 now_time 打印时没有空格，所以看起来像是一个字符串，实际上它是数字。在ROS日志中，我们使用%lld打印，它会正确显示。
现在，我们总结一下修改：
修改 ProjectileSimulator::get_fire_t() 返回 long long。
在 get_circles 中，使用 long long fire_t 接收，并用 %lld 打印。
在 get_pos_by_t 中，将 fire_t 转换为 double 时使用 static_cast<double>(this->fire_t)。
由于修改涉及头文件和实现文件，请确保修改一致。
如果还有问题，请检查 update_bullet 中计算 fire_time 时，是否真的加了延迟。根据你的代码，延迟为 eTime_ms + additional_delay + command_timespan_ms，如果这些值都是正数，那么 fire_time 就会比 current_time 大，所以子弹不会立即发射。
希望这些修改能帮助你调试。
根据日志输出，问题很明显了：it->proj.get_fire_t() 打印出来的是科学计数法（1.7665e+12），而 now_time 打印的是普通十进制（1766498503009）。这两个时间实际上是非常接近的！
让我分析并修复这个问题：
问题分析
从日志可以看到：
now_time: 1766498503009
fire_t: 1.7665e+12 = 1766500000000（科学计数法）
差值 = 1766500000000 - 1766498503009 = 约 1496991 纳秒 ≈ 1.5 毫秒
这意味着：
子弹确实有延迟发射：fire_t 比 now_time 大约 1.5 毫秒
时间单位问题：科学计数法显示导致难以直观比较
解决方案
## 1. 修改 get_fire_t() 返回类型
当前返回的是 double，但存储的是 long long。应该统一类型：
cpp
// 在ProjectileSimulator类中auto get_fire_t() const -> long long;  // 改为返回 long long
cpp
// 实现auto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}
## 2. 修改调试输出格式
在 get_circles 函数中，使用正确的格式打印：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();// 使用整数格式打印，避免科学计数法ROS_WARN("now_time: %lld, fire_t: %lld, 差值: %lldms",                 now_time, fire_t, fire_t - now_time);if (now_time < fire_t) {ROS_WARN("子弹未发射，还需等待 %lld 毫秒", fire_t - now_time);++it; 
            continue;}ROS_WARN("子弹已发射，已飞行 %lld 毫秒", now_time - fire_t);// 获取子弹在当前时刻的投影圆
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果,迭代器
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
## 3. 或者在ROS中使用合适的格式
如果你坚持使用 double 类型，可以这样格式化输出：
cpp
// 方法1：使用 std::fixed 和 std::setprecision#include <iomanip>#include <sstream>
std::stringstream ss;
ss << std::fixed << std::setprecision(0) << fire_t_double;ROS_WARN("fire_t: %s", ss.str().c_str());// 方法2：直接使用 printf 风格的格式化ROS_WARN("fire_t: %.0f", fire_t_double);
## 4. 完整修改方案
这是最简洁的修改方案：
cpp
#include "trajectory_visualizer.hpp"#include <chrono>#include <iomanip>#include <sstream>namespace tools{auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->cam2Map(bullet.pos);// 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();// 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);double radius = math::get_dis(edge_xy_u, center_xy_u);return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };}auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {// 将毫秒转换成秒double t_s = static_cast<double>(t * 0.001);double fire_t_s = static_cast<double>(this->fire_t * 0.001);double k = 0.1;   // 空气阻力系数// 计算水平位移double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);// 计算高度double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;// 弹道轨迹仅取决于目标点(理想弹道)const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 计算基准方向const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();const Eigen::Vector3d h_norm = { 0., 0., 1. };const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;const Eigen::Vector3d bullet_xyz_i_camera = this->coorConverter->gun2Cam(bullet_xyz_i_barrel);const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),bullet_xyz_i_barrel(1, 0) };const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),target_xyz_i_barrel(1, 0) };return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(), bullet_xyz_i_camera };}// 修改这里：返回 long long 而不是 doubleauto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {this->shoot_param = shoot_param;}auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 获取子弹发射时间long long fire_t = it->proj.get_fire_t();// 打印详细的时间信息ROS_WARN("时间比较: now_time=%lld, fire_t=%lld, 差值=%lldms",                     now_time, fire_t, fire_t - now_time);if (now_time < fire_t) {ROS_WARN("子弹未发射，还需等待 %lld 毫秒", fire_t - now_time);++it; 
                continue;}ROS_WARN("子弹已发射！已飞行 %lld 毫秒", now_time - fire_t);// 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {
                it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;ROS_WARN("创建新子弹: id=%lld, current_time=%lld, fire_time=%lld, 延迟=%lldms",                         next_id, current_time, fire_time,                         eTime_ms + additional_delay + command_timespan_ms);
                bullets.push_back(IdProj {
                    next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}// ... 其他函数保持不变 ...}
## 5. 修改头文件中的声明
cpp
// 在trajectory_visualizer.hpp中class ProjectileSimulator {public:// ... 其他函数 ...// 修改这里auto get_fire_t() const -> long long;private:const double g { 9.8 };const long long fire_t;  // 存储为long long
    CoordinateTransformer* coorConverter;
    ShootParam shoot_param;};
关键修改点
统一时间类型：get_fire_t() 返回 long long 而不是 double
清晰的时间比较：使用 %lld 格式打印，避免科学计数法
添加时间差信息：打印子弹还需等待的时间或已飞行的时间
问题根源
根据你的日志，子弹的 fire_t 比 now_time 大约 1.5毫秒。这意味着：
每50毫秒创建一个新子弹
每个子弹有大约137毫秒的发射延迟（eTime + additional_delay + COMMAND_TIMESPAN）
但由于时间精度问题，有时候差值只有1-2毫秒
这就是为什么你一直看到"子弹未发射"：因为子弹确实还没有到发射时间！
测试建议
运行程序，观察日志输出，你会看到：
开始时，子弹需要等待约137毫秒
随着时间的推移，差值逐渐减小
当 now_time >= fire_t 时，子弹开始显示
如果你希望立即看到效果，可以在 get_circles 中暂时放宽条件：
cpp
// 临时测试：如果差值小于5毫秒，也认为子弹已发射if (now_time < fire_t - 5) {  // 原来是 if (now_time < fire_t)ROS_WARN("子弹未发射，还需等待 %lld 毫秒", fire_t - now_time);++it; 
    continue;}
这样你就可以看到子弹在发射前5毫秒就开始显示了。
starting process
process[hikcamera-2]: started with pid [19118]
[ WARN] [1766498740.512420935]: now_time: 1766498740512, fire_t: 4627730092099895296, 差值: 1766498740512ms
[ WARN] [1766498740.512486911]: 111111111111111111111
[ WARN] [1766498740.536194465]: now_time: 1766498740536, fire_t: 4627730092099895296, 差值: 1766498740536ms
[ WARN] [1766498740.536237521]: 111111111111111111111
[ WARN] [1766498740.561442872]: now_time: 1766498740561, fire_t: 4627730092099895296, 差值: 1766498740561ms
[ WARN] [1766498740.561490286]: 111111111111111111111
[ WARN] [1766498740.584018819]: now_time: 1766498740583, fire_t: 4627730092099895296, 差值: 1766498740583ms
[ WARN] [1766498740.584055293]: 111111111111111111111
[ WARN] [1766498740.609506649]: now_time: 1766498740609, fire_t: 4627730092099895296, 差值: 1766498740609ms
[ WARN] [1766498740.609528872]: 111111111111111111111
[ WARN] [1766498740.635323791]: now_time: 1766498740635, fire_t: 4627730092099895296, 差值: 1766498740635ms
[ WARN] [1766498740.635359674]: 111111111111111111111
[ WARN] [1766498740.660263821]: now_time: 1766498740660, fire_t: 4627730092099895296, 差值: 1766498740660ms
[ WARN] [1766498740.660293805]: 111111111111111111111
[ WARN] [1766498740.684476247]: now_time: 1766498740684, fire_t: 4627730092099895296, 差值: 1766498740684ms
[ WARN] [1766498740.684511049]: 111111111111111111111
[ WARN] [1766498740.709618933]: now_time: 1766498740709, fire_t: 4627730092099895296, 差值: 1766498740709ms
[ WARN] [1766498740.709640828]: 111111111111111111111
[ WARN] [1766498740.734034179]: now_time: 1766498740733, fire_t: 4627730092099895296, 差值: 1766498740733ms
[ WARN] [1766498740.734063029]: 111111111111111111111
[ WARN] [1766498740.760499042]: now_time: 1766498740760, fire_t: 4627730092099895296, 差值: 1766498740760ms
[ WARN] [1766498740.760541135]: 111111111111111111111
[ WARN] [1766498740.783394642]: now_time: 1766498740783, fire_t: 4627730092099895296, 差值: 1766498740783ms
[ WARN] [1766498740.783433629]: 111111111111111111111
[hikcamera-2] process has died [pid 19118, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/732c0256-e008-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/732c0256-e008-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [19126]
[ WARN] [1766498740.811048380]: now_time: 1766498740810, fire_t: 4627730092099895296, 差值: 1766498740810ms
[ WARN] [1766498740.811102979]: 111111111111111111111
[ WARN] [1766498740.835822391]: now_time: 1766498740835, fire_t: 4627730092099895296, 差值: 1766498740835ms
[ WARN] [1766498740.835880668]: 111111111111111111111
[ WARN] [1766498740.862840799]: now_time: 1766498740862, fire_t: 4627730092099895296, 差值: 1766498740862ms
[ WARN] [1766498740.862874236]: 111111111111111111111
[ WARN] [1766498740.884836234]: now_time: 1766498740884, fire_t: 4627730092099895296, 差值: 1766498740884ms
[ WARN] [1766498740.884871186]: 111111111111111111111
[ WARN] [1766498740.910968607]: now_time: 1766498740910, fire_t: 4627730092099895296, 差值: 1766498740910ms
[ WARN] [1766498740.911007120]: 111111111111111111111
[ WARN] [1766498740.937332803]: now_time: 1766498740937, fire_t: 4627730092099895296, 差值: 1766498740937ms
[ WARN] [1766498740.937383112]: 111111111111111111111
[ WARN] [1766498740.960806543]: now_time: 1766498740960, fire_t: 4627730092099895296, 差值: 1766498740960ms
[ WARN] [1766498740.960835921]: 111111111111111111111
[ WARN] [1766498740.986319291]: now_time: 1766498740986, fire_t: 4627730092099895296, 差值: 1766498740986ms
[ WARN] [1766498740.986374925]: 111111111111111111111
[ WARN] [1766498741.013358055]: now_time: 1766498741013, fire_t: 4627730092099895296, 差值: 1766498741013ms
[ WARN] [1766498741.013391168]: 111111111111111111111
[ WARN] [1766498741.036542575]: now_time: 1766498741036, fire_t: 4627730092099895296, 差值: 1766498741036ms
[ WARN] [1766498741.036668343]: 111111111111111111111
[ WARN] [1766498741.060504611]: now_time: 1766498741060, fire_t: 4627730092099895296, 差值: 1766498741060ms
[ WARN] [1766498741.060533966]: 111111111111111111111
[ WARN] [1766498741.086116337]: now_time: 1766498741086, fire_t: 4627730092099895296, 差值: 1766498741086ms
[ WARN] [1766498741.086147190]: 111111111111111111111
[hikcamera-2] process has died [pid 19126, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/732c0256-e008-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/732c0256-e008-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [19135]
[ WARN] [1766498741.112363029]: now_time: 1766498741112, fire_t: 4627730092099895296, 差值: 1766498741112ms
[ WARN] [1766498741.112423522]: 111111111111111111111
[ WARN] [1766498741.139008850]: now_time: 1766498741138, fire_t: 4627730092099895296, 差值: 1766498741138ms
[ WARN] [1766498741.139050461]: 111111111111111111111
[ WARN] [1766498741.164081259]: now_time: 1766498741164, fire_t: 4627730092099895296, 差值: 1766498741164ms
[ WARN] [1766498741.164117424]: 111111111111111111111
^C[ WARN] [1766498741.187219952]: now_time: 1766498741187, fire_t: 4627730092099895296, 差值: 1766498741187ms
[ WARN] [1766498741.187260017]: 111111111111111111111
[identify-4] killing on exit
[tracker-5] killing on exit
这是什么情况?
问题分析：
从日志中可以看到，fire_t 的值异常巨大，达到了 4627730092099895296，而 now_time 是正常的时间戳（大约1.76e12）。这导致差值计算错误（实际上是负数，但因为是无符号长整型，所以显示为正数）。
这个问题的根本原因是：在构造ProjectileSimulator对象时，传入的fire_t参数类型不匹配。
在update_bullet函数中，我们计算了fire_time（long long类型），然后传递给ProjectileSimulator构造函数。但是，在构造函数中，我们将其存储为const long long fire_t。然而，在get_fire_t函数中，我们返回的是double类型（或者现在改成了long long？），但实际存储的是long long。
但是，从日志中看，fire_t的值异常大，这很可能是由于类型转换错误或者内存错误导致的。
检查代码：
在update_bullet函数中，我们计算fire_time：
cpp
long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;
然后，我们将其传递给ProjectileSimulator构造函数。在构造函数中，我们使用成员初始化列表来初始化fire_t。
但是，注意：在ProjectileSimulator类中，fire_t是const long long类型。这应该没问题。
然而，在get_fire_t函数中，我们返回this->fire_t。如果我们在头文件中将get_fire_t的返回类型声明为double，但在实现中返回long long，就会导致类型不匹配，从而产生未定义行为。
但是，根据之前的修改，我们已经将get_fire_t的返回类型改为long long。那么，为什么日志中打印的fire_t值如此巨大呢？
让我们检查一下打印的代码：
cpp
ROS_WARN("时间比较: now_time=%lld, fire_t=%lld, 差值=%lldms", now_time, fire_t, fire_t - now_time);
这里使用了%lld来打印，说明fire_t是long long类型。但是，打印出来的fire_t值是4627730092099895296，这个值大约为4.6e18，而now_time大约是1.76e12。这个差值大约是4.6e18，这显然不对。
可能的原因：
在ProjectileSimulator构造函数中，传入的fire_t参数已经是错误的。
内存错误，导致fire_t成员变量被错误地写入。
让我们检查ProjectileSimulator构造函数的调用：
cpp
bullets.push_back(IdProj {
    next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});
这里，fire_time是long long类型，然后传递给ProjectileSimulator的构造函数。在构造函数中，我们使用成员初始化列表来初始化fire_t。
但是，注意：在ProjectileSimulator类中，fire_t是const long long类型。这意味着它必须在构造函数初始化列表中初始化，并且一旦初始化就不能改变。
在构造函数的实现中，我们是这样写的：
cpp
ProjectileSimulator(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, const long long& fire_t) 
    : coorConverter{coorConverter}, shoot_param{shoot_param}, fire_t{fire_t} {}
这里，fire_t是参数，同时也是成员变量。注意：参数名和成员变量名相同，这会导致在初始化列表中，fire_t{fire_t}中的两个fire_t分别指代什么？在C++中，初始化列表中的fire_t是成员变量，而括号中的fire_t是参数。这应该没问题。
但是，如果我们没有在初始化列表中初始化fire_t，那么fire_t会被默认初始化，然后我们在构造函数体内对其赋值（因为它是const，所以不能赋值）。但是，我们的代码使用了初始化列表，所以应该没有问题。
另一种可能性：在ProjectileSimulator类的定义中，fire_t成员变量的类型可能是double，而不是long long。检查头文件：
在头文件中，我们定义ProjectileSimulator类：
cpp
class ProjectileSimulator {public:ProjectileSimulator(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, const long long& fire_t);// ...auto get_fire_t() const -> long long;private:const double g { 9.8 };const long long fire_t;  // 注意：这里是long long
    CoordinateTransformer* coorConverter;
    ShootParam shoot_param;};
这里，fire_t是long long类型。但是，在构造函数中，我们传入的参数是const long long& fire_t，所以类型匹配。
那么，为什么fire_t的值会如此巨大呢？可能是由于在update_bullet中计算fire_time时发生了整数溢出？让我们计算一下：
current_time大约是1.76e12（毫秒），加上延迟（假设eTime_ms、additional_delay、command_timespan_ms都是毫秒），总延迟大约137毫秒，所以fire_time应该是current_time + 137，大约是1.76e12，而不是4.6e18。
所以，整数溢出的可能性不大，因为1.76e12远小于long long的最大值（9.22e18）。
另一种可能性：在get_fire_t函数中，我们返回了错误的值。但是，我们只是返回this->fire_t，所以应该没问题。
那么，可能是ROS打印时格式错误？我们使用%lld打印long long，而now_time也是long long，打印出来正常，所以格式应该没问题。
让我们检查一下get_fire_t的实现：
cpp
auto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}
这个函数返回long long，而我们在打印时也使用%lld，所以匹配。
但是，日志中fire_t的值是4627730092099895296，这个值对应的十六进制是0x4038000000000000，这恰好是一个double类型的数据，表示的是12.0（因为0x4038000000000000是double的12.0的IEEE754表示）。这提示我们，可能在某些地方错误地将fire_t解释为double类型了。
检查头文件中get_fire_t的声明：
cpp
auto get_fire_t() const -> long long;
在实现中，我们返回的是long long，但是，如果头文件中声明为返回double，而实现中返回long long，那么就会导致问题。但是，我们已经修改了头文件，声明为返回long long。
不过，注意：我们修改了头文件，但是编译过程中可能没有重新编译所有依赖的文件。因此，可能存在编译不一致的情况。
解决方案：
清理并重新编译整个项目。
在get_fire_t函数中添加调试输出，直接打印fire_t的值，确保在函数内部值是正确的。
让我们修改get_fire_t函数，在返回之前打印一下：
cpp
auto ProjectileSimulator::get_fire_t() const -> long long {ROS_WARN("get_fire_t: fire_t = %lld", this->fire_t);return this->fire_t;}
然后重新运行，看看打印出来的值。
但是，从日志中已经可以看到，在get_circles函数中，我们打印的fire_t值异常，所以问题可能不在get_fire_t函数，而是fire_t成员变量本身在构造时就被赋予了错误的值。
那么，我们在构造函数中也添加打印：
cpp
ProjectileSimulator::ProjectileSimulator(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, const long long& fire_t) 
    : coorConverter{coorConverter}, shoot_param{shoot_param}, fire_t{fire_t} {ROS_WARN("ProjectileSimulator构造函数: fire_t = %lld", fire_t);}
这样，我们可以知道在构造时传入的fire_t参数是否正确。
另外，检查update_bullet函数中计算fire_time的代码：
cpp
long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;ROS_WARN("创建新子弹: id=%lld, current_time=%lld, fire_time=%lld, 延迟=%lldms",         next_id, current_time, fire_time,         eTime_ms + additional_delay + command_timespan_ms);
我们已经添加了打印，但是日志中没有看到这些打印信息。这说明update_bullet函数可能没有被调用，或者没有进入创建子弹的条件分支。
但是，我们确实看到了子弹列表中有子弹（因为get_circles被调用并打印了子弹信息），所以update_bullet一定被调用了，而且创建了子弹。
然而，我们看到的日志中并没有update_bullet中的打印信息，这说明我们可能在update_bullet函数中没有添加打印，或者打印的日志级别不同。让我们检查一下，我们在update_bullet中添加的打印是ROS_WARN，应该会输出。
但是，在提供的日志中，我们只看到了get_circles中的打印，没有update_bullet中的打印。所以，可能是update_bullet没有被调用，或者子弹是在其他地方创建的。
实际上，我们是在draw_simulated_bullets函数中创建AimCorrector对象，然后调用update_bullet。每次调用draw_simulated_bullets都会创建一个新的AimCorrector对象，这意味着子弹列表是空的，然后调用update_bullet可能会创建一颗子弹。但是，由于last_fire_time初始化为0，所以第一次调用update_bullet时，条件this->last_fire_time == 0为真，会创建一颗子弹。
但是，我们每次调用draw_simulated_bullets都会创建一个新的AimCorrector对象，所以每次都是第一次调用update_bullet，都会创建子弹。然后，在同一个函数中，我们调用get_circles来获取子弹并绘制。
但是，由于AimCorrector对象是局部的，每次调用draw_simulated_bullets都会创建新的对象，所以不会积累子弹。这可能是设计上的问题，但这不是当前问题的关键。
关键问题是，为什么fire_t的值异常巨大？
让我们在draw_simulated_bullets函数中添加打印，看看传入的参数：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){ROS_WARN("draw_simulated_bullets: now_time=%lld, eTime=%lld, COMMAND_TIMESPAN=%lld", now_time, eTime, COMMAND_TIMESPAN);// ... 其余代码 ...}
这样，我们可以知道传入的时间参数是否正确。
另外，在ProjectileSimulator构造函数中，我们打印传入的fire_t参数，以及成员变量fire_t。
但是，由于fire_t成员变量是const，我们只能在初始化列表中初始化，所以构造函数体内的打印只能打印参数，不能打印成员变量（因为成员变量在初始化列表之后才可用，但打印参数是可以的）。
所以，我们在构造函数体内打印参数fire_t，在get_fire_t函数中打印成员变量this->fire_t。
综合以上，我们进行以下修改：
在draw_simulated_bullets函数中，打印传入的时间参数。
在update_bullet函数中，打印计算出的fire_time。
在ProjectileSimulator构造函数中，打印传入的fire_t参数。
在get_fire_t函数中，打印成员变量this->fire_t。
然后重新编译运行，观察日志。
由于日志中已经显示了now_time是正常的，而fire_t异常，所以我们重点关注fire_t是如何被传递和存储的。
另外，注意：在ProjectileSimulator类中，fire_t是const long long类型，这确保了它不会被修改，所以只要构造函数中传入的值正确，那么get_fire_t返回的值就应该正确。
但是，从日志中看，fire_t的值是4.6e18，这可能是由于未初始化或内存错误导致的。所以，我们怀疑在构造函数中，传入的fire_t参数就已经是错误的。
那么，让我们检查update_bullet函数中计算fire_time的表达式：
cpp
long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;
这里，eTime_ms和command_timespan_ms是由eTime和COMMAND_TIMESPAN乘以1000得到的。而eTime和COMMAND_TIMESPAN是long long类型，但是它们原本是秒，乘以1000后转换成毫秒。
注意：eTime和COMMAND_TIMESPAN是long long类型，但是它们可能很小（例如0.002秒和0.11秒），乘以1000后是2和110，所以不会溢出。
但是，如果eTime和COMMAND_TIMESPAN是浮点数，那么这里就会有问题。因为函数参数是long long，而在Tracker中调用draw_simulated_bullets时，传入的是eTime和COMMAND_TIMESPAN，这两个变量在Tracker中是double类型。然后，在draw_simulated_bullets函数中，它们被接收为long long，这会导致类型转换，将浮点数转换为整数。
在Tracker中，我们这样调用：
cpp
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time, 
                             static_cast<long long>(eTime * 1000),  // 注意：这里eTime是double，乘以1000后是double，然后转换为long longstatic_cast<long long>(COMMAND_TIMESPAN * 1000));
在draw_simulated_bullets函数中，参数是long long eTime和long long COMMAND_TIMESPAN，但实际上我们已经传入了乘以1000后的值，所以函数内部不需要再乘以1000。
但是，在update_bullet函数中，我们又进行了一次乘以1000：
cpp
long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
这会导致eTime和COMMAND_TIMESPAN被错误地放大了1000倍，因为实际上它们已经是毫秒了。所以，这里应该直接使用，而不是再乘以1000。
在Tracker中，我们已经将秒转换为毫秒，然后传入draw_simulated_bullets，所以eTime和COMMAND_TIMESPAN已经是毫秒。因此，在update_bullet中，我们不应该再乘以1000。
修改update_bullet函数：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 注意：eTime和COMMAND_TIMESPAN已经是毫秒，所以不需要再乘以1000long long eTime_ms = eTime;long long command_timespan_ms = COMMAND_TIMESPAN;long long additional_delay = 25;  // 0.025秒 = 25毫秒// ... 其余代码不变 ...}
但是，为了清晰起见，我们可以重命名参数，或者修改调用处。
实际上，在Tracker中，我们传入的是eTime和COMMAND_TIMESPAN乘以1000后的值，所以它们已经是毫秒。因此，在update_bullet中，我们直接使用即可。
所以，将update_bullet函数中的转换去掉，直接使用参数。
另外，检查在Tracker中调用draw_simulated_bullets时，传入的eTime和COMMAND_TIMESPAN是否确实是乘以1000后的值。从之前的代码中，我们看到：
cpp
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_, draw_visual_now_time, 
                             static_cast<long long>(eTime * 1000), 
                             static_cast<long long>(COMMAND_TIMESPAN * 1000));
所以，传入的是毫秒，因此draw_simulated_bullets函数中的参数eTime和COMMAND_TIMESPAN已经是毫秒，那么在update_bullet中就不应该再乘以1000。
因此，修改update_bullet函数：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime_ms, long long command_timespan_ms) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒long long additional_delay = 25;  // 0.025秒 = 25毫秒if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {long long fire_time = current_time + eTime_ms + additional_delay + command_timespan_ms;ROS_WARN("创建新子弹: id=%lld, current_time=%lld, fire_time=%lld, 延迟=%lldms",                     next_id, current_time, fire_time,                     eTime_ms + additional_delay + command_timespan_ms);
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
同时，修改函数声明，将参数名改为eTime_ms和command_timespan_ms，以表明它们已经是毫秒。
然后，重新编译运行，观察日志。
如果问题仍然存在，那么我们需要在构造函数中打印传入的fire_t，在get_fire_t中打印成员变量，以确定问题出在哪里。
但是，从日志中看，fire_t的值是4.6e18，这个值很可能是由于未初始化或内存错误导致的。所以，我们也要检查是否在某个地方有内存越界，覆盖了fire_t的值。
另外，注意：在IdProj结构中，我们存储了ProjectileSimulator对象，而ProjectileSimulator有const成员，这可能会导致复制问题。但是，我们使用push_back将IdProj对象放入bullets列表，这会调用复制构造函数。如果ProjectileSimulator的复制构造函数没有正确复制fire_t，那么就会有问题。
但是，ProjectileSimulator有一个自定义的构造函数，编译器会自动生成复制构造函数，因为所有成员都可以复制。fire_t是const long long，所以复制构造函数会复制它的值。所以，这应该没问题。
但是，为了安全起见，我们可以为ProjectileSimulator实现一个复制构造函数，确保复制正确。
不过，根据C++规则，const成员和引用成员会导致编译器删除默认的复制赋值运算符，但复制构造函数仍然会被生成（除非有其他原因导致不生成）。但是，由于fire_t是const，复制构造函数会复制其值。
所以，我们先按照上述修改，重新编译运行，看看问题是否解决。
如果问题仍然存在，我们可以在ProjectileSimulator类中添加复制构造函数，并打印信息，以确保复制过程中值被正确传递。
但是，从日志中看，子弹列表中有子弹，所以ProjectileSimulator对象被创建了。问题可能在于fire_t的值在构造时就已经是错误的。
因此，我们首先在构造函数中打印传入的fire_t，在get_fire_t中打印成员变量，以确定问题出在哪个环节。
修改ProjectileSimulator构造函数和get_fire_t函数：
cpp
ProjectileSimulator::ProjectileSimulator(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, const long long& fire_t) 
    : coorConverter{coorConverter}, shoot_param{shoot_param}, fire_t{fire_t} {ROS_WARN("ProjectileSimulator构造函数: 传入的fire_t = %lld, 成员fire_t = %lld", fire_t, this->fire_t);}auto ProjectileSimulator::get_fire_t() const -> long long {ROS_WARN("get_fire_t: 返回fire_t = %lld", this->fire_t);return this->fire_t;}
然后重新编译运行，观察日志。
注意：由于每次调用get_circles都会遍历子弹，所以会多次调用get_fire_t，可能会打印很多日志。我们可以暂时减少日志，或者只打印一次。
但是，为了调试，我们可以先这样，找到问题后再删除。
另外，在update_bullet中，我们创建子弹时，打印fire_time，然后在构造函数中打印，这样就可以看到从计算到传递的过程。
让我们总结一下修改步骤：
修改update_bullet函数，去掉不必要的乘以1000，直接使用毫秒参数。
在update_bullet中，打印计算出的fire_time。
在ProjectileSimulator构造函数中，打印传入的fire_t和成员变量this->fire_t。
在get_fire_t函数中，打印返回的this->fire_t。
然后重新编译运行，观察日志。
如果问题仍然存在，那么可能是其他问题，比如内存错误。但是，从日志中看，now_time是正常的，所以时间获取函数应该没问题。
另外，检查fire_t在ProjectileSimulator类中是否被正确初始化。如果构造函数初始化列表中的顺序与声明顺序不一致，也可能导致问题。但是，我们的初始化列表顺序与声明顺序一致：先coorConverter，然后shoot_param，然后fire_t。而声明顺序是：g、fire_t、coorConverter、shoot_param。注意，g是const double，它先被初始化，然后fire_t，然后coorConverter，然后shoot_param。在初始化列表中，我们初始化了coorConverter、shoot_param和fire_t，但是g在fire_t之前被初始化。这应该没问题。
但是，为了排除初始化顺序的问题，我们可以将fire_t的声明放在g之后，其他成员之前，这样在初始化列表中，我们可以先初始化fire_t，然后其他成员。但是，由于fire_t是const，它必须在初始化列表中初始化，而且初始化顺序与声明顺序一致，所以只要声明顺序正确，初始化列表中的顺序不影响。
在我们的类声明中，fire_t在g之后，在coorConverter和shoot_param之前，所以初始化列表中，fire_t应该在coorConverter和shoot_param之前初始化。但是，我们的初始化列表顺序是coorConverter、shoot_param、fire_t，这会导致coorConverter和shoot_param在fire_t之前初始化，但是它们的声明在fire_t之后，所以这违反了初始化顺序。虽然编译器可能会警告，但不会报错，并且实际初始化的顺序是按照声明顺序，而不是初始化列表的顺序。所以，实际上fire_t会在coorConverter和shoot_param之前初始化，因为它的声明在它们之前。
因此，初始化顺序不会导致fire_t的值错误。
所以，我们按照上述修改，重新编译运行，看看日志输出。
注意：由于日志中已经显示fire_t异常，所以我们重点关注构造函数中打印的传入的fire_t是否异常。
如果构造函数中传入的fire_t就已经是异常值，那么问题在update_bullet中的计算。如果构造函数中传入的fire_t正常，但成员变量this->fire_t异常，那么问题在初始化过程。如果构造函数中打印的成员变量正常，但get_fire_t中打印的异常，那么问题在复制或其他地方。
由于我们每次调用draw_simulated_bullets都会创建新的AimCorrector对象，所以不会涉及复制问题，因为子弹列表是新的，子弹也是新创建的。
所以，我们逐步调试，看问题出在哪里。
最后，如果问题仍然无法解决，我们可以考虑将fire_t改为非const，并在构造函数中赋值，以排除const成员可能带来的问题。
但是，根据日志，问题很可能是在update_bullet中计算fire_time时，由于参数类型转换错误，导致eTime_ms和command_timespan_ms计算错误，从而使得fire_time计算错误。
所以，我们首先修改update_bullet函数，确保传入的eTime和COMMAND_TIMESPAN是毫秒，并且不再进行乘以1000的转换。
修改后，重新编译运行，观察日志。
如果问题解决，那么我们就找到了原因。如果问题仍然存在，那么我们需要进一步调试。
由于问题可能比较复杂，我们可以先简化问题，去掉空气阻力，使用简单的弹道模型，并确保时间参数正确。
另外，检查在Tracker中，eTime和COMMAND_TIMESPAN的值是多少，确保它们是正确的秒数，然后乘以1000转换为毫秒。
在Tracker中，eTime和COMMAND_TIMESPAN是double类型，我们乘以1000后转换为long long，然后传入draw_simulated_bullets。
所以，在draw_simulated_bullets中，参数eTime和COMMAND_TIMESPAN就是毫秒，因此update_bullet中不应该再乘以1000。
所以，修改update_bullet函数，去掉乘以1000的操作，直接使用参数。
让我们修改代码并重新编译运行。
问题确实很严重！从日志可以看到，fire_t 的值变成了一个巨大的数字 4627730092099895296（约 4.6e18），这完全不合理。这明显是一个内存错误或类型转换错误。
问题分析
## 1. 时间值异常
now_time: 1766498740512（约 1.76e12）这是正常的时间戳（毫秒）
fire_t: 4627730092099895296（约 4.6e18）这是明显错误的值
这个巨大的 fire_t 值看起来像是：
未初始化的内存
类型转换错误（如 double 转 long long 时的 bit 位错误）
构造函数初始化问题
[ WARN] [1766498932.050833846]: now_time: 1766498932050, fire_t: 4627730092099895296, 差值: 1766498932050ms
[ WARN] [1766498932.050845193]: 111111111111111111111
[ WARN] [1766498932.076612934]: 构造函数: 1766498932101
[ WARN] [1766498932.076653073]: now_time: 1766498932076, fire_t: 4627730092099895296, 差值: 1766498932076ms
[ WARN] [1766498932.076660062]: 111111111111111111111
[ WARN] [1766498932.100832963]: 构造函数: 1766498932125
[ WARN] [1766498932.100876434]: now_time: 1766498932100, fire_t: 4627730092099895296, 差值: 1766498932100ms
[ WARN] [1766498932.100887201]: 111111111111111111111
[ WARN] [1766498932.125072994]: 构造函数: 1766498932150
[ WARN] [1766498932.125109539]: now_time: 1766498932125, fire_t: 4627730092099895296, 差值: 1766498932125ms
[ WARN] [1766498932.125119513]: 111111111111111111111
[hikcamera-2] process has died [pid 19796, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [19804]
[ WARN] [1766498932.148736428]: 构造函数: 1766498932173
[ WARN] [1766498932.148775653]: now_time: 1766498932148, fire_t: 4627730092099895296, 差值: 1766498932148ms
[ WARN] [1766498932.148785387]: 111111111111111111111
[ WARN] [1766498932.176111516]: 构造函数: 1766498932201
[ WARN] [1766498932.176142995]: now_time: 1766498932176, fire_t: 4627730092099895296, 差值: 1766498932176ms
[ WARN] [1766498932.176152548]: 111111111111111111111
[ WARN] [1766498932.199018796]: 构造函数: 1766498932223
[ WARN] [1766498932.199053724]: now_time: 1766498932198, fire_t: 4627730092099895296, 差值: 1766498932198ms
[ WARN] [1766498932.199063767]: 111111111111111111111
[ WARN] [1766498932.224095546]: 构造函数: 1766498932249
[ WARN] [1766498932.224133666]: now_time: 1766498932224, fire_t: 4627730092099895296, 差值: 1766498932224ms
[ WARN] [1766498932.224144365]: 111111111111111111111
[ WARN] [1766498932.250528742]: 构造函数: 1766498932275
[ WARN] [1766498932.250567302]: now_time: 1766498932250, fire_t: 4627730092099895296, 差值: 1766498932250ms
[ WARN] [1766498932.250579169]: 111111111111111111111
[ WARN] [1766498932.273693262]: 构造函数: 1766498932298
[ WARN] [1766498932.273741385]: now_time: 1766498932273, fire_t: 4627730092099895296, 差值: 1766498932273ms
[ WARN] [1766498932.273753304]: 111111111111111111111
[ WARN] [1766498932.303325250]: 构造函数: 1766498932328
[ WARN] [1766498932.303367334]: now_time: 1766498932303, fire_t: 4627730092099895296, 差值: 1766498932303ms
[ WARN] [1766498932.303378704]: 111111111111111111111
[ WARN] [1766498932.326920889]: 构造函数: 1766498932351
[ WARN] [1766498932.326956853]: now_time: 1766498932326, fire_t: 4627730092099895296, 差值: 1766498932326ms
[ WARN] [1766498932.326965930]: 111111111111111111111
[ WARN] [1766498932.351962080]: 构造函数: 1766498932376
[ WARN] [1766498932.352003250]: now_time: 1766498932351, fire_t: 4627730092099895296, 差值: 1766498932351ms
[ WARN] [1766498932.352014403]: 111111111111111111111
[ WARN] [1766498932.377069650]: 构造函数: 1766498932401
[ WARN] [1766498932.377115896]: now_time: 1766498932376, fire_t: 4627730092099895296, 差值: 1766498932376ms
[ WARN] [1766498932.377125047]: 111111111111111111111
[ WARN] [1766498932.401698379]: 构造函数: 1766498932426
[ WARN] [1766498932.401728560]: now_time: 1766498932401, fire_t: 4627730092099895296, 差值: 1766498932401ms
[ WARN] [1766498932.401737200]: 111111111111111111111
[ WARN] [1766498932.428047482]: 构造函数: 1766498932453
[ WARN] [1766498932.428077758]: now_time: 1766498932428, fire_t: 4627730092099895296, 差值: 1766498932428ms
[ WARN] [1766498932.428089862]: 111111111111111111111
[hikcamera-2] process has died [pid 19804, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [19813]
[ WARN] [1766498932.451674896]: 构造函数: 1766498932476
[ WARN] [1766498932.451705229]: now_time: 1766498932451, fire_t: 4627730092099895296, 差值: 1766498932451ms
[ WARN] [1766498932.451715403]: 111111111111111111111
[ WARN] [1766498932.476933569]: 构造函数: 1766498932501
[ WARN] [1766498932.476987175]: now_time: 1766498932476, fire_t: 4627730092099895296, 差值: 1766498932476ms
[ WARN] [1766498932.476999295]: 111111111111111111111
[ WARN] [1766498932.501469224]: 构造函数: 1766498932526
[ WARN] [1766498932.501505862]: now_time: 1766498932501, fire_t: 4627730092099895296, 差值: 1766498932501ms
[ WARN] [1766498932.501514848]: 111111111111111111111
[ WARN] [1766498932.527582532]: 构造函数: 1766498932552
[ WARN] [1766498932.527624555]: now_time: 1766498932527, fire_t: 4627730092099895296, 差值: 1766498932527ms
[ WARN] [1766498932.527634859]: 111111111111111111111
[ WARN] [1766498932.552224344]: 构造函数: 1766498932577
[ WARN] [1766498932.552269646]: now_time: 1766498932552, fire_t: 4627730092099895296, 差值: 1766498932552ms
[ WARN] [1766498932.552280347]: 111111111111111111111
[ WARN] [1766498932.577441673]: 构造函数: 1766498932602
[ WARN] [1766498932.577478296]: now_time: 1766498932577, fire_t: 4627730092099895296, 差值: 1766498932577ms
[ WARN] [1766498932.577486076]: 111111111111111111111
[ WARN] [1766498932.602829840]: 构造函数: 1766498932627
[ WARN] [1766498932.602860160]: now_time: 1766498932602, fire_t: 4627730092099895296, 差值: 1766498932602ms
[ WARN] [1766498932.602870199]: 111111111111111111111
[ WARN] [1766498932.628344562]: 构造函数: 1766498932653
[ WARN] [1766498932.628414040]: now_time: 1766498932628, fire_t: 4627730092099895296, 差值: 1766498932628ms
[ WARN] [1766498932.628436720]: 111111111111111111111
[ WARN] [1766498932.652841657]: 构造函数: 1766498932677
[ WARN] [1766498932.652907587]: now_time: 1766498932652, fire_t: 4627730092099895296, 差值: 1766498932652ms
[ WARN] [1766498932.652919195]: 111111111111111111111
[ WARN] [1766498932.677990332]: 构造函数: 1766498932702
[ WARN] [1766498932.678044958]: now_time: 1766498932677, fire_t: 4627730092099895296, 差值: 1766498932677ms
[ WARN] [1766498932.678054217]: 111111111111111111111
[ WARN] [1766498932.703493548]: 构造函数: 1766498932728
[ WARN] [1766498932.703545353]: now_time: 1766498932703, fire_t: 4627730092099895296, 差值: 1766498932703ms
[ WARN] [1766498932.703556584]: 111111111111111111111
[ WARN] [1766498932.727852595]: 构造函数: 1766498932752
[ WARN] [1766498932.727890161]: now_time: 1766498932727, fire_t: 4627730092099895296, 差值: 1766498932727ms
[ WARN] [1766498932.727900079]: 111111111111111111111
[hikcamera-2] process has died [pid 19813, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/e5c2d8e4-e008-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [19822]
[ WARN] [1766498932.755455937]: 构造函数: 1766498932780
[ WARN] [1766498932.755495986]: now_time: 1766498932755, fire_t: 4627730092099895296, 差值: 1766498932755ms
[ WARN] [1766498932.755507286]: 111111111111111111111
[ WARN] [1766498932.780055565]: 构造函数: 1766498932804
[ WARN] [1766498932.780087135]: now_time: 1766498932779, fire_t: 4627730092099895296, 差值: 1766498932779ms
[ WARN] [1766498932.780095156]: 111111111111111111111
[ WARN] [1766498932.803273863]: 构造函数: 1766498932828
[ WARN] [1766498932.803333806]: now_time: 1766498932803, fire_t: 4627730092099895296, 差值: 1766498932803ms
[ WARN] [1766498932.803347274]: 111111111111111111111
[ WARN] [1766498932.827820312]: 构造函数: 1766498932852
[ WARN] [1766498932.827896675]: now_time: 1766498932827, fire_t: 4627730092099895296, 差值: 1766498932827ms
[ WARN] [1766498932.827912546]: 111111111111111111111
[ WARN] [1766498932.852794653]: 构造函数: 1766498932877
[ WARN] [1766498932.852828188]: now_time: 1766498932852, fire_t: 4627730092099895296, 差值: 1766498932852ms
[ WARN] [1766498932.852840463]: 111111111111111111111
[ WARN] [1766498932.878690888]: 构造函数: 1766498932903
[ WARN] [1766498932.878733784]: now_time: 1766498932878, fire_t: 4627730092099895296, 差值: 1766498932878ms
[ WARN] [1766498932.878745968]: 111111111111111111111
[ WARN] [1766498932.904411201]: 构造函数: 1766498932929
[ WARN] [1766498932.904460846]: now_time: 1766498932904, fire_t: 4627730092099895296, 差值: 1766498932904ms
[ WARN] [1766498932.904473375]: 111111111111111111111
^C[ WARN] [1766498932.928906407]: 构造函数: 1766498932953
[ WARN] [1766498932.928934181]: now_time: 1766498932928, fire_t: 4627730092099895296, 差值: 1766498932928ms
[ WARN] [1766498932.928941213]: 111111111111111111111
[tracker-5] killing on exit
[identify-4] killing on exit
[hikcamera-2] killing on exit
[rosout-1] killing on exit
[master] killing on exit
shutting down processing monitor...
... shutting down processing monitor complete
done
rm@rm-NUC11PAHi7:~$ 
#ifndef TRAJECTORY_VISUALIZER_HPP
#define TRAJECTORY_VISUALIZER_HPP
#include "math.hpp"
#include "CoorConverter.hpp"
#include <opencv2/opencv.hpp>
#include "GimbalPos.hpp"
#include "ros/ros.h"
namespace tools{
    const int FLASK_MAP_WIDTH = 1000;                   // 定义调试地图的水平分辨率
    const double FLASK_MAP_PETER_BY_BRIGHT = 1.;        // 默认亮度系数
    const int FLASK_MAP_MID_X = FLASK_MAP_WIDTH / 2;    // 地图的水平中心点,用于坐标变换的参考原点
    // 点绘制参数
    struct FlaskPoint {
        FlaskPoint(
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const int& radius,
            const int& thickness
        ):
            pt(pt),
            color(color),
            radius(radius),
            thickness(thickness) {}
        cv::Point2d pt;   // 圆心位置
        cv::Scalar color; // 颜色
        int radius;       // 半径
        int thickness;    // 线宽
    };
    struct FlaskLine {
        FlaskLine(
            const std::pair<cv::Point2f, cv::Point2f>& pt_pair,
            const cv::Scalar& color,
            const int& thickness
        ):
            pt_pair(pt_pair),
            color(color),
            thickness(thickness) {}
        std::pair<cv::Point2f, cv::Point2f> pt_pair;
        cv::Scalar color;
        int thickness;
    };
    // 文本绘制参数
    struct FlaskText {
        FlaskText(
            const std::string& str,
            const cv::Point2d& pt,
            const cv::Scalar& color,
            const double& scale
        ):
            str(str),
            pt(pt),
            color(color),
            scale(scale) {}
        std::string str;  // 文本内容
        cv::Point2d pt;   // 文本位置 (左下角)
        cv::Scalar color; // 颜色
        double scale;     // 字体大小
    };
    /*
        绘制流管理器
        @brief:
            收集绘制命令: 通过重载的<<操作符接收各种绘制元素
            批量执行绘制: 通过>>操作符将所有收集的命令绘制到图形上
            命令管理: 可以清空所有收集的绘制命令
    
/
    class FlaskStream {
    public:
        FlaskStream& operator<<(const char
 str);
        FlaskStream& operator<<(const std::string& str);
        FlaskStream& operator<<(const FlaskPoint& pt);
        FlaskStream& operator<<(const FlaskLine& line);
        FlaskStream& operator<<(const std::vector<FlaskLine>& lines);
        FlaskStream& operator<<(const FlaskText& text);
        FlaskStream& operator>>(cv::Mat& img);
        void clear();
    private:
        std::vector<std::string> logs;
        std::vector<FlaskPoint> pts;
        std::vector<FlaskLine> lines;
        std::vector<FlaskText> texts;
    };
    // 用于复现的瞄准参数
    // 移植代码的时候将这段代码移植到自瞄那里
    struct ShootParam {
        double v0 = 0.;                                                 // 子弹初速度
        double aim_angle = 0.;                                          // 发射仰角
        // Eigen::Vector3d aim_xyz_i_barrel = Eigen::Vector3d::Zero();     // 枪管坐标系瞄准点 (没有什么作用)
        Eigen::Vector3d target_xyz_i_camera = Eigen::Vector3d::Zero();  // 相机坐标系目标点
    };
    // 子弹命中位置信息
    struct HitPos {
        bool hit;
        Eigen::Vector3d pos;     // 子弹在世界坐标系上的位置
    };
    // 子弹图像投影信息
    struct HitCircle {
        bool hit;
        math::CircleF circle;    // 子弹在图像上的投影圆
    };
    // 匹配代价评估
    struct CaughtCost {
        bool caught;    // 是否满足匹配条件
        double cost;    // 匹配代价(越小越好)
    };
    // 子弹弹道物理模拟器
    class ProjectileSimulator {
    public:
        ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t) : coorConverter{coorConverter},shoot_param{shoot_param} ,fire_t{fire_t} {
            ROS_WARN("构造函数: %lld",fire_t);
        }
        // 子弹在图像平面上的投影计算
        auto get_circle_by_t(const double& t) const -> HitCircle;
        // 计算在指定时间t的子弹位置
        auto get_pos_by_t(const double& t) const -> HitPos;
        // 获取开火时间
        auto get_fire_t() const -> double;
    private:
        const double g { 9.8 };
        const long long fire_t;
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;
    };
    // 子弹位置信息
    struct IdPos {
        int id;
        Eigen::Vector3d pos;
    };
    // 子弹投影圆信息
    struct IdCircle {
        int id;
        math::CircleF circle; // 子弹在图像平面上的投影圆
    };
    // 子弹模拟器封装
    struct IdProj {
        int id;
        ProjectileSimulator proj;  // 子弹物理模拟器实例
    };
    // 自动瞄准误差校准(目前仅用来复现理想弹道)
    class AimCorrector {
    public:
        AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);
        // 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆
        auto get_circles(long long now_time) -> std::vector<IdCircle>;
        auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;
    private:
        std::list<IdProj> bullets;             // 活跃子弹容器模拟器
        CoordinateTransformer* coorConverter;  // 坐标变换器
        std::string config_path_;              // 存储配置路径
        ShootParam shoot_param;
        long long next_id = 0;
        long long last_fire_time = 0;
    };
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z);
    FlaskPoint pos_to_map_point(
        const Eigen::Vector3d& pos,
        const cv::Scalar& color,
        const int& radius,
        const int& thickness
    );
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN);
}
#endif // TRAJECTORY_VISUALIZER_HPP
#include "trajectory_visualizer.hpp"
#include <chrono>
namespace tools{
    auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        ROS_WARN("22222222222222222222222222222222");
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->cam2Map(bullet.pos);
        // 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();
        // 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);
        double radius = math::get_dis(edge_xy_u, center_xy_u); // 这里数学库要记得改成double类型,这里数学库应该还是float类型 // 这里数学库的这个函数已经更改成double类型
        return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };
    }
    auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {
        ROS_WARN("33333333333333333333333333333333333333333333333");
        // 将毫秒转换成毫秒
        double t_s = static_cast<double>(t * 0.001);
        double fire_t_s = static_cast<double>(this->fire_t * 0.001);
        double k = 0.1;   // 空气阻力系数
        // 计算水平位移
        double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);
        // 计算高度
        double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w
                / (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))
            + this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k
                / k;
        cout << "w: " << w << endl;
        cout << "h: " << h << endl;
        ROS_WARN("444444444444444444444444444444444444");
        // 弹道轨迹仅取决于目标点(理想弹道)
        const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);
        // 计算基准方向
        const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();
        const Eigen::Vector3d h_norm = { 0., 0., 1. };
        const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;
        const Eigen::Vector3d bullet_xyz_i_camera =this->coorConverter->gun2Cam(bullet_xyz_i_barrel);
        const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),
                                                     bullet_xyz_i_barrel(1, 0) };
        const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),
                                                     target_xyz_i_barrel(1, 0) };
        return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(),bullet_xyz_i_camera};
    }
    auto ProjectileSimulator::get_fire_t() const -> double {
        return this->fire_t;
    }
    AimCorrector::AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param) {
        this->shoot_param = shoot_param;
    }
    auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
        // 初始化结果向量
        std::vector<IdCircle> res;
        // 开始遍历子弹列表 bullets: 存储所有活跃子弹模拟器的链表
        for (auto it = this->bullets.begin(); it != this->bullets.end();) {
            // 检查子弹是否已发射
            // 当前图像时间 < 子弹发射时间
            // 是 -> 子弹还未发射,跳过
            // 否 -> 子弹已发射,继续处理
            ROS_WARN("now_time: %lld, fire_t: %lld, 差值: %lldms",
                 now_time, it->proj.get_fire_t(), it->proj.get_fire_t() - now_time);
            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111");
                ++it;
                continue;
            }
            ROS_WARN("66666666666666666666666666666666");
            // 获取子弹在当前时刻的投影圆
            HitCircle hit_circle = it->proj.get_circle_by_t(now_time);
            // 检查子弹是否已击中 -> 已击中删除
            if (hit_circle.hit) {
                it = this->bullets.erase(it);
            } else {
            // 处理未击中的子弹 -> 未击中添加到结果,迭代器
                res.push_back(IdCircle { it->id, hit_circle.circle });
                ++it;
            }
        }
        return res;
    }
    // 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 50;  // 发射间隔：50毫秒
        // 将秒转换为毫秒
        long long eTime_ms = static_cast<long long>(eTime * 1000);
        long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
        long long additional_delay = 25;  // 0.025秒 = 25毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime_ms + additional_delay + command_timespan_ms)
                });
                this->last_fire_time = current_time;
            }
        }
    }
    FlaskStream& FlaskStream::operator<<(const char* str) {
        this->logs.emplace_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::string& str) {
        this->logs.push_back(str);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskPoint& pt) {
        this->pts.push_back(pt);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskLine& line) {
        this->lines.push_back(line);
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const std::vector<FlaskLine>& lines) {
        for (const auto& line: lines) {
            this->lines.push_back(line);
        }
        return *this;
    }
    FlaskStream& FlaskStream::operator<<(const FlaskText& text) {
        this->texts.push_back(text);
        return *this;
    }
    FlaskStream& FlaskStream::operator>>(cv::Mat& img) {
        int cnt = 0;
        for (auto& str: this->logs) {
            cv::putText(
                img,
                str,
                { 20, 80 + cnt * 24 },
                cv::FONT_HERSHEY_DUPLEX,
                0.8,
                { 0, 0, 255 }
            );
            ++cnt;
        }
        for (auto& pt: this->pts) {
            cv::circle(img, pt.pt, pt.radius, pt.color, pt.thickness);
        }
        for (auto& line: this->lines) {
            cv::line(img, line.pt_pair.first, line.pt_pair.second, line.color, line.thickness);
        }
        for (auto& text: this->texts) {
            cv::putText(
                img,
                text.str,
                { int(text.pt.x), int(text.pt.y) },
                cv::FONT_HERSHEY_DUPLEX,
                text.scale,
                text.color
            );
        }
        return *this;
    }
    void FlaskStream::clear() {
        this->logs.clear();
        this->pts.clear();
        this->lines.clear();
        this->texts.clear();
    }
    cv::Scalar heightened_color(const cv::Scalar& color, const double& z) {
        cv::Scalar res;
        for (int i = 0; i < 3; ++i) {
            res[i] = z >= 0. ? 255. - (255. - color[i]) * std::pow(0.5, z / FLASK_MAP_PETER_BY_BRIGHT)
                            : color[i] * std::pow(0.5, -z / FLASK_MAP_PETER_BY_BRIGHT);
        }
        return res;
    }
    // FlaskPoint pos_to_map_point(
    //     const Eigen::Vector3d& pos,
    //     const cv::Scalar& color,
    //     const int& radius,
    //     const int& thickness
    // ) {
    //     return FlaskPoint(
    //         { float(
    //             FLASK_MAP_MID_X
    //             + pos(0, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ),
    //         float(
    //             FLASK_MAP_MID_Y
    //             - pos(1, 0) * base::get_param<double>("auto-aim.debug.flask.map.pixel-per-meter")
    //         ) },
    //         heightened_color(color, pos(2, 0)),
    //         radius,
    //         thickness
    //     );
    // }
    // auto Stm32Shoot::add(const int& id, const double& img_t) -> void {
    //     // 时间超过 t + latency 后可以发射
    //     if (this->pending_signals.size() + 1 <= Stm32Shoot::MAX_SZ) {
    //         this->pending_signals.push_back(Stm32Shoot::IdT { id, img_t });
    //     }
    // }
    // auto Stm32Shoot::get_last_shoot_id(const double& img_t) -> int {
    //     // 实际上是传输过去有延迟，
    //     while (!this->pending_signals.empty()
    //         && img_t >= this->pending_signals.front().img_t + Stm32Shoot::SHOOT_LATENCY)
    //     {
    //         // 信号已经到达，进行信号处理
    //         if (this->pending_signals.front().img_t >= this->last_shoot.img_t
    //                 + base::get_param<double>("auto-aim.ec-simulator.shoot-interval"))
    //         {
    //             this->last_shoot = this->pending_signals.front();
    //         }
    //         this->pending_signals.pop_front();
    //     }
    //     return this->last_shoot.id;
    // }
    // 绘制模拟发射的子弹
    void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,cv::Mat& img,long long now_time, long long eTime, long long COMMAND_TIMESPAN){
        FlaskStream flask_aim;
        FlaskStream flask_map;
        flask_aim.clear();
        flask_map.clear();
        AimCorrector aim_corrector = AimCorrector(coorConverter,shoot_param);
        // 更新子弹序列
        // 传入当前帧的时间和当前帧的瞄准姿态
        aim_corrector.update_bullet(now_time,eTime,COMMAND_TIMESPAN);
        std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);
        for (auto& bullet: bullets) {
            flask_aim << FlaskPoint(
                bullet.circle.center,
                { 0, 0, 255 },
                bullet.circle.r,
                2
            );
            flask_aim << FlaskText(
                std::to_string(bullet.id),
                { bullet.circle.center.x + 20.f, bullet.circle.center.y },
                { 0, 0, 255 },
                0.8
            );
            // flask_map << pos_to_map_point(bullet.pos,{0, 0, 255}, 4,-1);
        }
        flask_aim >> img;
    }
}
分析一下是什么问题导致的?
从日志可以看到，构造函数中打印的fire_t是正常的（比如1766498932101，和当前时间now_time接近，加上延迟后大约为当前时间+延迟毫秒数）。但是在get_circles中打印的it->proj.get_fire_t()却是一个巨大的数4627730092099895296。
注意到，在ProjectileSimulator类中，fire_t成员变量是const long long类型，并且在构造函数中初始化。但是，在get_fire_t()函数中，返回的是double类型。这里可能存在类型转换问题。
另外，在get_circles函数中，我们打印时使用的是%lld格式，但是it->proj.get_fire_t()返回的是double，这会导致未定义行为（因为printf期望的是long long，但传入的是double，可能按double的二进制表示解释为整数，从而得到一个非常大的数）。
解决方案：
将get_fire_t()的返回类型改为long long，以匹配fire_t的类型。
确保在打印时使用正确的格式。
但是，注意在get_circles函数中，我们使用了ROS_WARN打印，其中%lld期望的是long long，而如果get_fire_t()返回double，则会导致问题。
让我们检查一下：
在头文件中，get_fire_t()声明为返回double，但实际存储的fire_t是long long。在实现中，get_fire_t()返回this->fire_t，这里将long long隐式转换为double。
在get_circles函数中，我们使用%lld打印一个double，这是错误的。
有两种修改方式：
方式一：修改get_fire_t()返回long long，这样打印和比较都用long long。
方式二：在打印时将double转换为long long，但注意double可能无法精确表示这么大的整数（因为double的尾数只有53位，而long long是64位，可能会丢失精度）。
由于fire_t是时间戳（毫秒），可能超过double的精确表示范围（2^53约等于9e15），而我们的fire_t大约1e12，还在精确范围内，但是为了安全和一致性，我们使用long long。
因此，我建议修改get_fire_t()返回long long。
修改步骤：
在头文件中，将get_fire_t()的返回类型改为long long。
在实现文件中，将get_fire_t()的返回类型改为long long，并返回fire_t。
确保在get_circles函数中，使用%lld打印long long。
另外，在get_pos_by_t函数中，我们使用了this->fire_t，它也是long long，所以在计算时转换为double。
同时，注意在update_bullet函数中，我们传递的fire_t是long long，所以没有问题。
让我们修改代码。
根据日志，问题非常明显了！让我分析一下：
问题分析
## 1. 构造函数中的时间正常
从日志可以看到：
构造函数打印的 fire_t 是正常的，如 1766498932101（这是正常的时间戳）
这个值大约是当前时间 now_time 加上延迟（50ms间隔 + 各种延迟）
## 2. 但是 get_fire_t() 返回的是错误的巨大值
在 get_circles 函数中，it->proj.get_fire_t() 返回的是 4627730092099895296（约4.6e18），这完全不对！
根本原因
问题出在 头文件和实现文件之间的类型不匹配：
头文件声明：
cpp
auto get_fire_t() const -> double;  // 声明返回 double
实现文件实现：
cpp
auto ProjectileSimulator::get_fire_t() const -> double {return this->fire_t;  // fire_t 是 long long，隐式转换为 double}
问题：当 long long 转换为 double 时，由于 double 的精度问题（只有53位尾数），而 long long 是64位整数，可能会发生精度丢失，导致值变成完全不同的数字。
解决方案
方案1：统一使用 long long 类型（推荐）
cpp
// 修改头文件auto get_fire_t() const -> long long;  // 改为返回 long long
cpp
// 修改实现文件auto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}
方案2：统一使用 double 类型
cpp
// 修改头文件class ProjectileSimulator {public:ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const double& fire_t)  // 参数改为 double: coorConverter{coorConverter},          shoot_param{shoot_param},          fire_t{fire_t} {ROS_WARN("构造函数: %.0f", fire_t);}private:const double fire_t;  // 改为 double};
完整修复代码
## 1. 修改头文件 (trajectory_visualizer.hpp)
cpp
#ifndef TRAJECTORY_VISUALIZER_HPP#define TRAJECTORY_VISUALIZER_HPP#include "math.hpp"#include "CoorConverter.hpp"#include <opencv2/opencv.hpp>#include "GimbalPos.hpp"#include "ros/ros.h"namespace tools {// ... 其他结构体定义保持不变 ...// 子弹弹道物理模拟器class ProjectileSimulator {public:ProjectileSimulator(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,const long long& fire_t): coorConverter{coorConverter},              shoot_param{shoot_param},              fire_t{fire_t} {ROS_WARN("构造函数: %lld", fire_t);}// 子弹在图像平面上的投影计算auto get_circle_by_t(const double& t) const -> HitCircle;// 计算在指定时间t的子弹位置auto get_pos_by_t(const double& t) const -> HitPos;// 修改这里：返回 long longauto get_fire_t() const -> long long;private:const double g { 9.8 };const long long fire_t;  // long long 类型
        CoordinateTransformer* coorConverter;
        ShootParam shoot_param;};// ... 其他类定义保持不变 ...}#endif // TRAJECTORY_VISUALIZER_HPP
## 2. 修改实现文件 (trajectory_visualizer.cpp)
cpp
#include "trajectory_visualizer.hpp"#include <chrono>namespace tools {auto ProjectileSimulator::get_circle_by_t(const double& t) const -> HitCircle {
        HitPos bullet = this->get_pos_by_t(t);
        Eigen::Vector3d xyz_c = this->coorConverter->cam2Map(bullet.pos);// 沿着正 y 轴与视角的叉积方向得到一个边缘坐标，以计算半径
        Eigen::Vector3d crossed = Eigen::Vector3d(0., 1., 0.).cross(xyz_c).normalized();// 这里用到的参数应该是小弹丸的半径
        Eigen::Vector3d edge_xyz_c = xyz_c + crossed * 0.0085;
        Eigen::Vector3d edge_xyz_i = this->coorConverter->cam2Map(edge_xyz_c);
        cv::Point2d edge_xy_u = this->coorConverter->map2pixel(edge_xyz_i);
        cv::Point2d center_xy_u = this->coorConverter->map2pixel(bullet.pos);double radius = math::get_dis(edge_xy_u, center_xy_u);return HitCircle { bullet.hit, math::CircleF(edge_xy_u, radius) };}auto ProjectileSimulator::get_pos_by_t(const double& t) const -> HitPos {// 将毫秒转换成秒double t_s = t * 0.001;double fire_t_s = this->fire_t * 0.001;double k = 0.1;   // 空气阻力系数// 计算水平位移double w = (t_s - fire_t_s) * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle);// 计算高度double h = (k * this->shoot_param.v0 * std::sin(this->shoot_param.aim_angle) + this->g) * k * w/ (k * k * this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))+ this->g * std::log(1. - (k * w) / (this->shoot_param.v0 * std::cos(this->shoot_param.aim_angle))) / k/ k;// 弹道轨迹仅取决于目标点(理想弹道)const Eigen::Vector3d target_xyz_i_barrel = this->coorConverter->cam2Gun(this->shoot_param.target_xyz_i_camera);// 计算基准方向const Eigen::Vector3d w_norm = Eigen::Vector3d(target_xyz_i_barrel(0, 0), target_xyz_i_barrel(1, 0), 0).normalized();const Eigen::Vector3d h_norm = { 0., 0., 1. };const Eigen::Vector3d bullet_xyz_i_barrel = w * w_norm + h * h_norm;const Eigen::Vector3d bullet_xyz_i_camera = this->coorConverter->gun2Cam(bullet_xyz_i_barrel);const Eigen::Vector2d bullet_xy_i_barrel = { bullet_xyz_i_barrel(0, 0),bullet_xyz_i_barrel(1, 0) };const Eigen::Vector2d target_xy_i_barrel = { target_xyz_i_barrel(0, 0),target_xyz_i_barrel(1, 0) };return HitPos { bullet_xy_i_barrel.norm() >= target_xy_i_barrel.norm(), bullet_xyz_i_camera };}// 修改这里：返回 long longauto ProjectileSimulator::get_fire_t() const -> long long {return this->fire_t;}// ... 其他函数实现保持不变 ...}
## 3. 修改调试输出
在 get_circles 函数中，确保打印格式正确：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 获取发射时间long long fire_t = it->proj.get_fire_t();// 打印详细信息ROS_WARN("时间比较: now_time=%lld, fire_t=%lld, 差值=%lldms",                 now_time, fire_t, fire_t - now_time);if (now_time < fire_t) {ROS_WARN("子弹未发射，还需等待 %lld 毫秒", fire_t - now_time);++it; 
            continue;}ROS_WARN("子弹已发射！已飞行 %lld 毫秒", now_time - fire_t);// 获取子弹在当前时刻的投影圆
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果,迭代器
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
关键修改点
类型一致性：get_fire_t() 现在返回 long long，与存储的类型一致
避免精度丢失：不再进行 long long 到 double 的隐式转换
正确的格式说明符：使用 %lld 打印 long long 类型
为什么会出现这个错误？
当 long long 值被当作 double 传递给 printf 的 %lld 格式时，会发生未定义行为。printf 会从内存中读取64位整数，但由于传递的是 double（8字节，但内部表示不同），它读取到的就是错误的数据。
简而言之：你告诉 printf 要读取一个 long long，但实际上传递了一个 double，导致内存解释错误。
修复后，程序应该能正确显示子弹发射时间，并且当 now_time >= fire_t 时，子弹就会开始显示飞行轨迹。
[ WARN] [1766499224.339994458]: 111111111111111111111
[ WARN] [1766499224.365333418]: 构造函数: 1766499224390
[ WARN] [1766499224.365381748]: now_time: 1766499224365, fire_t: 1766499224390, 差值: 25ms
[ WARN] [1766499224.365392342]: 111111111111111111111
[hikcamera-2] process has died [pid 21047, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/8b0b56d2-e009-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/8b0b56d2-e009-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [21055]
[ WARN] [1766499224.391052007]: 构造函数: 1766499224416
[ WARN] [1766499224.391097736]: now_time: 1766499224391, fire_t: 1766499224416, 差值: 25ms
[ WARN] [1766499224.391112294]: 111111111111111111111
[ WARN] [1766499224.416360678]: 构造函数: 1766499224441
[ WARN] [1766499224.416391542]: now_time: 1766499224416, fire_t: 1766499224441, 差值: 25ms
[ WARN] [1766499224.416399873]: 111111111111111111111
[ WARN] [1766499224.440360583]: 构造函数: 1766499224465
[ WARN] [1766499224.440400402]: now_time: 1766499224440, fire_t: 1766499224465, 差值: 25ms
[ WARN] [1766499224.440410683]: 111111111111111111111
[ WARN] [1766499224.466583353]: 构造函数: 1766499224491
[ WARN] [1766499224.466615975]: now_time: 1766499224466, fire_t: 1766499224491, 差值: 25ms
[ WARN] [1766499224.466625743]: 111111111111111111111
[ WARN] [1766499224.491281160]: 构造函数: 1766499224516
[ WARN] [1766499224.491366704]: now_time: 1766499224491, fire_t: 1766499224516, 差值: 25ms
[ WARN] [1766499224.491378868]: 111111111111111111111
[ WARN] [1766499224.516487291]: 构造函数: 1766499224541
[ WARN] [1766499224.516521151]: now_time: 1766499224516, fire_t: 1766499224541, 差值: 25ms
[ WARN] [1766499224.516531777]: 111111111111111111111
[ WARN] [1766499224.539867514]: 构造函数: 1766499224564
[ WARN] [1766499224.539899906]: now_time: 1766499224539, fire_t: 1766499224564, 差值: 25ms
[ WARN] [1766499224.539910284]: 111111111111111111111
[ WARN] [1766499224.565722170]: 构造函数: 1766499224590
[ WARN] [1766499224.565766930]: now_time: 1766499224565, fire_t: 1766499224590, 差值: 25ms
[ WARN] [1766499224.565778946]: 111111111111111111111
[ WARN] [1766499224.590320500]: 构造函数: 1766499224615
[ WARN] [1766499224.590361457]: now_time: 1766499224590, fire_t: 1766499224615, 差值: 25ms
[ WARN] [1766499224.590370662]: 111111111111111111111
[ WARN] [1766499224.615417853]: 构造函数: 1766499224640
[ WARN] [1766499224.615466837]: now_time: 1766499224615, fire_t: 1766499224640, 差值: 25ms
[ WARN] [1766499224.615480620]: 111111111111111111111
[ WARN] [1766499224.639006819]: 构造函数: 1766499224663
[ WARN] [1766499224.639049653]: now_time: 1766499224638, fire_t: 1766499224663, 差值: 25ms
[ WARN] [1766499224.639058831]: 111111111111111111111
[ WARN] [1766499224.664996044]: 构造函数: 1766499224689
[ WARN] [1766499224.665024559]: now_time: 1766499224664, fire_t: 1766499224689, 差值: 25ms
[ WARN] [1766499224.665033664]: 111111111111111111111
[ WARN] [1766499224.691008541]: 构造函数: 1766499224715
[ WARN] [1766499224.691054708]: now_time: 1766499224690, fire_t: 1766499224715, 差值: 25ms
[ WARN] [1766499224.691068111]: 111111111111111111111
[hikcamera-2] process has died [pid 21055, exit code 255, cmd /home/rm/ws_glut_vison/devel/lib/rm_hikcamera/hikcamera __name:=hikcamera __log:=/home/rm/.ros/log/8b0b56d2-e009-11f0-881b-033937e15f81/hikcamera-2.log].
log file: /home/rm/.ros/log/8b0b56d2-e009-11f0-881b-033937e15f81/hikcamera-2*.log
[hikcamera-2] restarting process
process[hikcamera-2]: started with pid [21064]
[ WARN] [1766499224.716570775]: 构造函数: 1766499224741
[ WARN] [1766499224.716618554]: now_time: 1766499224716, fire_t: 1766499224741, 差值: 25ms
[ WARN] [1766499224.716633023]: 111111111111111111111
[ WARN] [1766499224.741707545]: 构造函数: 1766499224766
[ WARN] [1766499224.741756017]: now_time: 1766499224741, fire_t: 1766499224766, 差值: 25ms
[ WARN] [1766499224.741767048]: 111111111111111111111
[ WARN] [1766499224.765959369]: 构造函数: 1766499224790
[ WARN] [1766499224.766089369]: now_time: 1766499224765, fire_t: 1766499224790, 差值: 25ms
[ WARN] [1766499224.766110943]: 111111111111111111111
[ WARN] [1766499224.792502752]: 构造函数: 1766499224817
[ WARN] [1766499224.792547161]: now_time: 1766499224792, fire_t: 1766499224817, 差值: 25ms
[ WARN] [1766499224.792561376]: 111111111111111111111
[ WARN] [1766499224.816675912]: 构造函数: 1766499224841
[ WARN] [1766499224.816736832]: now_time: 1766499224816, fire_t: 1766499224841, 差值: 25ms
[ WARN] [1766499224.816750511]: 111111111111111111111
[ WARN] [1766499224.841117446]: 构造函数: 1766499224866
[ WARN] [1766499224.841215303]: now_time: 1766499224841, fire_t: 1766499224866, 差值: 25ms
[ WARN] [1766499224.841235442]: 111111111111111111111
[ WARN] [1766499224.867828799]: 构造函数: 1766499224892
[ WARN] [1766499224.867885115]: now_time: 1766499224867, fire_t: 1766499224892, 差值: 25ms
[ WARN] [1766499224.867899800]: 111111111111111111111
[ WARN] [1766499224.893400233]: 构造函数: 1766499224918
[ WARN] [1766499224.893437171]: now_time: 1766499224893, fire_t: 1766499224918, 差值: 25ms
[ WARN] [1766499224.893445229]: 111111111111111111111
[ WARN] [1766499224.918721369]: 构造函数: 1766499224943
[ WARN] [1766499224.918763103]: now_time: 1766499224918, fire_t: 1766499224943, 差值: 25ms
[ WARN] [1766499224.918773842]: 111111111111111111111
[ WARN] [1766499224.942702635]: 构造函数: 1766499224967
[ WARN] [1766499224.942731168]: now_time: 1766499224942, fire_t: 1766499224967, 差值: 25ms
[ WARN] [1766499224.942738151]: 111111111111111111111
[ WARN] [1766499224.967903158]: 构造函数: 1766499224992
[ WARN] [1766499224.967943012]: now_time: 1766499224967, fire_t: 1766499224992, 差值: 25ms
[ WARN] [1766499224.967952577]: 111111111111111111111
[ WARN] [1766499224.993416062]: 构造函数: 1766499225018
[ WARN] [1766499224.993469889]: now_time: 1766499224993, fire_t: 1766499225018, 差值: 25ms
我已经统一成double 了为什么还是没有跑到那个            if (now_time < it->proj.get_fire_t()) {
                ROS_WARN("111111111111111111111");
                ++it;
                continue;
            }的下面打印出666666666666
问题分析：
从日志中可以看出，now_time 和 fire_t 的差值一直是25毫秒，且now_time始终小于fire_t，所以每次都会进入if语句，打印111111111，然后continue，不会执行后面的代码。
为什么差值总是25毫秒？
根据你的设计，在update_bullet中，你设置的发射间隔是50毫秒，并且你设置了延迟：eTime_ms + additional_delay + command_timespan_ms。但是，从日志看，构造函数中打印的fire_t和当前的now_time差值都是25毫秒，而不是你预期的延迟（应该是eTime+0.025+COMMAND_TIMESPAN，单位是秒，转换成毫秒后应该不止25毫秒）。
可能的原因：
你在update_bullet中计算延迟时，传入的eTime和COMMAND_TIMESPAN可能是0，或者很小，导致总延迟只有25毫秒（即additional_delay）。
你的now_time是当前时间，而fire_t是当前时间加上25毫秒，所以总是差25毫秒。
但是，即使差25毫秒，由于now_time < fire_t，所以子弹仍然被认为是未发射，所以不会进入后面的代码。
你想要的效果是：子弹在创建后，经过一段延迟（比如137毫秒）再发射。但是，从日志看，你创建的子弹的发射时间（fire_t）只比当前时间晚25毫秒，所以你需要等待25毫秒，子弹才会发射。
然而，由于你的程序运行很快，每50毫秒创建一个新子弹，每个子弹的发射时间都是创建时间+25毫秒。但是，你的now_time是当前时间，它也在增长。所以，你可能会看到，当now_time增长到超过某个子弹的fire_t时，那个子弹就会发射。
但是，从日志看，你打印的时间差都是25毫秒，这意味着你每次检查的时候，子弹都还需要25毫秒才发射。这可能是因为你的程序运行频率很高，每25毫秒左右就检查一次，而子弹的发射时间总是当前时间+25毫秒，所以总是差25毫秒。
实际上，从日志的时间戳看，两次打印的时间间隔大约是25毫秒（例如1766499224.339994到1766499224.365333，间隔约25毫秒）。所以，每次检查时，子弹的发射时间都是当前时间+25毫秒，所以永远不会发射。
这可能是由于你计算fire_t的方式不对。在update_bullet中，你计算fire_t为：
cpp
current_time + eTime_ms + additional_delay + command_timespan_ms
但是，你传入的eTime和COMMAND_TIMESPAN可能是0，所以实际上只加了25毫秒（additional_delay）。
你需要检查在Tracker中调用draw_simulated_bullets时传入的eTime和COMMAND_TIMESPAN是多少。可能是这两个参数都是0，所以总延迟只有25毫秒。
另外，你可能需要调整你的逻辑：如果你希望子弹在创建后立即发射，那么就不应该加延迟。如果你希望模拟真实的延迟，那么应该加上正确的延迟值。
从你的描述中，你希望子弹有发射延迟（曝光、开火延迟、自瞄程序运行耗时等），所以你应该将这些延迟正确地计算出来并传入。
现在，由于延迟只有25毫秒，而你的程序运行间隔大约是25毫秒，所以每次检查时，子弹都还没有到发射时间。
解决方法：
确保传入的eTime和COMMAND_TIMESPAN是正确的值（单位是秒）。在Tracker中，你从配置文件中读取了这些值，应该将它们传入。
如果你希望子弹在创建后立即发射，可以将延迟设置为0。
另外，注意：你的发射间隔是50毫秒，而子弹的发射延迟是25毫秒，所以子弹会在创建后的25毫秒发射。但是，由于你的检查频率很高（大约25毫秒一次），所以第一次检查时，子弹还需要25毫秒发射，第二次检查时，子弹可能已经发射了（因为两次检查间隔25毫秒，而子弹的发射延迟也是25毫秒）。
但是，从日志看，每次检查的时间间隔和子弹的剩余延迟时间几乎相同，所以总是差25毫秒。这可能是因为你的系统时间戳精度问题，或者你的循环运行频率刚好是40Hz（25毫秒一次）。
你可以尝试在get_circles函数中，将条件改为小于等于0，即：
cpp
if (now_time <= it->proj.get_fire_t()) {
这样，当时间相等时，也会认为子弹已发射。
或者，你可以将发射时间设置得比当前时间稍微提前一点，比如减去1毫秒，以抵消时间戳的误差。
但是，更根本的问题是，你的延迟计算可能不对。请检查在Tracker中调用draw_simulated_bullets时传入的eTime和COMMAND_TIMESPAN的值。
如果这些值都是0，那么总延迟就是25毫秒。而你的循环周期大约是25毫秒，所以每次检查时，子弹都还没有到发射时间。
你可以尝试将additional_delay设置为0，这样子弹的发射时间就是当前时间，那么下次检查时，now_time应该大于等于fire_t，就会进入后面的代码。
总结：
检查传入的eTime和COMMAND_TIMESPAN参数。
考虑调整条件判断，使用now_time <= fire_t。
考虑减少延迟，或者调整循环频率，使得子弹能够在预期的时间内发射。
另外，注意你的hikcamera进程一直在重启，可能是相机驱动问题，但这可能不影响你的逻辑。
现在，为了快速测试，你可以将additional_delay设置为0，然后看看子弹是否会发射。如果还是不行，可能是你的时间戳单位或计算有问题。
请尝试修改update_bullet中的延迟计算，将fire_t设置为current_time，即立即发射，然后观察是否能够进入后面的代码。
如果立即发射可以，那么说明问题在于延迟计算。如果立即发射还是不行，那么可能是其他问题，比如时间戳的单位不一致等。
从日志可以看到，现在时间值已经正常了！但问题是：
子弹的发射时间 fire_t 总是比当前时间 now_time 大25毫秒，所以 now_time < fire_t 始终成立，子弹一直被判定为"未发射"状态。
问题分析
日志显示：
now_time: 1766499224365
fire_t: 1766499224390
差值: 25毫秒
这意味着你设置的发射延迟正好是25毫秒，导致每次检查时子弹都还需要等待25毫秒才能发射。
为什么子弹一直不发射？
在你的代码中，update_bullet 函数计算发射时间为：
cpp
current_time + eTime_ms + additional_delay + command_timespan_ms
从日志看，总延迟 = 25毫秒。这很可能是：
eTime_ms = 0 (eTime = 0秒)
command_timespan_ms = 0 (COMMAND_TIMESPAN = 0秒)
additional_delay = 25 (固定25毫秒)
解决方案
方案1：放宽发射条件
让子弹在接近发射时间时就显示：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();long long time_diff = fire_t - now_time;// 如果子弹将在未来10毫秒内发射，就认为它已经发射了if (time_diff > 10) {  // 原来是 if (now_time < fire_t)ROS_WARN("子弹未发射，还需等待 %lld 毫秒", time_diff);++it; 
            continue;}ROS_WARN("子弹已发射或即将发射！");
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
方案2：减少发射延迟
修改 update_bullet 中的延迟计算：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 0;  // 改为0毫秒，立即发射if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {// 使用当前时间，不加延迟long long fire_time = current_time;  // 立即发射ROS_WARN("创建子弹: 立即发射，fire_time = now_time = %lld", fire_time);
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
方案3：在Tracker中检查传入的参数
检查你在Tracker中调用 draw_simulated_bullets 时传入的参数：
cpp
// 在Tracker的reconstruction_choose_compensation函数中：
cout << "eTime: " << eTime << "秒" << endl;
cout << "COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << "秒" << endl;// 如果这些值都是0，那么总延迟就是25毫秒
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             eTime,    // 检查这个值
                             COMMAND_TIMESPAN);  // 检查这个值
方案4：最佳方案 - 允许子弹显示发射前的状态
即使子弹还没发射，也可以显示它的"准备状态"：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;for (auto it = this->bullets.begin(); it != this->bullets.end();) {long long fire_t = it->proj.get_fire_t();// 总是计算子弹位置，不管是否已发射
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 如果子弹已击中，删除它if (hit_circle.hit) {
            it = this->bullets.erase(it);} else {// 子弹未击中，添加到结果// 可以根据发射状态设置不同的颜色
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}return res;}
最简单的测试方法
直接在 get_circles 中强制让所有子弹都"已发射"：
cpp
auto AimCorrector::get_circles(long long now_time) -> std::vector<IdCircle>{
    std::vector<IdCircle> res;ROS_WARN("=== get_circles 开始，子弹数量: %zu ===", bullets.size());for (auto it = this->bullets.begin(); it != this->bullets.end();) {// 强制认为所有子弹都已发射（测试用）ROS_WARN("处理子弹 %d，强制认为已发射", it->id);// 获取子弹在当前时刻的投影圆
        HitCircle hit_circle = it->proj.get_circle_by_t(now_time);// 检查子弹是否已击中 -> 已击中删除if (hit_circle.hit) {ROS_WARN("子弹 %d 已击中，删除", it->id);
            it = this->bullets.erase(it);} else {// 处理未击中的子弹 -> 未击中添加到结果ROS_WARN("子弹 %d 未击中，添加到绘制列表", it->id);
            res.push_back(IdCircle { it->id, hit_circle.circle });++it;}}ROS_WARN("=== get_circles 结束，返回 %zu 个子弹 ===", res.size());return res;}
根本原因总结
你的代码逻辑是正确的，但存在一个小问题：
时间精度问题：子弹有25毫秒的发射延迟，而你的程序检查频率大约也是每25毫秒一次。这导致：
第一次检查：子弹还需25毫秒发射

子弹的发射时间 fire_t 总是比当前时间 now_time 大25毫秒，所以 now_time < fire_t 始终成立，子弹一直被判定为"未发射"状态。  为什么子弹的发射时间总是比当前时间大25ms?
根据日志，子弹的发射时间总是比当前时间大25ms，这是因为在update_bullet函数中，计算发射时间时，你使用了固定的额外延迟25ms（additional_delay = 25），并且可能eTime和COMMAND_TIMESPAN都是0，所以总延迟就是25ms。
但是，这里有一个关键问题：你是在同一帧中创建子弹并立即检查。也就是说，你调用update_bullet时传入的current_time是当前时间，然后你设置子弹的发射时间为current_time + 25ms。紧接着你调用get_circles(now_time)，这里的now_time与current_time相同（或者非常接近，相差几毫秒）。因此，在检查时，子弹的发射时间总是比当前时间大25ms左右。
为了看到子弹发射后的轨迹，你需要等待至少25ms。但是你的程序是实时运行的，每一帧都在创建新的子弹（每50ms创建一颗），并且每一帧都在检查。由于发射延迟（25ms）小于创建间隔（50ms），所以你会不断创建新的子弹，但每一颗子弹在创建后的25ms内都不会被绘制。
如果你希望子弹创建后立即显示（即没有发射延迟），那么你应该将additional_delay设为0，并且确保eTime和COMMAND_TIMESPAN也是0。
但是，从你的需求描述来看，你希望模拟真实的发射延迟（曝光、开火延迟等）。如果是这样，那么子弹在发射延迟内不会显示是正常的。然而，你可能希望看到的是子弹发射后的轨迹，而不是延迟内的等待。
根据你的需求，你有两种选择：
如果你想模拟真实情况，即子弹有发射延迟，那么你需要在子弹发射延迟过后才能看到它的轨迹。这意味着在子弹创建后的前25ms内，你不会看到它，25ms后你才能看到它的飞行轨迹。但是，由于你的程序是连续运行的，你会看到每一颗子弹在创建25ms后开始显示，并且显示它的飞行轨迹直到击中目标。
如果你只是想测试弹道计算，希望子弹立即显示，那么你应该将发射延迟设为0。
从你的日志来看，你似乎希望子弹立即显示，因为你不理解为什么子弹一直没有显示。所以，我建议你先将发射延迟设为0，确保弹道计算正确，然后再考虑添加延迟。
修改方法：
在update_bullet函数中，将additional_delay设为0，同时检查eTime和COMMAND_TIMESPAN是否为0。如果它们不为0，并且你希望立即显示，那么也应该将它们设为0。
但是，请注意，在Tracker中调用draw_simulated_bullets时，传入的eTime和COMMAND_TIMESPAN可能不是0。你需要检查Tracker中这两个参数的值。
如果你确定希望子弹立即显示，可以这样修改：
cpp
   auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 0;  // 改为0毫秒，立即发射if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {long long fire_time = current_time;  // 不加延迟
               bullets.push_back(IdProj {
                   next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
这样修改后，子弹的发射时间就是当前时间，在get_circles中，now_time应该等于或略大于fire_t（因为创建子弹和检查可能在同一帧，但now_time可能稍微大一点，因为代码执行需要时间），从而进入绘制流程。
如果修改后仍然没有进入绘制流程，请检查now_time和fire_t的打印值，确保now_time大于等于fire_t。
另外，注意你的程序中有多个ROS_WARN打印，可能会影响性能，建议调试完成后减少打印。
最后，如果你希望保留发射延迟，但又想看到子弹在延迟期间的位置（比如在枪口等待），那么你需要修改get_circles函数，即使子弹未发射，也计算其位置（在枪口位置），这样你就可以看到子弹在发射前停留在枪口，发射后再飞出去。但这需要修改弹道计算，在未发射时返回枪口位置。
这是一个很好的问题！让我详细分析为什么子弹的发射时间总是比当前时间大25ms：
原因分析
## 1. 查看你的 update_bullet 函数
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime * 1000);long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒  ← 这里！if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
            bullets.push_back(IdProj {
                next_id++,// 发射时间 = 当前时间 + 25msProjectileSimulator(coorConverter, this->shoot_param,                                   current_time + eTime_ms + additional_delay + command_timespan_ms)});this->last_fire_time = current_time;}}}
## 2. 数学计算
假设：
eTime = 0.002秒 = 2毫秒
COMMAND_TIMESPAN = 0.11秒 = 110毫秒
additional_delay = 25毫秒
总延迟 = 2 + 25 + 110 = 137毫秒
但是从日志看，差值只有25毫秒！这说明：
很可能 eTime 和 COMMAND_TIMESPAN 都是 0 或非常小！
## 3. 在Tracker中检查参数值
你需要检查Tracker中传入的参数：
cpp
// 在Tracker的reconstruction_choose_compensation函数中
cout << "DEBUG - eTime: " << eTime << " (秒)" << endl;
cout << "DEBUG - COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << " (秒)" << endl;
cout << "DEBUG - eTime_ms: " << static_cast<long long>(eTime * 1000) << " (毫秒)" << endl;
cout << "DEBUG - COMMAND_TIMESPAN_ms: " << static_cast<long long>(COMMAND_TIMESPAN * 1000) << " (毫秒)" << endl;// 然后调用
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             eTime,    // 很可能是0
                             COMMAND_TIMESPAN);  // 很可能是0
## 4. 最可能的情况
从日志差值固定为25ms来看，几乎可以肯定：
eTime * 1000 = 0（eTime = 0秒）
COMMAND_TIMESPAN * 1000 = 0（COMMAND_TIMESPAN = 0秒）
所以总延迟只有 additional_delay = 25毫秒
验证方法
方法1：在Tracker中打印参数
cpp
// 在Tracker构造函数或setParam函数中ROS_WARN("eTime: %.6f秒, COMMAND_TIMESPAN: %.6f秒", eTime, COMMAND_TIMESPAN);
方法2：在draw_simulated_bullets中添加调试
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {ROS_WARN("传入参数: eTime=%lld, COMMAND_TIMESPAN=%lld", eTime, COMMAND_TIMESPAN);// ... 其余代码 ...}
方法3：检查配置文件
检查你的YAML配置文件：
yaml
Tracker:eTime: 0.002           # 应该是0.002秒 = 2毫秒COMMAND_TIMESPAN: 0.11  # 应该是0.11秒 = 110毫秒
解决方案
方案1：修复参数传递
如果参数确实是0，修复它们：
cpp
// 在Tracker中确保使用正确的参数
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time, 
                             static_cast<long long>(eTime * 1000),      // 确保转换为毫秒static_cast<long long>(COMMAND_TIMESPAN * 1000));
方案2：修改update_bullet函数接受秒参数
cpp
auto AimCorrector::update_bullet(long long current_time, double eTime_sec, double command_timespan_sec) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 将秒转换为毫秒long long eTime_ms = static_cast<long long>(eTime_sec * 1000);long long command_timespan_ms = static_cast<long long>(command_timespan_sec * 1000);long long additional_delay = 25;  // 0.025秒 = 25毫秒ROS_WARN("延迟计算: eTime_ms=%lld, command_timespan_ms=%lld, 总延迟=%lld",             eTime_ms, command_timespan_ms, eTime_ms + additional_delay + command_timespan_ms);// ... 其余代码 ...}
方案3：最简方案：直接使用毫秒
cpp
auto AimCorrector::update_bullet(long long current_time) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒const long long total_delay = 137;   // 固定总延迟137毫秒（2+25+110）if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {long long fire_time = current_time + total_delay;ROS_WARN("创建子弹: now=%lld, fire=%lld, 延迟=%lldms",                     current_time, fire_time, total_delay);
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
立即测试的方法
要立即看到子弹，最简单的是：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;// 临时：设为0延迟long long fire_time = current_time;  // 立即发射if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, fire_time)});this->last_fire_time = current_time;}}}
总结
根本原因：你的 eTime 和 COMMAND_TIMESPAN 参数很可能是0，所以总延迟只有固定的25ms（来自 additional_delay）。
解决方案：
检查并修复Tracker中传入的参数
或者修改代码让子弹立即发射（用于测试）
或者等待25ms让子弹自然发射
由于25ms的延迟很短，如果你运行程序足够长时间，子弹最终会发射（当 now_time 超过 fire_t 时）。但你的程序可能每帧都在创建新子弹，而每颗新子弹又有25ms延迟，所以看起来子弹永远不发射。
快速测试：将 additional_delay 设为0，就能立即看到子弹了！
[identify-4] killing on exit
[hikcamera-2] killing on exit
eTime_ms: 0
command_timespan_ms: 0
eTime_ms: 0
command_timespan_ms: 0
eTime_ms: 0
command_timespan_ms: 0
eTime_ms: 0
command_timespan_ms: 0
eTime_ms: 0
command_timespan_ms: 0
eTime_ms: 0
command_timespan_ms: 0
[rosout-1] killing on exit
[master] killing on exit
        // 将秒转换为毫秒
        long long eTime_ms = static_cast<long long>(eTime * 1000);
        long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);
        long long additional_delay = 25;  // 0.025秒 = 25毫秒
        cout << "eTime_ms: " << eTime_ms << endl;
        cout << "command_timespan_ms: " << command_timespan_ms << endl;
为什么这两个打印出来是零?
问题：在update_bullet函数中，eTime_ms和command_timespan_ms打印出来都是0。
原因：传入的eTime和COMMAND_TIMESPAN参数本身是0，或者转换过程中出现了问题。
根据日志，打印出来的都是0，说明传入的eTime和COMMAND_TIMESPAN都是0。
我们需要检查在Tracker中调用draw_simulated_bullets时传入的eTime和COMMAND_TIMESPAN参数是否正确。
在Tracker中，这两个参数应该从配置文件中读取，分别对应eTime和COMMAND_TIMESPAN。
检查Tracker的构造函数和setParam函数，确保正确读取了这两个参数。
另外，注意在Tracker的reconstruction_choose_compensation函数中调用draw_simulated_bullets时传入的参数。
可能的原因：
配置文件没有正确设置，导致eTime和COMMAND_TIMESPAN为0。
在Tracker中，没有将这两个参数传递给draw_simulated_bullets。
解决步骤：
步骤1：在Tracker的构造函数中打印这两个参数，确认是否从配置文件中正确读取。
步骤2：在调用draw_simulated_bullets之前打印这两个参数。
步骤3：确保在调用draw_simulated_bullets时传入的是正确的变量。
根据之前Tracker的代码，我们看到在reconstruction_choose_compensation函数中调用draw_simulated_bullets时，传入的是eTime和COMMAND_TIMESPAN。
但是，注意在Tracker类中，这两个成员变量是否已经正确赋值？在setParam函数中，我们从配置文件中读取了这两个值，所以应该检查setParam函数是否正确读取。
如果配置文件中的键名不对，或者类型不对，可能导致读取失败，使用默认值0。
因此，我们需要检查配置文件和setParam函数。
例如，在setParam函数中，我们使用如下代码读取：
COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);
eTime = track["eTime"].as<double>(0.002);
如果配置文件中没有这些键，则会使用默认值（0.11和0.002），但如果我们打印出来是0，说明读取到的就是0。
可能配置文件中的节和键名不匹配。请检查配置文件。
假设配置文件路径正确，并且格式如下：
Tracker:
COMMAND_TIMESPAN: 0.11
eTime: 0.002
那么，请检查在setParam函数中，是否使用了正确的节点路径。
另外，注意在Tracker的构造函数中，我们调用了setParam函数，所以如果配置文件路径不对，也会导致使用默认值0。
我们可以通过打印来确认。
在Tracker的setParam函数中，添加打印：
text
    COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);
    eTime = track["eTime"].as<double>(0.002);
    ROS_INFO("COMMAND_TIMESPAN: %f, eTime: %f", COMMAND_TIMESPAN, eTime);
如果打印出来是0，说明配置文件中没有正确设置，或者节点名称不对。
另外，注意在Tracker类中，这两个变量是double类型，而在调用draw_simulated_bullets时，我们将其转换为long long类型（毫秒）。
如果这两个double类型的值本身是0，那么转换后就是0。
所以，问题很可能出现在配置文件的读取上。
请检查配置文件，确保有以下内容：
Tracker:
COMMAND_TIMESPAN: 0.11
eTime: 0.002
并且注意缩进，YAML文件对缩进敏感。
如果以上都正确，那么可能是配置文件路径不对，导致没有读取到，而是使用了默认值0（注意，我们在as<double>中给出的默认值是0.11和0.002，但如果读取失败，可能会返回0？实际上，如果键不存在，as<double>会返回我们指定的默认值，所以应该是0.11和0.002）。
因此，如果打印出来是0，那说明读取的键存在，但值为0。可能是配置文件中写成了0。
请仔细检查配置文件。
另外，注意在Tracker类中，这两个变量是double类型，我们将其转换为毫秒时，乘以1000，然后转换为long long。
如果eTime=0.002，乘以1000等于2，转换为long long后是2，不为0。
所以，如果打印出0，说明eTime和COMMAND_TIMESPAN本身就是0。
因此，问题在于配置文件读取。
我们可以在Tracker的setParam函数中，打印出整个track节点，看看是否还有其他键值对。
例如：
YAML::Node track = config["Tracker"];
for(YAML::const_iterator it=track.begin();it!=track.end();++it) {
std::cout << it->first.asstd::string() << " = " << it->second.as<double>() << std::endl;
}
这样可以看到Tracker节点下所有的键值对。
如果发现没有COMMAND_TIMESPAN和eTime，那么说明配置文件写错了。
如果发现有，但是值是0，那么配置文件中的值就是0。
根据上述分析，请检查配置文件。
如果配置文件正确，那么可能是YAML文件的格式问题，比如使用了Tab缩进，应该使用空格。
建议使用空格缩进，并且确保缩进正确。
由于时间关系，我们可以先临时修改Tracker的代码，将这两个参数设置为非零值，测试子弹是否能正常发射。
例如，在Tracker的构造函数中，直接赋值：
eTime = 0.002;
COMMAND_TIMESPAN = 0.11;
然后运行，看看是否还有问题。
如果这样修改后，子弹仍然不发射，那么问题可能出在其他地方。
如果修改后子弹发射了，那么问题就是配置文件读取的问题。
请按照上述步骤排查。
很好！现在问题明确了：eTime_ms 和 command_timespan_ms 都是0。这意味着传入的 eTime 和 COMMAND_TIMESPAN 参数都是0。
根本原因
eTime 和 COMMAND_TIMESPAN 在Tracker中都是0，所以计算出的毫秒值也是0。
检查Tracker中的参数
## 1. 检查Tracker构造函数中的参数初始化
cpp
// 在Tracker的构造函数中
cout << "Tracker构造函数 - eTime: " << eTime << endl;
cout << "Tracker构造函数 - COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << endl;
## 2. 检查setParam函数是否正确读取
在你的Tracker的setParam函数中，应该有这样的代码：
cpp
void Tracker::setParam(const std::string &file_path) {// ...if (config["Tracker"]){
        YAML::Node track = config["Tracker"];
        eTime = track["eTime"].as<double>(0.002);  // 默认值0.002
        COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>(0.11);  // 默认值0.11
        cout << "读取参数: eTime=" << eTime << ", COMMAND_TIMESPAN=" << COMMAND_TIMESPAN << endl;}// ...}
## 3. 检查配置文件内容
你的YAML配置文件应该是这样的：
yaml
Tracker:eTime: 0.002             # 曝光时间，单位秒COMMAND_TIMESPAN: 0.11   # 电控延迟，单位秒# 其他参数...
## 4. 检查参数传递路径
在Tracker中，调用draw_simulated_bullets时：
cpp
// 在reconstruction_choose_compensation函数中
cout << "调用前 - eTime: " << eTime << ", COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << endl;
cout << "调用前 - eTime*1000: " << eTime * 1000 << ", COMMAND_TIMESPAN*1000: " << COMMAND_TIMESPAN * 1000 << endl;
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             eTime,           // 这里应该是0.002
                             COMMAND_TIMESPAN); // 这里应该是0.11
可能的解决方案
方案1：修复参数读取
如果配置文件没有问题，可能是读取代码有问题：
cpp
void Tracker::setParam(const std::string &file_path) {try {
        YAML::Node config = YAML::LoadFile(file_path);if (config["Tracker"]) {
            YAML::Node track = config["Tracker"];// 检查键是否存在if (track["eTime"]) {
                eTime = track["eTime"].as<double>();
                cout << "成功读取 eTime: " << eTime << endl;} else {
                eTime = 0.002;  // 默认值
                cout << "使用默认 eTime: " << eTime << endl;}if (track["COMMAND_TIMESPAN"]) {
                COMMAND_TIMESPAN = track["COMMAND_TIMESPAN"].as<double>();
                cout << "成功读取 COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << endl;} else {
                COMMAND_TIMESPAN = 0.11;  // 默认值
                cout << "使用默认 COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << endl;}}} catch (...) {// 错误处理}}
方案2：使用硬编码值测试
暂时绕过配置文件问题：
cpp
// 在Tracker构造函数中Tracker::Tracker(ros::NodeHandle& nh,const std::string& config_path) : config_path_(config_path), tfListener(tfBuffer_) {this->nh = nh;// 硬编码测试值
    eTime = 0.002;          // 2毫秒
    COMMAND_TIMESPAN = 0.11; // 110毫秒
    cout << "硬编码参数: eTime=" << eTime << ", COMMAND_TIMESPAN=" << COMMAND_TIMESPAN << endl;// ... 其余初始化代码}
方案3：修改draw_simulated_bullets调用
如果参数确实是0，可以修改调用方式：
cpp
// 在Tracker的reconstruction_choose_compensation函数中// 使用固定值而不是成员变量double test_eTime = 0.002;double test_COMMAND_TIMESPAN = 0.11;
cout << "使用测试参数: eTime=" << test_eTime << ", COMMAND_TIMESPAN=" << test_COMMAND_TIMESPAN << endl;
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             test_eTime,
                             test_COMMAND_TIMESPAN);
方案4：修改update_bullet函数使用固定延迟
最简单直接的解决方案：
cpp
auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {const long long fire_interval = 50;  // 发射间隔：50毫秒// 忽略传入的参数，使用固定值const long long fixed_eTime_ms = 2;          // 2毫秒const long long fixed_command_timespan_ms = 110;  // 110毫秒const long long additional_delay = 25;       // 25毫秒long long total_delay = fixed_eTime_ms + additional_delay + fixed_command_timespan_ms;
    cout << "使用固定延迟: " << total_delay << "ms (2+25+110)" << endl;if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
            bullets.push_back(IdProj {
                next_id++,ProjectileSimulator(coorConverter, this->shoot_param, current_time + total_delay)});this->last_fire_time = current_time;}}}
快速诊断脚本
在Tracker中添加这个函数来诊断问题：
cpp
void Tracker::debugParams() {
    cout << "=== 参数诊断 ===" << endl;
    cout << "配置文件路径: " << config_path_ << endl;
    cout << "eTime: " << eTime << " (秒)" << endl;
    cout << "eTime * 1000: " << eTime * 1000 << " (毫秒)" << endl;
    cout << "COMMAND_TIMESPAN: " << COMMAND_TIMESPAN << " (秒)" << endl;
    cout << "COMMAND_TIMESPAN * 1000: " << COMMAND_TIMESPAN * 1000 << " (毫秒)" << endl;
    cout << "static_cast<long long>(eTime * 1000): " << static_cast<long long>(eTime * 1000) << endl;
    cout << "static_cast<long long>(COMMAND_TIMESPAN * 1000): " << static_cast<long long>(COMMAND_TIMESPAN * 1000) << endl;
    cout << "=== 诊断结束 ===" << endl;}// 在Track()函数开始处调用void Track() {debugParams();  // 添加这行// ... 其余代码}
最可能的根本原因
配置文件路径错误：config_path_ 可能指向了错误的文件
配置文件格式错误：YAML文件格式不正确
参数名不匹配：配置文件中使用了不同的键名
构造函数调用顺序：在读取配置文件之前使用了参数
推荐立即采用的解决方案：
使用方案4（固定延迟），这样无论Tracker的参数是什么，子弹都会有合理的延迟（2+25+110=137毫秒）。这样你就能立即看到子弹效果，然后再去解决Tracker的参数读取问题。
// 这里写的很简略,只能看静止弹道对不对
    // 每隔一段时间就放一颗弹丸,假想一个发弹时间固定的模拟器
    const std::size_t AIM_CORRECTOR_BULLETS_MAX_SZ = 200u;
    auto AimCorrector::update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void {
        const long long fire_interval = 50;  // 发射间隔：50毫秒
        if (this->last_fire_time == 0 || (current_time - this->last_fire_time) >= fire_interval) {
            if (bullets.size() + 1u <= AIM_CORRECTOR_BULLETS_MAX_SZ) {
                bullets.push_back(IdProj {
                    next_id++,
                    ProjectileSimulator(coorConverter, this->shoot_param, current_time + eTime + 25 + COMMAND_TIMESPAN)
                });
                this->last_fire_time = current_time;
            }
        }
    }
我发现这里面始终只有一颗子弹也就是说这个bullets变量没有维护,程序每跑到这里就创建一个全新的
我们看到问题：每次调用draw_simulated_bullets时，都会创建一个新的AimCorrector对象，然后调用update_bullet。由于AimCorrector对象是局部变量，所以每次都会重新开始，子弹列表bullets也是空的，然后创建一颗子弹，但是这颗子弹的发射时间设置在未来（当前时间+延迟），所以在同一帧的get_circles调用中，子弹还未发射，因此不会被绘制。然后这个AimCorrector对象就被销毁了。下一帧又是同样的过程。
换句话说，子弹列表没有被持久化，每一帧都是一个新的AimCorrector，所以每一帧都只有一颗新创建的子弹，而且这颗子弹还未发射。
为了解决这个问题，我们需要将AimCorrector对象作为持久化的对象，而不是每次调用draw_simulated_bullets都新建一个。
建议修改：将AimCorrector作为Tracker类的成员变量，这样它就可以持续维护子弹列表。
具体步骤：
在Tracker类中添加一个AimCorrector成员变量，例如aim_corrector。
在Tracker的构造函数中初始化aim_corrector，但注意需要传入coorConverter和shoot_param，而shoot_param是每次计算都会变化的，所以不能只初始化一次。
或者，我们可以在每次调用draw_simulated_bullets时，使用同一个AimCorrector对象，但更新其内部的shoot_param。
但是，注意AimCorrector的构造函数需要coorConverter和shoot_param，而shoot_param是随着目标变化而变化的。所以我们可以这样做：
修改AimCorrector类，使其可以更新shoot_param，并且update_bullet函数不再创建新的子弹模拟器，而是根据当前的shoot_param和当前时间创建子弹。
但是，我们也可以不修改AimCorrector，而是在Tracker中维护一个AimCorrector对象，并在每次调用draw_simulated_bullets时，传入当前的shoot_param，并让AimCorrector更新这个参数。
由于AimCorrector现在需要更新子弹，而且子弹的创建依赖于shoot_param，我们可以这样做：
修改AimCorrector，添加一个成员函数来更新shoot_param，然后在update_bullet中使用最新的shoot_param。
但是，考虑到我们每次调用draw_simulated_bullets时，shoot_param可能不同，所以每次都应该用最新的shoot_param来创建子弹。
然而，子弹的创建是时间触发的（每50ms），所以我们需要在AimCorrector内部维护一个shoot_param，并允许外部更新它。
修改AimCorrector类：
添加一个成员函数update_shoot_param，用于更新shoot_param。
在update_bullet中，使用当前存储的shoot_param来创建子弹。
同时，我们需要在Tracker中创建一个持久的AimCorrector对象，并在每次调用draw_simulated_bullets时，先更新aim_corrector的shoot_param，然后调用update_bullet和get_circles。
但是，注意AimCorrector的构造函数需要coorConverter，这个在Tracker中已经存在，可以传入。
具体代码修改：
在Tracker类中添加成员变量：
cpp
std::unique_ptr<tools::AimCorrector> aim_corrector_;
在Tracker的构造函数中初始化：
cpp
aim_corrector_ = std::make_unique<tools::AimCorrector>(coorConverter, tools::ShootParam());
在reconstruction_choose_compensation函数中，设置shoot_param并更新子弹：
cpp
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);// 更新aim_corrector_的shoot_param
aim_corrector_->update_shoot_param(shoot_param);// 更新子弹
aim_corrector_->update_bullet(draw_visual_now_time, eTime, COMMAND_TIMESPAN);// 绘制子弹
tools::draw_simulated_bullets_with_corrector(aim_corrector_.get(), frame_, draw_visual_now_time);
注意，我们需要修改draw_simulated_bullets函数，改为接受一个AimCorrector指针，而不是每次都新建一个。或者，我们直接使用aim_corrector_来获取子弹并绘制，而不通过draw_simulated_bullets。
为了最小改动，我们可以修改draw_simulated_bullets，使其接受一个AimCorrector对象，并在内部调用它的get_circles。
但是，我们也可以不修改draw_simulated_bullets，而是让Tracker直接调用aim_corrector_->get_circles，然后绘制。
考虑到代码结构，我们可能希望将绘制函数与AimCorrector分离，所以我们可以保留draw_simulated_bullets，但修改其参数，接受一个AimCorrector对象。
修改draw_simulated_bullets函数：
cpp
void draw_simulated_bullets(AimCorrector* aim_corrector, cv::Mat& img, long long now_time);
这样，在Tracker中就可以这样调用：
cpp
tools::draw_simulated_bullets(aim_corrector_.get(), frame_, draw_visual_now_time);
但是，原来的draw_simulated_bullets函数中还有创建AimCorrector和更新子弹的逻辑，现在这些逻辑应该移到外面。
因此，我们修改draw_simulated_bullets函数，只负责绘制子弹，而子弹的更新由外部完成。
总结修改步骤：
修改AimCorrector类，添加update_shoot_param函数，并修改update_bullet函数，使用成员变量shoot_param。
在Tracker中添加AimCorrector成员变量，并在构造函数中初始化。
在Tracker的reconstruction_choose_compensation函数中，更新aim_corrector_的shoot_param，并调用update_bullet。
修改draw_simulated_bullets函数，接受AimCorrector指针和当前时间，从中获取子弹并绘制。
由于改动较大，我们一步步来。
首先，修改AimCorrector类（头文件）：
cpp
class AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param);// 更新射击参数void update_shoot_param(const ShootParam& shoot_param);// 获取所有已经发射但尚未"击中"的子弹在当前时刻的图像投影圆auto get_circles(long long now_time) -> std::vector<IdCircle>;auto update_bullet(long long current_time, long long eTime, long long COMMAND_TIMESPAN) -> void;private:
    std::list<IdProj> bullets;             // 活跃子弹容器模拟器          
    CoordinateTransformer* coorConverter;  // 坐标变换器
    ShootParam shoot_param;                // 当前射击参数long long next_id = 0;           
    long long last_fire_time = 0;};
实现update_shoot_param：
cpp
void AimCorrector::update_shoot_param(const ShootParam& new_shoot_param) {this->shoot_param = new_shoot_param;}
然后，修改draw_simulated_bullets函数，使其接受一个AimCorrector指针：
cpp
void draw_simulated_bullets(AimCorrector* aim_corrector, cv::Mat& img, long long now_time);
实现：
cpp
void draw_simulated_bullets(AimCorrector* aim_corrector, cv::Mat& img, long long now_time) {
    FlaskStream flask_aim;
    flask_aim.clear();// 从aim_corrector获取子弹
    std::vector<IdCircle> bullets = aim_corrector->get_circles(now_time);for (auto& bullet: bullets) {
        flask_aim << FlaskPoint(
            bullet.circle.center,{ 0, 0, 255 },
            bullet.circle.r,2);
        flask_aim << FlaskText(
            std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
    flask_aim >> img;}
在Tracker中，添加成员变量并初始化：
cpp
class Tracker {private:
    std::unique_ptr<tools::AimCorrector> aim_corrector_;// ...};Tracker::Tracker(ros::NodeHandle& nh,const std::string& config_path) : config_path_(config_path), tfListener(tfBuffer_) {// ... 其他初始化// 初始化aim_corrector_，初始shoot_param可以随意，因为后面会更新
    aim_corrector_ = std::make_unique<tools::AimCorrector>(coorConverter, tools::ShootParam());}
在reconstruction_choose_compensation函数中，更新aim_corrector_并绘制：
cpp
// 计算shoot_param
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);// 更新aim_corrector_的射击参数
aim_corrector_->update_shoot_param(shoot_param);// 更新子弹（注意：这里传入的eTime和COMMAND_TIMESPAN需要是毫秒，但你的函数现在接受的是long long，且已经乘以1000，所以这里直接传入毫秒值）
aim_corrector_->update_bullet(draw_visual_now_time, 
                              static_cast<long long>(eTime * 1000), 
                              static_cast<long long>(COMMAND_TIMESPAN * 1000));// 绘制子弹
tools::draw_simulated_bullets(aim_corrector_.get(), frame_, draw_visual_now_time);
这样，子弹列表就会在aim_corrector_中持续维护，而不是每次调用都新建。
注意：原来的draw_simulated_bullets函数有多个参数，现在改为接受AimCorrector指针，所以需要修改调用方式。
另外，由于AimCorrector的update_bullet函数中，子弹的创建依赖于传入的延迟参数（eTime, COMMAND_TIMESPAN）和当前的shoot_param，所以每次更新射击参数后，新创建的子弹都会使用最新的参数。
这样修改后，子弹就会在多次调用中持续存在，并且会按照设定的间隔（50ms）创建新的子弹，直到达到最大数量。
注意：原来的draw_simulated_bullets函数中还有eTime和COMMAND_TIMESPAN参数，现在这些参数在Tracker中调用update_bullet时传入，所以不再需要。
由于改动较大，请确保在修改后编译通过，并测试功能。
如果不想大幅改动，也可以将AimCorrector对象作为静态局部变量放在draw_simulated_bullets函数内部，这样也可以保持子弹列表的持久化，但这样不利于多目标等情况。考虑到自瞄系统通常只有一个目标，所以也可以使用静态局部变量。
但是，使用静态局部变量会使得函数不可重入，并且如果同时有多个目标需要绘制弹道，就会有问题。因此，建议使用成员变量的方式。
根据你的实际情况选择。如果你希望简单修改，可以使用静态局部变量，但这样可能会带来其他问题。
这里给出使用静态局部变量的修改方法（不推荐，但快速）：
修改draw_simulated_bullets函数：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter, const ShootParam& shoot_param, cv::Mat& img, long long now_time, long long eTime, long long COMMAND_TIMESPAN) {
    FlaskStream flask_aim;
    flask_aim.clear();static AimCorrector aim_corrector(coorConverter, shoot_param); // 静态局部变量// 更新射击参数
    aim_corrector.update_shoot_param(shoot_param);// 更新子弹序列
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);for (auto& bullet: bullets) {
        flask_aim << FlaskPoint(
            bullet.circle.center,{ 0, 0, 255 },
            bullet.circle.r,2);
        flask_aim << FlaskText(
            std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
    flask_aim >> img;}
这样，aim_corrector在第一次调用时被初始化，之后每次调用都使用同一个对象，从而维护了子弹列表。
注意：静态局部变量只初始化一次，所以第一次调用时传入的coorConverter和shoot_param被用于初始化，后续调用update_shoot_param来更新射击参数。
但是，这种方法有一个问题：如果coorConverter指针发生变化（比如指向了另一个对象），那么后续的调用就会使用错误的coorConverter。所以，请确保coorConverter在程序运行期间是有效的。
根据你的代码，coorConverter在Tracker中是一个成员变量，并且在Tracker的生命周期内是有效的，所以可以使用。
两种方法都可以，但成员变量方法更面向对象，更安全。静态局部变量方法简单，但需要注意上述问题。
请根据你的情况选择修改方式。
你说得非常对！这就是核心问题所在。
问题分析
每次调用 draw_simulated_bullets 时，都会创建一个全新的 AimCorrector 对象：
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {
    AimCorrector aim_corrector = AimCorrector(coorConverter, shoot_param);  // 每次都新建！
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);// ...}
这意味着：
第1帧：创建 AimCorrector 对象 A，添加子弹1，检查子弹（未发射），子弹列表 = [子弹1]
第2帧：创建 新的 AimCorrector 对象 B，添加子弹2，检查子弹（未发射），子弹列表 = [子弹2]
第3帧：创建 新的 AimCorrector 对象 C，添加子弹3，检查子弹（未发射），子弹列表 = [子弹3]
每个 AimCorrector 对象都是独立的，它们之间不共享子弹列表！
解决方案
方案1：将 AimCorrector 作为静态变量（最简单）
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {static AimCorrector aim_corrector(coorConverter, shoot_param);  // 静态变量，只初始化一次！
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);// ...}
优点：最简单，只需要改一个关键字 static
缺点：只有一个全局的子弹列表，如果多次调用可能会有问题
方案2：将 AimCorrector 作为 Tracker 的成员变量（推荐）
cpp
// 在Tracker.h中class Tracker {private:// 添加成员变量
    tools::AimCorrector* aim_corrector_;// ...};
cpp
// 在Tracker.cpp的构造函数中Tracker::Tracker(ros::NodeHandle& nh,const std::string& config_path) : 
    config_path_(config_path), tfListener(tfBuffer_) {this->nh = nh;// 初始化aim_corrector
    aim_corrector_ = new tools::AimCorrector(coorConverter, tools::ShootParam());// ...}
cpp
// 在Tracker的析构函数中Tracker::~Tracker() {delete cal;delete coorConverter;delete aim_corrector_;  // 释放内存}
cpp
// 在reconstruction_choose_compensation函数中
tools::ShootParam shoot_param;
shoot_param.v0 = BulletVector;
shoot_param.aim_angle = m_target_pose.pitch + m_pitchOffset;
shoot_param.target_xyz_i_camera = coorConverter->map2Cam(abs_target_armor.position);// 更新aim_corrector的射击参数
aim_corrector_->update_shoot_param(shoot_param);  // 需要添加这个方法// 更新子弹
aim_corrector_->update_bullet(draw_visual_now_time, 
                             static_cast<long long>(eTime * 1000),static_cast<long long>(COMMAND_TIMESPAN * 1000));// 获取并绘制子弹
std::vector<tools::IdCircle> bullets = aim_corrector_->get_circles(draw_visual_now_time);// 绘制代码...
方案3：修改 AimCorrector 使其可更新参数
首先在 AimCorrector 类中添加更新参数的方法：
cpp
// 在trajectory_visualizer.hpp中class AimCorrector {public:// ... 其他方法 ...// 添加这个方法void update_shoot_param(const ShootParam& new_param) {
        shoot_param = new_param;}// ...};
方案4：修改 draw_simulated_bullets 接收 AimCorrector 引用
cpp
// 修改函数签名void draw_simulated_bullets(AimCorrector& aim_corrector,  // 改为引用const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {// 更新射击参数
    aim_corrector.update_shoot_param(shoot_param);
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);// ...}
完整推荐方案（方案1 + 方案3）
这是最简单有效的解决方案：
步骤1：在 AimCorrector 中添加参数更新方法
cpp
// trajectory_visualizer.hppclass AimCorrector {public:AimCorrector(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param);// 更新射击参数void update_shoot_param(const ShootParam& new_param) {
        shoot_param = new_param;}// ... 其他方法 ...};
步骤2：修改 draw_simulated_bullets 使用静态变量
cpp
void draw_simulated_bullets(CoordinateTransformer* const coorConverter,const ShootParam& shoot_param,
                           cv::Mat& img,long long now_time, 
                           long long eTime, 
                           long long COMMAND_TIMESPAN) {
    FlaskStream flask_aim;
    flask_aim.clear();// 使用静态变量，只初始化一次static AimCorrector aim_corrector(coorConverter, shoot_param);// 更新射击参数（因为shoot_param可能每帧都变化）
    aim_corrector.update_shoot_param(shoot_param);// 更新子弹序列
    aim_corrector.update_bullet(now_time, eTime, COMMAND_TIMESPAN);
    std::vector<IdCircle> bullets = aim_corrector.get_circles(now_time);// 绘制子弹for (auto& bullet: bullets) {
        flask_aim << FlaskPoint(
            bullet.circle.center,{ 0, 0, 255 },
            bullet.circle.r,2);
        flask_aim << FlaskText(
            std::to_string(bullet.id),{ bullet.circle.center.x + 20.f, bullet.circle.center.y },{ 0, 0, 255 },0.8);}
    flask_aim >> img;}
步骤3：在Tracker中修复参数传递问题
cpp
// 在Tracker的reconstruction_choose_compensation函数中// 确保传入正确的参数（毫秒）long long eTime_ms = static_cast<long long>(eTime * 1000);  // 应该是2long long command_timespan_ms = static_cast<long long>(COMMAND_TIMESPAN * 1000);  // 应该是110// 如果参数为0，使用固定值if (eTime_ms == 0) eTime_ms = 2;if (command_timespan_ms == 0) command_timespan_ms = 110;
tools::draw_simulated_bullets(this->coorConverter, shoot_param, frame_,                             draw_visual_now_time,                             eTime_ms,           // 传入毫秒
                             command_timespan_ms);  // 传入毫秒
关键修复点
使用 static 关键字：确保 AimCorrector 对象只创建一次
添加参数更新方法：因为每帧的射击参数可能不同
修复参数传递：确保传入正确的毫秒值
这样修改后，子弹列表会被持续维护，子弹会按照50ms的间隔持续添加到列表中，并且会随着时间推移逐渐发射。

