# 板球控制系统

本项目的目的是复现电赛题目的板球控制系统

[系统架构设计](/documentation/板球控制系统.pdf)

# Todo-List

## 硬件部分
### 机械结构

## 软件部分
### 球运动控制
#### pid算法部分
- [x] pid算法的学习了解
- [x] 使用MATLAB仿真
  - [x] 建模仿真
  - [x] pid算法仿真
- [ ] pid算法在硬件平台的实现
  - [x] 实现舵机、传感器驱动正常工作
  - [x] 对单个舵机验证性实验
  - [ ] 应用算法到实际项目中
- [ ] pid算法在实际情况的验证

### 球位置识别

# PR要求
+ 开发时新建分支用于开发
+ 从开发所用分支进行PR
+ 尽量少的commit

# 代码规范
[参考rtt代码规范](https://github.com/RT-Thread/rt-thread/blob/master/documentation/coding_style_cn.md)