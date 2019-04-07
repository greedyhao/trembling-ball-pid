# MATLAB仿真相关代码

+ motor_model_sys_id.slx 使用系统识别的方式建模
+ motor_model_linear.slx 使用线性化的方式建模

该目录下的这两个文件是两种建模的方式，使用前需要初始化模型变量

相关博客可以[参考这里](http://greedyhao.cc/2019/04/06/pid%E5%BB%BA%E7%AB%8B%E6%A8%A1%E5%9E%8B/)

```
J = 0.01;
b = 0.1;
K = 0.01;
R = 1;
L = 0.5;
```

+ motor_model_pid.slx pid自整定

这个文件是使用MATLAB自整定pid后的模型

相关博客可以[参考这里](http://greedyhao.cc/2019/04/12/%E8%87%AA%E5%8A%A8%E6%8E%A7%E5%88%B6-PID-2019-04-12-pid%E8%B0%83%E8%8A%82%E7%9A%84%E6%96%B9%E6%B3%95/)
