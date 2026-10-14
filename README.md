# 学习过程中的仿真

记录《迭代学习控制》学习过程中的 MATLAB 仿真，按章节整理。

## 第一章 1.3：直流电机 D 型学习律

新增 [第一章仿真与代码说明](chapter1/README.md)，包括可直接运行的 MATLAB 脚本和已生成的结果图。

```matlab
run('chapter1/ilc_case_1_3.m')
```

模型为 `y_dot + y = u`，期望轨迹为 `y_d(t)=t`，学习律为 `u_{k+1}=u_k+e_dot_k/1.25`。第 9 次最大跟踪误差约为 `4.888466e-04`，可复现书中图 1.3.2 的逼近效果。
## 第二章 2.2：PID 型学习律

[第二章仿真与运行说明](chapter2/README.md)，包含 MATLAB 脚本、输出跟踪与误差收敛图、输入学习曲线。

在 MATLAB 中进入 `chapter2` 文件夹后运行 `ilc_pid_validation`。示例满足 `rho(I-C*B*Gamma)=0.4<1`；80 次迭代后最大跟踪误差约为 `4.960881e-04`。
