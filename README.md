# RobotArm305运动学运动学仿真及控制代码实现

基于2023工创赛高工305物流车机械臂

### 仿真环境配置
###### 安装系统

微软商店下载即可

###### 安装图形桌面

打开Windows功能，打开“适用于Linux的Windows子系统”和“虚拟机平台”

打开终端，依次输入以下命令：
```powershell
wsl --list --online
wsl --install -d <安装的版本全称>
```

以下在Ubuntu界面输入：
```bash
sudo apt-get update
```

回到终端输入：
```powershell
wsl -l -v
wsl --update
```

以管理员身份打开终端：
```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart

dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

在Ubuntu中输入:
```bash
lsb_release -a
sudo apt update && sudo apt -y upgrade
sudo apt install xrdp
sudo apt install -y xfce4
calc
exit
sudo apt install -y xfce4-goodies
sudo vi /etc/xrdp/startwm.sh
service xrdp status
sudo /etc/init.d/xrdp start
ip a 
sudo apt-get install xfce4-terminal
```
之后使用Windows自带的远程桌面即可访问

### 仿真模型文件获取

**1.sw模型基准面修正；**

参考教程：

【新手必会技巧！solidworks零件视图歪了？三种操作教你如何摆正！】 https://www.bilibili.com/video/BV1oP411y7b1/?share_source=copy_web&vd_source=a60cef1f9c05dd5a70c97ee1ae8edf98；

**2.sw转urdf；**

参考教程：

【SolidWorks模型导出urdf   （古月居老师）】 https://www.bilibili.com/video/BV1Tx411o7rH/?share_source=copy_web&vd_source=a60cef1f9c05dd5a70c97ee1ae8edf98；

### 运动学正逆解

**确定DH参数（即$\alpha_i,~a_i,~\theta_i,~d_i$）**
机械臂模型及坐标系如下标识：
<img src="./assets/Pasted image 20250212044818.png" alt="Pasted image 20250212044818.png" style="zoom: 33%;" />

正向转动：按右手螺旋定则
$\alpha_i$ 为绕$\hat{X}_i$ 轴从$\hat{Z}_i$ 到$\hat{Z}_{i+1}$ 的转角，绕$\hat{X}_i$ 正向转动方向为正；
$a_i$ 为沿$\hat{X}_i$ 轴从$\hat{Z}_i$ 到$\hat{Z}_{i+1}$ 的距离，与$\hat{X}_i$ 方向相同为正；
$\theta_i$ 为绕$\hat{Z}_{i}$ 轴（同时也是$\hat{Z}_{i+1}$ 轴）从$\hat{X}_{i}$ 到$\hat{X}_{i+1}$ 的转角，绕$\hat{Z}_{i}$ 正向转动方向为正；
$d_i$ 为沿$\hat{Z}_{i}$ 轴（同时也是$\hat{Z}_{i+1}$ 轴）从$\hat{X}_{i}$ 到$\hat{X}_{i+1}$ 的距离，与$\hat{Z}_{i}$ 方向相同为正；

也可以理解为，按照上述四个步骤旋转、移动、旋转、移动，从上一个坐标系变换到下一个坐标系的过程中用到的四个参数；
改进D-H法的关键在于保证前后两个坐标系的Z轴是严格对齐的，方便后续的计算；
这里需要注意的是$\theta_i$ 是会随着机械臂的运动而变化的，其实也是机械臂运动过程关节电机的旋转角度，全为0时为初始状态；

距离单位：mm，角度单位：rad
D-H表如下：

|   name   |  i  |  $\alpha_i$   | $a_i$  | $\theta_i$ | $d_i$ |
| :------: | :-: | :-----------: | :----: | :--------: | :---: |
|   base   |  1  |       0       |   0    | $\theta_1$ | 85.25 |
| shoulder |  2  | $\pi \over 2$ |   0    | $\theta_2$ | 48.1  |
|  elbow   |  3  |       0       | 128.93 | $\theta_3$ |   0   |
|  wrist   |  4  |       0       |  129   | $\theta_4$ | 48.55 |

其中
$$
\begin{align}
\theta_1&=\Delta\theta_1 \\
\theta_2&={\pi\over2}+\Delta\theta_2 \\
\theta_3&=\Delta\theta_3 \\
\theta_2&=-{\pi\over2}+\Delta\theta_2 \\
\end{align}
$$
即坐标计算时，相对于电机目标旋转角度有一个初始值；

**计算D-H变换矩阵**
根据D-H参数计算D-H变换矩阵，其计算公式如下：
$$
\begin{align}
T_i^{i-1} &= Rx​(αi​)⋅Dx​(ai​)⋅Rz​(θi​)⋅Dz​(di​) \\ 
&= \begin{bmatrix}
\cos\theta_i & -\sin\theta_i & 0 & a_i \\
\sin\theta_i\cos\alpha_i & \cos\theta_i\cos\alpha_i & -\sin\alpha_i & -d_i\sin\alpha_i \\
\sin\theta_i\sin\alpha_i & \cos\theta_i\sin\alpha_i & \cos\alpha_i & d_i\cos\alpha_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\end{align}
$$
由两次旋转和两次平移构成；

该机械臂为4自由度的机械臂，因此四个变换矩阵即可完成机械臂的正运动学变换；

最终得到如下变换矩阵：
$$
T^0_4 = \begin{bmatrix}
r_{11} & r_{12} & r_{13} & p_x \\
r_{21} & r_{22} & r_{23} & p_y \\
r_{31} & r_{32} & r_{33} & p_z \\
0 & 0 & 0 & 1 \\
\end{bmatrix}
$$
其中 $(p_x, p_y, p_z)$ 即末端执行器在基座坐标系中的坐标，同时将旋转矩阵转化成欧拉角就能得到末端执行器相对于基座坐标系的姿态；

