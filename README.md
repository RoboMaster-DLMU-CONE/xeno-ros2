# XenoCar ROS2 Package

## 项目简介

本项目用于大连民族大学C·ONE战队2024年工程机器人。其依赖ROS2 Humble / Jazzy，在运行`Ubuntu >= 22.04`的妙算2-G平台上进行运动规划并驱动机械臂进行运动。

下图详细描述了该工程的代码结构。

本项目使用了自行开发的Linux通用驱动[Linux-Motor-Drivers](https://gitee.com/dlmu-cone/rm-linux-motor-driver)进行电机读写操作。

## 配置引导

### 安装前置组件

- [安装ROS2](https://mirrors.tuna.tsinghua.edu.cn/help/ros2/)

```shell
#按链接添加source
#Ubuntu 24.04
sudo apt install ros-jazzy-desktop
#Ubuntu 22.04
# sudo apt install ros-humble-desktop
```

- [安装并初始化rosdep](https://mirrors.tuna.tsinghua.edu.cn/help/rosdistro/)

```shell
#rosdep 包：
sudo apt install python3-rosdep2
#按链接进行rosdep init & update
```

- 在仓库下使用`rosdep`检测并下载依赖

```shell
# git clone https://gitee.com/dlmu-cone/xeno-ros2.git
# cd xeno-ros2
rosdep install --from-paths src --ignore-src -r -y
```

### 编译

编译之前请确保您已位于工作目录的`根目录`下。

```shell
colcon build --symlink-install
```

### 测试

在运行之前，请先编译并使用下列命令配置终端环境

```shell
source install/setup.zsh
#or setup.bash or setup.sh
```

首先启动`ros2_control`节点。

```shell
ros2 launch xeno_control moveit_control.gui.launch.py
```

接着新建终端标签页，运行`moveit`运动规划节点。

```shell
ros2 launch xeno_moveit demo.launch.py
```

在弹出的rviz视图中，你可以使用虚拟参数进行模拟运动规划。

- 还可以启动`xeno_simple_moveit`节点，从外部节点执行`moveit`运动规划。

```shell
ros2 run xeno_simple_move xeno_simple_move
```

- 还可以使用键盘控制各关节状态。键盘发布的各组`position`也会被`xeno_control`的`XenoHardware`类通过位置PID输出反馈到电机上。

```shell
ros2 launch xeno_control keyboard_control.gui.launch.py
```

```shell
ros2 run xeno_keyboard xeno_keyboard_node
```

- 键盘操作指南：
  - 方向键`上`/`下`/`左`/`右`（慢速）、 小键盘区`8`/`2`/`4`/`6`（快速）对应伸出关节的`前进`/`后退`和横移关节的`左移`/`右移`；
  - `[`/`]`（慢速） 、`{`/`}`（快速） 对应升降关节的`上升`和`下降`；
  - `z`/`c`、`a`/`d`、`q`/`e`、分别对应小臂上的三个电机关节的顺/逆时针旋转。小写为慢速，大写为快速。

实机运行时不需要图形化界面，启动无GUI的`ros2_control`和`move_group`即可。

```shell
ros2 launch xeno_control moveit_control.launch.py
```

```shell
ros2 launch xeno_moveit move_group.launch.py
```

## 开发指南

## TODO

- [ ] 接入Linux-Motor-Drivers
- [ ] 实机调参
