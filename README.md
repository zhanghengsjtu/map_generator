## 使用说明

### 1. 安装ros和moveit

步骤略

### 2. 编译工作空间

```
cd map_generartor
catkin_make
```

PS1: 编译过程中如果缺少对应的ros包，可以根据提示直接安装即可。

PS2: 本测试环境为Ubuntu16 + ros kinetic，在ros melodic和noetic版本中，可能会提示找不到fcl对应的头文件。原因是moveit_core在kinetic之后发生了变化，可以选择手动安装moveit。

### 3. 生成地图

每个命令运行之前，记得在map_generator目录下面运行`source devel/setup.bash`。

1. 启动仿真环境

```
roslaunch agv_moveit_config demo.launch
```

2. 生成环境地图

```
# 第一个参数为1时，代表生成地图，为0时，代表不生成地图
# 第二个参数为1时，代表加载地图，为0时，代表不加载地图
# 第三个参数代表生成的地图文件，目前使用yaml文件保存

# 生成地图同时加载地图
# 其他地图参数参考源文件
rosrun mm_local_motion_planning calculateCollisionMap 1 1 map.yaml 

# 加载已经存在的地图
rosrun mm_local_motion_planning calculateCollisionMap 0 1 map.yaml
```

PS: 地图对应的话题名称为`gridMap`，可以使用`remap`特性进行修改，或者修改`agv_collion_map.cpp`中的`publishMap`函数


3. 保存ROS格式地图

```
rosrun map_server map_saver -f test_map map:=gridMap
```

### 4. 配置过程

1. 核心原理是利用fcl提供的函数计算`agv`和`env`之间的距离，根据实际距离与设置的阈值比较，来确定对应的位姿是否是障碍物；
2. 代码里面有一些定制化的代码，比如在urdf中，`agv`的连杆名称必须带`agv_base_link`(参考`agv_distance_detection.cpp`的17行)，环境的模型必须带`env`(参考参考`agv_distance_detection.cpp`的31行)，即带`agv_base_link`的连杆集合和带`env`的连杆集合做碰撞检测；
3. 移动机器人的规划组需要与`calculateCollisionMap.cpp`的参数保持一致；

