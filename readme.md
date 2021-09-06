
## 文件结构

 - roshand_gen2_demo : roshand gen2控制示例程序包
 - roshand_gen2_driver : roshand gen2的驱动包
 - roshand_gen2_msgs :  messages, servers and actionlib 在这里定义.
 - roshand_gen2_visualizer : roshand gen2的URDF文件. 
 

## 安装

# 按照如下步骤安装、编译 :

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/verma-robot/RosHand.git scholar-ros
  cd ~/catkin_ws
  catkin_make

  ```
# 修改Scholar E600 串口权限
  copy the udev rule file `scholar_600.rules` from `~/catkin_ws/src/scholar-ros/scholar_bringup/udev` to `/etc/udev/rules.d/`:

  ```
  cd ~/catkin_ws/src/scholar-ros
  sudo cp scholar_bringup/udev/scholar_600.rules /etc/udev/rules.d/
  
  ```
#修改Scholar E600 内置IMU的串口权限
  copy the udev rule file `imu.rules` from `~/catkin_ws/src/scholar-ros/scholar_imu/udev` to `/etc/udev/rules.d/`:

  ```
  cd ~/catkin_ws/src/scholar-ros
  sudo cp scholar_imu/udev/imu.rules /etc/udev/rules.d/
  
  ```
#### 控制机器人（非导航模式）

###  with real robot

## 启动机器人

#启动Scholar E600

  ```
  roslaunch scholar_bringup bringup.launch 

  ```
#启动Scholar E600内置IMU

  ```
  roslaunch scholar_imu bringup.launch 

  ```
##键盘控制机器人

  ```
  roslaunch scholar_joy_teleop scholar_teleop.launch 

  ```

##校准机器人线速度、角速度

#校准线速度：

  ```
  rosrun scholar_calibrate calibrate_linear.py 

  ```

#校准角速度：

  ```
  rosrun scholar_calibrate angular_calibrate.py 

  ```

###  without real robot(Gazebo)

##启动机器人（Gazebo）

  ```
  roslaunch scholar_gazebo scholar_gazebo.launch

  ```

### rviz

  ```

   roslaunch scholar_rviz view_robot.launch 

  ```

#### SLAM

###启动机器人、内置IMU、激光雷达传感器

##  with real robot
  
  ```

   roslaunch scholar_navigation bringup.launch 

  ```
##  without real robot(Gazebo)

  ```
  roslaunch scholar_gazebo scholar_gazebo.launch

  ```

###建图

  ```
  roslaunch scholar_navigation gmapping.launch

  ```
  而后通过键盘控制机器人缓慢移动，建图结束后保存地图

  ```
  cd ~/catkin_ws/src/scholar_ros/scholar_navigation/map
  rosrun map_server map_saver

  ```
###自动导航

  ```
  roslaunch scholar_navigation move_base_demo.launch

  ```
###rviz可视化

  ```
  roslaunch scholar_rviz view_slam.launch

  ```

## Report a Bug
  Any bugs, issues or suggestions may be sent to support@verma-robot.com

  Thanks!
