
## 文件结构

 - roshand_gen2_demo : roshand gen2控制示例程序包
 - roshand_gen2_driver : roshand gen2的驱动包
 - roshand_gen2_msgs :  messages, servers and actionlib 在这里定义.
 - roshand_gen2_visualizer : roshand gen2的URDF文件. 
 

## 安装

# 按照如下步骤安装、编译 :

  ```
  cd ~/catkin_ws/src
  git clone https://github.com/verma-robot/Roshand-Gen2.git scholar-ros
  cd ~/catkin_ws
  catkin_make

  ```
# 修改 串口权限

  ```
  sudo chmod 666 /dev/ttyUSB0  
  ```

## 启动机器人手抓

  ```
  roslaunch roshand_gen2_driver bringup.launch 

  ```
##
