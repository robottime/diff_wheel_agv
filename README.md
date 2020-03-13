# diff_wheel_agv
two wheel differential agv control、lidar slam、lidar navigation、QR code detection、 QR code location、QR code waypoint move    

## agv_bringup   
启动小车和rviz软件并使用键盘控制小车移动   
```
  roslaunch agv_bringup agv_robot.launch  rviz:=true
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
## agv_slam
构建地图并保存     
```
  roslaunch agv_bringup agv_robot.launch  rviz:=true
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  roslaunch agv_slam agv_slam.launch
```
使用键盘遥控器遥控小车在整个环境中往返移动，直至生成的地图完整、准确，保存地图到导航包路径下   
```
  cd **/agv_navigation/maps
  rosrun map_server map_saver -f ./map
```
## agv_navigation
```
  roslaunch agv_bringup agv_robot.launch  rviz:=true
  rosrun teleop_twist_keyboard teleop_twist_keyboard.py
  roslaunch agv_navigation agv_navigation.launch
```
设置“Fixed Frame”为Map
使用rviz工具栏提供的"2D Pose Estimate"工具指定小车当前位置与方向
如果amcl pose array(红色小箭头)不集中，适当的旋转或移动小车使其集中
使用rviz工具栏提供的"2D Nav Goal"工具指定小车目标位置与方向

***错误解决***   
请确保电脑内装有以下几个Ros包：    
1. ros-melodic-gmapping   
2. ros-melodic-move-base    
3. ros-melodic-amcl   
4. ros-melodic-navigation 

## agv_camera
agv底盘车底部中心位置装有一枚向下的usb摄像头用以拍摄地面上apriltag二维码，该程序包可以单独启动，在该项目中用在“agv_navigation_qr”中 
### 图片
## agv_navigation_qr
使用二维码定位，控制小车按固定路线行驶，路线定义在”agv_navigation_qr/scripts/autonav.py“Line 35,如下图所示 
### 图片
### 视频
***错误解决***   
not find qr_locate,该程序包位于https://github.com/MapleHan/qr_locate  


