# AGV Car Bringup    
该文件夹是实体车的驱动程序文件夹，包含了"小车移动控制与里程信息发布"程序和"电机驱动接口"程序，程序使用python2编写。      
其中小车控制器与电机采用“USB转485“总线通信，每个电机分别地址为1、2电机最高速度为1500RPM。    
该文件夹下还包含一个"rviz.launch"文件，用来查看在Rviz数据可视化软件中查看小车状态与定位、导航信息。    

### 启动底盘启动程序    
### 启动雷达、雷达数据滤波     
### 发布odom---base_footprint---lidar_link、base_footprint---camera_link TF    
### 订阅cmd_vel    
### 发布odom、scan      
