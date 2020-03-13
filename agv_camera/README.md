# agv camera

## agv_camera.launch
```bash
  roslaunch agv_camera agv_camera.launch
```
驱动usb相机  
发布：
/camera_name/image_raw   
/camera_name/camera_info   
/camera_name/image_rect  

## continuous_detection.launch  
```bash
  roslaunch agv_camera continuous_detection.launch
```
检测视频中的二维码（apriltag）,该包依赖apriltag_ros    
订阅：    
/camera_name/camera_info     
/camera_name/image_rect    
发布：   
/tag_detections  

