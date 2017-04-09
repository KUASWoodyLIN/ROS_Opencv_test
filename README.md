# ROS_Opencv_test
A ROS package that including Opencv (Image capture and video image detection), MAVROS, Web videoserver.

## VideoCapture and VideoCapture_cam
VideoCapture using opencv VideoCapture.

VideoCapture_cam subscribe topics from "camera/image" topic.

## Camshift and Camshift_cam
Camshift using opencv VideoCapture and detection the object like ball,color,people.

Camshift_cam subscribe topics from "camera/image" topic and detection the object like ball,color,people.

## faceshow
Detection people face.

## peopledetect
Detection people.

#found ball(Gazebo and Erle_copter version)
Inside include the (Web video sever, Copter control system and Video detection system)

##Web video sever
```bash
git clone https://github.com/RobotWebTools/web_video_server
```
##Copter control system
```bash
found_ball/src/copture_control.cpp
```
##Video detection system
```bash
found_ball/src/ball_tracking.cpp
```