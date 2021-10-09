jsk问题解决：

```shell
sudo apt-get install ros-kinetic-jsk-recognition-msgs
sudo apt-get install ros-kinetic-jsk-rviz-plugins
```



# jsk_recognition_msgs/BoundingBox Message

## File: jsk_recognition_msgs/BoundingBox.msg

## Raw Message Definition

```c++
//BoundingBox represents a oriented bounding box.
Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions  # size of bounding box (x, y, z)
//You can use this field to hold value such as likelihood
float32 value
uint32 label
```

 

## Compact Message Definition

```c++
std_msgs/Header header
geometry_msgs/Pose pose
geometry_msgs/Vector3 dimensions
float32 value
uint32 label
```



# jsk_recognition_msgs/BoundingBoxArray Message

## File: jsk_recognition_msgs/BoundingBoxArray.msg

## Raw Message Definition

```c++
//BoundingBoxArray is a list of BoundingBox.
//You can use jsk_rviz_plugins to visualize BoungingBoxArray on rviz.
Header header
BoundingBox[] boxes
```

 

## Compact Message Definition

```
std_msgs/Header header
jsk_recognition_msgs/BoundingBox[] boxes
```

