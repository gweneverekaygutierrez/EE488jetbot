# jetbot_picam
Detect color object on Nvidia Jetbot. The color area is classified as HSV and the area can be defined by the parameters.

## Subscribed Topic
- cam_pi (sensor_msgs/Image)

## Published Topic
- bounding_box (vision_msgs/BoundingBox2D)
- inrange_image (sensor_msgs/Image)

## Parmameters
- lowre_h (int)
- lowre_s (int)
- lowre_v (int)
- higher_h (int)
- higher_s (int)
- higher_v (int)

# Tutorial
```
$ cd {your_ROS_workspace}/src
$ git clone https://github.com/hdh7485/jetbot_picam.git
$ cd ../
$ source devel/setup.bash
$ rospack profile
$ rosdep install -ary
$ roslaunch jetbot_picam color_detector.launch
```

# Result
![image](./docs/images/result.png)
