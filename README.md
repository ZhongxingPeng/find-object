## find_object_2d (ROS package)

This package is forked from [introlab/find-object](https://github.com/introlab/find-object), and modified by Zhongxing Peng.

### Build from Source

```bash
 $ cd ~/catkin_ws
 $ git clone https://github.com/ZhongxingPeng/find-object.git
 $ catkin_make
```

### Run

- Run with `uvc_camera` 
    ```bash
    $ roscore &
    $ rosrun uvc_camera uvc_camera_node &
    $ rosrun find_object_2d find_object_2d image:=image_raw
    ```

- Run with `usb_cam`

    ```bash
    roscore &
    rosrun usb_cam usb_cam_node _pixel_format:=yuyv &
    rosrun find_object_2d find_object_2d image:=/usb_cam/image_raw
    ```
