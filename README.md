# v4l2_camera

A ROS 2 camera driver using Video4Linux2 For Canlab (V4L2).

### System Requirements

Requirements:
  * [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

### Download Pacakage
If you need to modify the code or ensure you have the latest update you will need to clone this repo then build the package.

    $ mkdir -p ~/ros2_cam/src
    $ cd ~/ros2_cam/src
    $ git clone http://192.168.0.54:90/cleaning/ros2_cam.git
    $ cd ~/ros2_cam
    $ colcon build
    $ source ~/ros2_cam/install/setup.bash

### Usage
Publish camera images, using the parameters:

        # launch the v4l2_camera executable
        ros2 launch v4l2_camera cleaning_cam_launch.py

Preview the image (open another terminal):

        ros2 run rqt_image_view rqt_image_view

## DDS Configuration
For better image transport performance over DDS, we recommend using [FastDDS](https://github.com/eProsima/Fast-DDS) with Shared Memory Transport enabled.
First copy the the `fastdds.xml` config file to a suitable directory, eg. `$HOME/fastdds.xml`
```bash
cd ~/ros2_cam/src/ros2_cam
cp fastdds.xml ~/
```

Next add these two lines to your `~/.bashrc`
```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=$HOME/fastdds.xml
```

Make sure to `source ~/.bashrc` first on all terminals before launching any ROS 2 nodes including the driver.

## Nodes

### v4l2_camera_node

The `v4l2_camera_node` interfaces with standard V4L2 devices and
publishes images as `sensor_msgs/msg/Image` messages.

#### Published Topics

* `/image_raw` - `sensor_msgs/msg/Image`

    The image.

#### Parameters

* `video_device` - `string`, default: `"/dev/video0"`

    The device the camera is on.

* `pixel_format` - `string`, default: `"UYVY"`

    The pixel format to request from the camera. Must be a valid four
    character '[FOURCC](http://fourcc.org/)' code [supported by
    V4L2](https://linuxtv.org/downloads/v4l-dvb-apis/uapi/v4l/videodev.html)
    and by your camera. The node outputs the available formats
    supported by your camera when started.  
    Currently supported: `"UYVY"`

* `output_encoding` - `string`, default: `"yuv422"`

    The encoding to use for the output image.  
    Currently supported: `"yuv422", "mono8"`.  
  
* `image_size` - `integer_array`, default: `[1920, 1080]`

    Width and height of the image.  
    Currently supported: `[1920, 1080], [640, 480]`

* Camera Control Parameters

    Not Support
