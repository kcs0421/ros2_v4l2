# Copyright 2018 Lucas Walter
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Lucas Walter nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse
import os
import sys

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    image_size_rgb = LaunchConfiguration('image_size', default="[1920, 1080]")
    image_size_ir = LaunchConfiguration('image_size', default="[640, 480]")
    pixel_format = LaunchConfiguration('pixel_format', default='UYVY')
    ld = LaunchDescription()


    rgb_cam_node=Node(
        package='v4l2_camera', executable='v4l2_camera_node', output='screen',
        name='rgb_cam_node',
        #namespace='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video0'},
            {'camera_frame_id': 'video0'},
            {'image_size': image_size_rgb},
            {'pixel_format': pixel_format },
            {'output_encoding': 'yuv422'},
        ])
    ir_cam_node=Node(
        package='v4l2_camera', executable='v4l2_camera_node', output='screen',
        name='ir_cam_node',
        #namespace='v4l2_camera',
        parameters=[
            {'video_device': '/dev/video1'},
            {'camera_frame_id': 'video1'},
            {'image_size': image_size_ir},
            {'pixel_format': pixel_format },
            {'output_encoding': 'mono8'},
        ])
        
    ld.add_action(rgb_cam_node)
    ld.add_action(ir_cam_node)
    
    return ld
