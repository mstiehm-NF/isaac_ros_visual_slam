# SPDX-FileCopyrightText: NVIDIA CORPORATION & AFFILIATES
# Copyright (c) 2021-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# SPDX-License-Identifier: Apache-2.0
import os
import yaml
import launch
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def load_camera_pose(file_path):
    with open(file_path, 'r') as file:
        data = yaml.safe_load(file)
    return data

def generate_launch_description():

    # Load camera pose from YAML file
    camera_pose_file = '/usr/config/camera_pose.yaml'
    camera_pose = load_camera_pose(camera_pose_file)

    x, y, z = camera_pose['translation']['x'], camera_pose['translation']['y'], camera_pose['translation']['z']
    roll, pitch, yaw = camera_pose['rotation']['roll'], camera_pose['rotation']['pitch'], camera_pose['rotation']['yaw']

    launch_args = [
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the nodes'
        ),
        ExecuteProcess(
            cmd=['echo', 'Namespace is:', LaunchConfiguration('namespace')],
            output='screen',
        ),
    ]

    # Namespace
    namespace = LaunchConfiguration('namespace')

     # Static transform publisher
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        ## Arguments: x, y, z, r, p, y, frame_id, child_frame_id
        arguments=[str(x), str(y), str(z), str(roll), str(pitch), str(yaw), 'base_link', 'camera_link'],
        output='screen') 

    """Launch file which brings up visual slam node configured for RealSense."""
    realsense_camera_node = Node(
        name='camera',
        namespace=namespace,
        package='realsense2_camera',
        executable='realsense2_camera_node',
        parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_color': True,
                'enable_depth': True,
                'pointcloud.enable': True,
                'depth_module.emitter_enabled': 0,
                'depth_module.profile': '640x360x90',
                'rgb_camera.profile': '1280x720x30',
                'enable_gyro': True,
                'enable_accel': True,
                'gyro_fps': 200,
                'accel_fps': 250,
                'unite_imu_method': 2
        }]
    )

    visual_slam_node = ComposableNode(
        name='visual_slam_node',
        namespace=namespace,
        package='isaac_ros_visual_slam',
        plugin='nvidia::isaac_ros::visual_slam::VisualSlamNode',
        parameters=[{
                    'denoise_input_images': False,
                    'rectified_images': True,
                    'enable_debug_mode': False,
                    'debug_dump_path': '/tmp/cuvslam',
                    'enable_slam_visualization': True,
                    'enable_landmarks_view': True,
                    'enable_observations_view': False,
                    'map_frame': 'map',
                    'odom_frame': 'odom',
                    'base_frame': 'base_link',
                    'input_imu_frame': 'camera_gyro_optical_frame',
                    'enable_imu_fusion': True,
                    'gyro_noise_density': 0.000244,
                    'gyro_random_walk': 0.000019393,
                    'accel_noise_density': 0.001862,
                    'accel_random_walk': 0.003,
                    'calibration_frequency': 200.0,
                    'img_jitter_threshold_ms': 25.00,
                    'path_max_size': 1000000,
                    'enable_planar_mode': True,
                    }],
        remappings=[('stereo_camera/left/image', 'infra1/image_rect_raw'),
                    ('stereo_camera/left/camera_info', 'infra1/camera_info'),
                    ('stereo_camera/right/image', 'infra2/image_rect_raw'),
                    ('stereo_camera/right/camera_info', 'infra2/camera_info'),
                    ('visual_slam/imu', 'imu')]
    )

    visual_slam_launch_container = ComposableNodeContainer(
        name='visual_slam_launch_container',
        namespace=namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            visual_slam_node
        ],
        output='screen'
    )
    final_launch_description = launch_args + [visual_slam_launch_container, static_tf, realsense_camera_node]
    return launch.LaunchDescription(final_launch_description)