# Drone Detect

## Overview

This is a package for detecting other drones in depth image and then calculating their true pose error in the world frame of the observer drone.



![Example image](doc/demo.jpg)



## Usage

You can launch the node alongside the main after you assigning the right topic name

```bash
ros2 launch drone_detect drone_detect.launch.py
```



or add the following code in `run_in_sim.launch.py`

```python
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

Node(
    package='drone_detect',
    executable='drone_detect',
    name=f'drone_{drone_id}_drone_detect',
    output='screen',
    parameters=[
        os.path.join(get_package_share_directory('local_sensing_node'), 'params', 'camera.yaml'),
        os.path.join(get_package_share_directory('drone_detect'), 'config', 'default.yaml'),
        {'my_id': drone_id},
    ],
    remappings=[
        ('~/odometry', f'/drone_{drone_id}_{odom_topic}'),
        ('~/depth', f'/drone_{drone_id}_pcl_render_node/depth'),
        ('~/colordepth', f'/drone_{drone_id}_pcl_render_node/colordepth'),
        ('~/camera_pose', f'/drone_{drone_id}_pcl_render_node/camera_pose'),
        ('~/drone0', f'/drone_0_{odom_topic}'),
        ('~/drone1', f'/drone_1_{odom_topic}'),
        ('~/drone2', f'/drone_2_{odom_topic}'),
    ],
)
```



## Config files

* **camera.yaml** : The camera intrinsics are stored in this file
* **default.yaml**: Some parameters related to drone detection.

```yaml
debug_flag: true
# the proportion of the pixel threshold to the total pixels projected from the drone to the depth map 
estimate/pixel_ratio: 0.1
# the radius of the pose errror sphere
estimate/max_pose_error: 0.4
# the width and height of the model of drone
estimate/drone_width: 0.5  
estimate/drone_height: 0.1 
```



## Nodes


#### Subscribed Topics

* **`/depth`** ([sensor_msgs::Image])

	The depth image from pcl_render_node.
	
* **`/colordepth`**([sensor_msgs::Image])

  The color image from pcl_render_node

* **`/camera_pose`** ([geometry_msgs::PoseStamped])

  The camere pose from pcl_render_node

The above three topics are synchronized when in use, the callback function is **`rcvDepthColorCamPoseCallback`**

- **`/dronex_odom_sub_`** ([nav_msgs::Odometry])

  The odometry of other drones

#### Published Topics

* **`/new_depth`** ([sensor_msgs::Image])

  The new depth image after erasing the moving drones

* **`/new_colordepth`**([sensor_msgs::Image])

  The color image with some debug marks

* **`/camera_pose_error`** ([geometry_msgs::PoseStamped])

  The pose error of detected drones in world frame of the observer drone.