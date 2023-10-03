# Nav2 + Ignition


Run it in local:
```bash
docker run -it --rm \
    -e NOVNC_ENABLE=true -e WINDOW_MANAGER_ENABLE=true \
    -e STARTX11=true \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -p 8080:8080 --name nav2_edge \
    ToDo
```

Visualize with NoVNC in browser: `http://localhost:8080/`


## Launch in Development

**All in One Launch**
```bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch nav2_edge_eval nav2_edge_eval.launch.py record_rosbag:=true world:=warehouse world_var:=D1 localization_method:=rtabmap start_pose:='0,0,0' goal_poses:='2,-2,0;0,-3.5,0;-2,-3,0'
```

**Visualize Nav2**
```bash
ros2 launch nav2_bringup rviz_launch.py rviz_config:=/workspace/tb4_bringup/config/nav2_eval_rviz.config.rviz
```


**Visualize Ignition world**
check the world and light conditions
```bash
ros2 launch tb4_bringup ignition.launch.py world:=warehouse world_var:=D1
```


**ignition and spwan a robot**
```bash
ros2 launch tb4_bringup turtlebot4_ignition.launch.py world:=warehouse rviz:=false
```



## Deployment with KubeROS


## Evaluation with KubeROS BatchJob



## For Testing using Docker in local
Create a new network interface
```bash
docker network create --driver bridge ros-net
```

## Development in DevContainer

You can easily try this example with `DevContainer` in `VSCode`, without installing any packages on your host system. 

Depending on the system setup, you may need to do the following. 

To use an Nvidia GPU, add the following variables to `.devcontainer/docker-compose.yml`
```yaml
environment:
    DISPLAY: $DISPLAY
    NVIDIA_VISIBLE_DEVICES: all
    NVIDIA_DRIVER_CAPABILITIES: all
runtime: nvidia
```
Otherwise, comment it out.


Allow the root user to connect to X server:
```bash
xhost + local:root
```


## Request navigation

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{ pose: { header: { frame_id: 'map' }, pose: { position: { x: 0, y: 1 } } } }"
```

## Visualize Nav2

```
ros2 launch nav2_bringup rviz_launch.py
```

## Dataset usage

### Capture dataset from simulation run

```bash
# run with simulation, record also input topics
ros2 launch nav2_edge_eval nav2_edge_eval.launch.py record_input_rosbag:=True record_rosbag:=True

# remove all localization topics and map->odom transform
./utils/rosbag_filter_localization_output.py --parent-dir rosbags/

#final rosbag is placed in rosbags/<test_run>_localization_input/
```

### Playback of rosbag to evaluate localization

```bash
ros2 launch nav2_edge_eval nav2_edge_eval.launch.py mode:=dataset input_dataset:=rosbags/<test_run>_localization_input/ input_dataset_playback_rate:=1.0
```

### Run TUM dataset

```
ros2 launch nav2_edge_eval rgbd_tum_eval.launch.py world:=tum localization_method:=<amcl|slamtoolbox|rtabmap> input_dataset:=<path to prepared tum rosbag> start_pose:=<start pose from table below>
```

|Dataset No.| `start_pose` |
|-----------|--------------|
| 1         | `start_pose:="3.6,-1.8,3.4"` |
| 2         | `start_pose:="3.0,-1.6,0.3"` |
| 3         | `start_pose:="-0.5,-5.0,-1.4"` |

# Update Notes: 

## V0.2.11
 - Fixed problem with lifecycle manager
 - Add realrobot dataset

