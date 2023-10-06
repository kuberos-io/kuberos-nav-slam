# Nav2 with KubeROS

This repository contains a navigation software stack based on Nav2 and Gazebo Ignition on Turtlebot4 to demonstrate the applicability of the proposed closed-loop workflow with KubeROS and its scalability in both development and deployment phases.

The typical development cycle for a navigation system for an autonomous mobile robot (AMR) includes the following steps:
 - Comparing algorithms on benchmark datasets 
 - Simulation-based integration, parameter study, testing
 - validation on real robots
 - deployment to a robot fleets (more than one robot)


We use one microservices-oriented, fully containerized software stack to move through all phases with minimal parameter changes.
Code and parameters are separated not only during development but also in the deployment.
The parameters are loaded by KubeROS as `ConfigMap` in Kubernetes and can be easily adjusted at runtime and in each phase. Using the `BatchJob` you can run the experiments in very large scale to find a better sensor setup, optimize the parameters, test the software in different scenes. The software can be easily deployed on any self-hosted KubeROS platform using our pre-built containers (in Docker Hub).


In the example we use the following setup. Later we will upload a tutorial to show you how to customize it with your own robots, algorithms, simulation::
 - Robot: Turtlebot 4
 - Simulation: Gazebo Ignition
 - Environment: Warehouse
 - Navigation stack: Nav2 (with the default planner, and default behavior tree)
 - Localization approaches: AMCL, RTABMap, and SLAM Toolbox



## List of containers

We use the microservices-oriented architecture and package each functional module in its own container. The granularity depends on the application. Below is a brief overview of all containers for this application:

 - All in one container: All ROS2 packages and Gazebo in one container: 
 - gazebo_sim: Container containing only the Gazebo instances with interfaces to ROS
 - nav2_stack: An extended Nav2-based navigation software stack, including several localization algorithms.
 - nav2_rtabmap: Contains only RTABMap as a localizer (in certain applications we use this approach, others are not needed).
 - nav2_slamtoolbox: Contains slamtoolbox as localizer and its configuration parameters
 - nav2_amcl: Contains amcl as localizer, parameters and the pre-scanned maps.
 - nav_to_pose: A container that encapsulates the task_controller package.
 - rosbag_recording: A container used to record the topics like truth_pose from simulation, robot trajectory, etc.
 - rviz_viewer: A container that contains rviz2 and its plugin packages, used only in development phase.


## Development

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
Create a new network interface ([Bridge network](https://docs.docker.com/network/network-tutorial-standalone/#use-user-defined-bridge-networks))
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

