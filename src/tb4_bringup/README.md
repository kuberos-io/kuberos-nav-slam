# Turtlebot4 Bringup

Package includes launch files to start the Gazebo Ignition Simulator and spawn the Turtlebot 4.

### Launch and arguments: 

```bash
ros2 launch tb4_bringup turtlebot4_ignition.launch.py
```

 - **namespace**: robot namespace, used for simulation with multiple robots. 
 - **world**: Ignition world (scene). used in `ignition.launch.py`. Ignition searches for the `{world}.sdf` in the folders that defined by `IGN_GAZEBO_RESOURCE_PATH`. default value: `warehouse`
 - **world_var**: World variations
 - **model**: select the turtlebot 4 model: `standard` or `lite`.
 - **launch_simulation**: Set false to skip the simulation startup. Default: `True`
 - **headless**: Start the ignition GUI or not. Default: `True`


### Customized Worlds

Different ignition world (warehouse scene) are created by randomizing the lighting conditions and the obstacles. The world descrition in stored in the folder `worlds`.


### Todos
 - [] ...

