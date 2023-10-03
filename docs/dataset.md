## Comparision on Datasets

### TUM Dataset

```
ros2 launch nav2_edge_eval rgbd_tum_eval.launch.py world:=tum localization_method:=<amcl|slamtoolbox|rtabmap> input_dataset:=<path to prepared tum rosbag> start_pose:=<start pose from table below>
```

|Dataset No.| `start_pose` |
|-----------|--------------|
| 1         | `start_pose:="3.6,-1.8,3.4"` |
| 2         | `start_pose:="3.0,-1.6,0.3"` |
| 3         | `start_pose:="-0.5,-5.0,-1.4"` |
