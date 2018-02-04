# velodyne_calibration
In this repository, I compiled the source code using ROS for the hardware calibration by velodyne

<!-- ## Requirement -->
<!-- - mmath (https://github.com/matchey/mmath) -->

<!-- ## Usage -->
### Create yaml file for Transform (map to vicon)
#### Requirement
- mmath (https://github.com/matchey/mmath)
- complement (https://github.com/matchey/complement)
#### Preparation for the execution
- Set yaml files
	- init_pose.yaml
	- map_name.yaml
	- topic_pc.yaml

#### How to run
```
$ roslaunch velodyne_calibration tf_map2vicon.launch
$ rosrun velodyne_calibration pub_vicon_pose
```
##### Calculate init pose
```
$ vim lastpose.txt
yy o <C-r> = <C-r> 0 <BS> <pose of vicon> <CR>
```

