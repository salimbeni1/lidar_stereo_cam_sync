# ROS sync for lidar and 2 cameras


In this repo we propose 3 ways to sync the 3 sensors :

* sync_node_lidar : to sync with the rate of the lidar topic
* sync_node_stereo: to sync with the rate of one camera topic
* sync_node_freq : to sync with a given rate

## setup

at the root of the repo
```
catkin_make
source devel/setup.bash
catkin_make
```

## run

to launch the node simply use :
```
rosrun sync sync_node_freq
rosrun sync sync_node_lidar
rosrun sync sync_node_stereo
```

### params
by default sync_node_freq will sample at 10Hz
to use other frequencies do : 
```
rosrun sync sync_node_freq _freq:=15.0
```

## input / output topics

### input 

* `/os1_cloud_node/points` : lidar point clouds
* `/Camera/camera0_image`  : image stream 0
* `/Camera/camera1_image`  : image stream 1 

### output

* `/sync_node/lidar` : synchronized point cloud stream
* `/sync_node/cam0`  : synchronized image stream 0
* `/sync_node/cam1`  : synchronized image stream 1
