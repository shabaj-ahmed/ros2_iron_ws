# PointCloud Height Filter (ROS 2 Iron)
This ROS 2 node filters incoming sensor_msgs/PointCloud2 messages by the z (height) field. Points with z values outside the configured [min_height, max_height] range are removed, and the remaining points are republished with their full records preserved (x, y, and all other fields).


## Assumptions & Limitations
- Input is always a 3D LiDAR point cloud with valid x, y, and z fields.
- No runtime validation is performed on field presence. For the provided dataset, these fields are guaranteed.
- min_height and max_height are assumed valid and ordered.
- No checks are made against the physical operating bounds of the LiDAR.


## Implementation Notes
1. Subscribes to an input sensor_msgs/PointCloud2 topic and republishes a filtered cloud with the same field layout.
2. Parameters:
    - min_height (default: 0.0)
    - max_height (default: 2.0)
3. Filtering is performed in two passes:
    1. Count how many points fall within [min_height, max_height].
    2. Allocate an output cloud of that size and copy valid points.
4. Copying uses std::memcpy at the record level (point_step bytes per point), which preserves all fields (e.g., intensity, ring, time) for valid points.


## Usage
This package includes a launch file. It launches one node you can modify the parameters in this file.


### Run the node (Terminal A):
```
ros2 launch pointcloud_height_filter filter.launch.py \
  min_height:=0.0 max_height:=2.0 \
  input:=/lidars/bpearl_front_right \
  output:=/lidars/bpearl_front_right_filtered
```


### Run against a rosbag (MCAP)
You can test the node by replaying a .mcap bag that contains sensor_msgs/PointCloud2.


#### Prereqs
sudo apt install -y ros-iron-rosbag2-storage-mcap ros-iron-rosbag2-transport


#### Play the bag (Terminal B):
```
source /opt/ros/iron/setup.bash
ros2 bag play /path/to/lidar_data_0.mcap --loop -r 0.5
```


## Future Improvements
- Parameter validation
    - Ensure min_height <= max_height.
    - Optionally reject startup or swap values if invalid.
    - Warn if values are outside expected LiDAR range.
- Message format checks
    - Verify x, y, z fields exist and are FLOAT32.
    - Ignore, log, or error if malformed.
- Health feedback
    - Log number of points kept/dropped.
    - Publish diagnostics via diagnostic_updater.
- Performance options
    - Single-pass filtering with a growing buffer.
    - GPU or SIMD acceleration for high-density clouds.

