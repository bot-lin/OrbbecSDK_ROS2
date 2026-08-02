## Enabling and Visualizing Point Cloud in ROS 2

This section demonstrates how to enable point cloud data output from the camera node and visualize it using RViz2, similarly to the initial camera node setup discussed in the [Starting Camera Node](./start_camera_node.MD) document.

### Enabling Depth Point Cloud

#### Command to Enable Depth Point Cloud

To activate the point cloud data stream for depth information, use the following command:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true
```

The depth point cloud topic can also be enabled or disabled at runtime:

```bash
ros2 param set /camera/camera enable_point_cloud false
ros2 param set /camera/camera enable_point_cloud true
```

Disabling `enable_point_cloud` stops `/camera/depth/points` publication. It does not disable the
`get_point_cloud_roi` service.

#### Visualizing Depth Point Cloud in RViz2

After running the above command, perform the following steps to visualize the depth point cloud:

1. Open RViz2.
2. Add a `PointCloud2` display.
3. Select the `/camera/depth/points` topic for visualization.
4. Set the fixed frame to `base_footprint` to properly align the data. This requires a TF path from
   the camera optical frame to `base_footprint`. To publish the cloud in the camera optical frame
   instead, launch with `point_cloud_in_base_footprint:=false`.

##### Example Visualization

Here is what the depth point cloud might look like in RViz2:

![Depth Point Cloud Visualization](./images/image5.jpg)

### Enabling Colored Point Cloud

#### Command to Enable Colored Point Cloud

To enable the colored point cloud feature, enter the following command:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_colored_point_cloud:=true
```

#### Visualizing Colored Point Cloud in RViz2

To visualize the colored point cloud data:

1. Launch RViz2 following the command execution.
2. Add a `PointCloud2` display panel.
3. Choose the `/camera/depth_registered/points` topic from the list.
4. Ensure the fixed frame is set to `base_footprint`.

### Filtering Point Cloud by ROI

Point cloud ROI bounds are expressed in `base_footprint`. For example:

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true \
  enable_point_cloud_roi:=true \
  point_cloud_roi_min_x:=0.0 point_cloud_roi_max_x:=3.0 \
  point_cloud_roi_min_y:=-0.8 point_cloud_roi_max_y:=0.8 \
  point_cloud_roi_min_z:=0.0 point_cloud_roi_max_z:=1.5
```

### Getting a Point Cloud with a Per-Call ROI

Use the `get_point_cloud_roi` service to request the latest depth point cloud clipped by ROI bounds
provided in the request. The ROI is expressed in `base_footprint`, and the returned
`sensor_msgs/PointCloud2` is also in `base_footprint`.

```bash
ros2 service call /camera/get_point_cloud_roi orbbec_camera_msgs/srv/GetPointCloudRoi \
  "{min_x: 0.0, max_x: 3.0, min_y: -0.8, max_y: 0.8, min_z: 0.0, max_z: 1.5}"
```

To remove points near a ground plane while keeping the ROI limits, set `remove_ground: true`.
The ground plane is expressed in `base_footprint` as `ax + by + cz + d = 0`. If `a/b/c` are
all zero, the service uses the default horizontal plane `z = 0`. If
`ground_distance_threshold` is `0.0`, the service uses `0.05` meters.

```bash
ros2 service call /camera/get_point_cloud_roi orbbec_camera_msgs/srv/GetPointCloudRoi \
  "{min_x: 0.0, max_x: 3.0, min_y: -0.8, max_y: 0.8, min_z: -0.2, max_z: 1.5, \
    remove_ground: true, ground_distance_threshold: 0.06}"
```

The service does not apply the published depth cloud reduction filters. `point_cloud_stride`,
`point_cloud_max_distance`, `point_cloud_max_points`, `point_cloud_radius`, and
`point_cloud_min_neighbors` do not affect this response; only the requested ROI limits the returned
points. If enabled in the service request, ground-plane removal is applied after ROI clipping.

##### Example Visualization

The result of the colored point cloud in RViz2 should look similar to this:

![Colored Point Cloud Visualization](./images/image6.jpg)
