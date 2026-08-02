## 在ROS 2中启用和可视化点云

本节演示如何从相机节点启用点云数据输出，并使用RViz2进行可视化，类似于之前讨论的[启动相机节点](./start_camera_node.MD)文档中的初始相机节点设置。

### 启用深度点云

#### 启用深度点云的命令

要激活深度信息的点云数据流，请使用以下命令：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true
```

深度点云topic也可以在运行时动态开启或关闭：

```bash
ros2 param set /camera/camera enable_point_cloud false
ros2 param set /camera/camera enable_point_cloud true
```

关闭`enable_point_cloud`会停止发布`/camera/depth/points`，但不会禁用
`get_point_cloud_roi`服务。

#### 在RViz2中可视化深度点云

运行上述命令后，执行以下步骤来可视化深度点云：

1. 打开RViz2。
2. 添加一个`PointCloud2`显示。
3. 选择`/camera/depth/points`主题进行可视化。
4. 将固定帧设置为`base_footprint`以正确对齐数据。这需要存在从相机光学坐标系到`base_footprint`的TF链路。如需在相机光学坐标系中发布点云，可在启动时设置`point_cloud_in_base_footprint:=false`。

##### 示例可视化

以下是在RViz2中可能看到的深度点云的样子：

![深度点云可视化](./images/image5.jpg)

### 启用彩色点云

#### 启用彩色点云的命令

要启用彩色点云功能，请输入以下命令：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_colored_point_cloud:=true
```

#### 在RViz2中可视化彩色点云

要可视化彩色点云数据：

1. 在执行命令后启动RViz2。
2. 添加一个`PointCloud2`显示面板。
3. 从列表中选择`/camera/depth_registered/points`主题。
4. 确保固定帧设置为`base_footprint`。

### 按ROI过滤点云

点云ROI边界使用`base_footprint`坐标系。例如：

```bash
ros2 launch orbbec_camera gemini_330_series.launch.py enable_point_cloud:=true \
  enable_point_cloud_roi:=true \
  point_cloud_roi_min_x:=0.0 point_cloud_roi_max_x:=3.0 \
  point_cloud_roi_min_y:=-0.8 point_cloud_roi_max_y:=0.8 \
  point_cloud_roi_min_z:=0.0 point_cloud_roi_max_z:=1.5
```

### 使用单次调用ROI获取点云

使用`get_point_cloud_roi`服务可以按请求中的ROI边界获取最新深度点云。ROI使用
`base_footprint`坐标系，返回的`sensor_msgs/PointCloud2`也在`base_footprint`坐标系中。

```bash
ros2 service call /camera/get_point_cloud_roi orbbec_camera_msgs/srv/GetPointCloudRoi \
  "{min_x: 0.0, max_x: 3.0, min_y: -0.8, max_y: 0.8, min_z: 0.0, max_z: 1.5}"
```

如果需要在保留ROI边界的同时移除地面附近的点，设置`remove_ground: true`。
地面平面在`base_footprint`中表示为`ax + by + cz + d = 0`。如果`a/b/c`全为0，
服务默认使用水平平面`z = 0`；如果`ground_distance_threshold`为`0.0`，
服务默认使用`0.05`米。

```bash
ros2 service call /camera/get_point_cloud_roi orbbec_camera_msgs/srv/GetPointCloudRoi \
  "{min_x: 0.0, max_x: 3.0, min_y: -0.8, max_y: 0.8, min_z: -0.2, max_z: 1.5, \
    remove_ground: true, ground_distance_threshold: 0.06}"
```

该服务不会应用发布深度点云时使用的降采样或裁剪参数。`point_cloud_stride`、
`point_cloud_max_distance`、`point_cloud_max_points`、`point_cloud_radius`和
`point_cloud_min_neighbors`不会影响该服务返回结果；返回点只受请求中的ROI限制。
如果请求中开启地面平面移除，该过滤会在ROI裁剪之后执行。

##### 示例可视化

在RViz2中彩色点云的结果应类似于这样：

![彩色点云可视化](./images/image6.jpg)
