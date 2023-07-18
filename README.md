# lidar_filter

This pkg filters a sensor_msgs/PointCloud2 msg (default topic `/rslidar_points_unfiltered`) and republishes it (default topic `/rslidar_points`). The filter clips points that are within a specified box around `base_link`.

Parameters are:
- `input_topic` (default: /rslidar_points_unfiltered)
- `output_topic` (default: /rslidar_points)
- `x_min` (default: -0.42)
- `x_max` (default: 0.42)
- `y_min` (default: -0.31)
- `y_max` (default: 0.31)
- `z_min` (default: -0.1)
- `z_max` (default: 1.0)
