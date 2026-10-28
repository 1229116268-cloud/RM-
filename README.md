# RoboMaster 作业 5 · MID-360 建图与二维地图

用 bag 中的 Livox MID-360 点云与 IMU 数据建立三维点云地图，再把三维地图投影成二维栅格地图，并在 rviz 中对齐显示。

## 环境

- Ubuntu 22.04 / ROS 2 Humble
- Livox-SDK2 + livox_ros_driver2（提供 `livox_ros_driver2/msg/CustomMsg`）
- FAST-LIO（ROS 2 分支）
- Python 3 + numpy + opencv-python（离线调参脚本）

## 编译

    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    cd ~/RM-
    colcon build --symlink-install

## 数据

bag 放在仓库外（约 407 MB），包含两个话题：

- `/livox/lidar_192_168_1_140` — `livox_ros_driver2/msg/CustomMsg`（1042 帧）
- `/livox/imu_192_168_1_140` — `sensor_msgs/msg/Imu`（20783 条）

## 运行（三维建图）

1. 修改 FAST-LIO 配置 `config/mid360_bag.yaml`：话题改为上面的真名，`pcd_save.pcd_save_en` 设为 `true`，`map_file_path` 指向 `maps/map_3d.pcd`

2. 启动建图（不开 rviz 省资源）：

    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    source ~/ws_fastlio/install/setup.bash
    export LD_LIBRARY_PATH=$(echo "$LD_LIBRARY_PATH" | tr ':' '\n' | grep -v '/opt/MVS' | paste -sd: -)
    ros2 launch fast_lio mapping.launch.py config_file:=mid360_bag.yaml \
      config_path:=$HOME/ws_fastlio/src/fast_lio/config use_sim_time:=true rviz:=false

3. 另一个终端回放数据：

    source /opt/ros/humble/setup.bash
    source ~/ws_livox/install/setup.bash
    ros2 bag play ~/bags/rosbag2_2026_05_28-17_11_13 --clock 200 --rate 1.0 \
      --topics /livox/lidar_192_168_1_140 /livox/imu_192_168_1_140

4. 回放结束后保存地图：

    ros2 service call /map_save std_srvs/srv/Trigger

## 二维处理节点（map_processor_node）

订阅 `/mapping/map`（整张地图快照），依次做：去掉无效点 → 体素去重（`voxel_size`，默认 0.1 m）→ 3σ 去除极端离群点 → 投影到 0.1 m 栅格，落点标 100（占用），其余标 -1（未知），发布 `nav_msgs/msg/OccupancyGrid` 到 `/mapping/map_2d`。

运行：

    source install/setup.bash
    ros2 run mid360_mapping map_processor_node --ros-args \
      --params-file src/mid360_mapping/config/map_processing.yaml

保存（写 `map_filtered.pcd`、`map.pgm`、`map.yaml` 到 `output_directory`）：

    ros2 service call /mapping/save_artifacts std_srvs/srv/Trigger

测试时可用 `pcl_ros` 把 pcd 直接发成话题：

    ros2 run pcl_ros pcd_to_pointcloud --ros-args \
      -p file_name:=$HOME/RM-/maps/map_3d.pcd -p tf_frame:=map -p publishing_period_ms:=5000 \
      -r /cloud_pcd:=/mapping/map

## 离线脚本（可选）

`scripts/pcd_to_map.py` 与节点逻辑一致，用于快速试参数：

    python3 scripts/pcd_to_map.py

## 结果

- `maps/map_3d.pcd`：125,485 个点，约 3.9 MB
- `maps/map.pgm` + `maps/map.yaml`：708 × 653 格，分辨率 0.1 m
- `docs/rviz.png`：rviz 中三维点云与二维栅格的对齐效果

## 说明

- `mapping_node` 仍为骨架；三维地图由 FAST-LIO 直接产出并保存。
- 二维图采用保守策略：只有观测到落点的格子标占用，其余标未知（205），未做射线清除。
- 曾尝试射线清除（射线遇障碍即停）：由于点云在传感器附近非常密集（2 m 内 1610 个点），单原点射线走一两格就被截断，输出几乎全是未知，因此未采用。
- 该数据的地面不在 z≈0 附近（z 中位数约 17.7 m），没有稳定的水平地面，因此未做地面分割。

## 参考

- FAST-LIO: https://github.com/hku-mars/FAST_LIO
- livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2
