# gp4_station

URDF / xacro workcell description for the Yaskawa GP4 robot mounted on station 3.

## Prerequisites

### 1. Place the STL mesh

```bash
# Download or copy station3.stl into the meshes directory
cp /path/to/station3.stl \
   ~/gp4_ws/src/gp4_station/meshes/station3.stl
```

### 2. Make sure `motoman_gp4_support` is in the workspace

```bash
# The package lives under the motoman_ros2_support_packages vendor tree
ls ~/gp4_ws/src/motoman_ros2_support_packages/motoman_gp4_support/urdf/gp4_macro.xacro
```

If it is not present, clone or symlink the MotoPlus support package under
`~/gp4_ws/src/motoman_ros2_support_packages/` before building.

## Build

```bash
cd ~/gp4_ws
colcon build --packages-select gp4_station
source install/setup.bash
```

## Launch

```bash
ros2 launch gp4_station view_station.launch.py
```

This opens RViz with the full station + GP4 model. Use the
**joint_state_publisher_gui** window to pose the robot interactively.

## Fine-tuning station and robot alignment

The workcell uses two fixed joints in `urdf/gp4_on_station.urdf.xacro`:

- `world_to_station`: places the station in the world frame
- `station_to_robot`: places the robot base in the station frame

Current defaults:

```xml
<joint name="world_to_station" type="fixed">
  <parent link="world"/>
  <child  link="station_link"/>
  <origin xyz="0 0 0.8" rpy="1.5708 1.5708 1"/>
</joint>

<joint name="station_to_robot" type="fixed">
  <parent link="station_link"/>
  <child  link="base_link"/>
  <origin xyz="0 0 0" rpy="1.5708 0 -1.5708"/>
</joint>
```

Current calibrated xacro also keeps station mesh visual/collision origin at
`rpy="-1.5708 0 1.5708"` inside `station_link`.
With the full chain (`station_link` visual + `station_to_robot`) the station mesh in
`base_link` is rotated as `Rz(180 deg)` and yields:
X[-0.482, +0.761] Y[-0.197, +0.806] Z[-0.757, +1.093] m.
`scene_objects.yaml` and MoveIt station visual must match this base-frame result.

Procedure:

1. Launch `view_station.launch.py`.
2. In RViz, visually compare the rendered robot position against the station
   mesh (bolt holes, mount pads).
3. Adjust these `<origin>` values as needed:
   - `station_link` mesh `<visual>/<collision>` origin: coarse alignment of
     the STL in station frame.
   - `world_to_station.xyz`: global station placement
   - `station_to_robot.xyz`: robot base offset on the station
   - `x` / `y` – lateral/fore-aft centering on the mount surface.
   - `z` – fine-tune if the flange or tool center is consistently high/low.
   - `station_to_robot.rpy` – base orientation alignment to station fixture.
4. Save the file and re-launch; iterate until aligned.
