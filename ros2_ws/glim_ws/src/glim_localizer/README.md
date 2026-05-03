# glim_localizer

GLIM saved map 위에서 **Localization + Map Update** 를 수행하는 ROS2 패키지.

## Components

| Component | Type | 역할 |
|---|---|---|
| `map_loader_node` | ROS2 node | GLIM map 로드 → `/map_pointcloud` publish |
| `localization_node` | ROS2 node | NDT(coarse) + VGICP(fine) → `/localized_pose` |
| `map_update_node` | ROS2 node | localized scan을 GlobalMapping에 integrate |
| `init_pose_viewer` | GLIM extension module | Top-down ImGui UI → `/initial_pose` publish |

## 빌드

```bash
cd ~/box/vln-large-scale-farm/glim_ws
colcon build --packages-select glim_localizer --symlink-install
```

## init_pose_viewer 등록

`ros2_ws/glim_ws/config/config_ros.json` 의 `extension_modules` 에 추가:

```json
"extension_modules": [
  "libmemory_monitor.so",
  "libstandard_viewer.so",
  "librviz_viewer.so",
  "libinit_pose_viewer.so"   ← 추가
]
```

빌드 후 .so 위치 확인:
```bash
find ros2_ws/glim_ws/install -name "libinit_pose_viewer.so"
```
→ `LD_LIBRARY_PATH` 에 해당 경로가 있어야 GLIM이 로드할 수 있음.

## 실행 순서

### 1. GLIM (map + init_pose_viewer UI 포함)
```bash
source ros2_ws/glim_ws/install/setup.bash
ros2 run glim_ros glim_rosnode \
  --ros-args -p config_path:=/home/koala/box/vln-large-scale-farm/ros2_ws/glim_ws/config
```

### 2. localizer stack
```bash
source ros2_ws/glim_ws/install/setup.bash
ros2 launch glim_localizer localizer.launch.py \
  map_path:=/path/to/your/saved_map
```

### 3. 수동 map save (필요시)
```bash
ros2 service call /map_update_node/save_map std_srvs/srv/Trigger
```

## Topic graph

```
[init_pose_viewer] ──/initial_pose──→ [localization_node]
                                             ↑
[map_loader_node] ──/map_pointcloud──────────┘
                                             ↓
                              /localized_pose + TF(map→livox_frame)
                                             ↓
                                    [map_update_node]
                                             ↓
                               GlobalMapping::save() 주기적 실행
```

## 주의사항

### VGICP factor API
`gtsam_points` 버전에 따라 `IntegratedVGICPFactor` 생성자 인자가 다를 수 있음.
`localization_node.cpp` 의 `vgicp_align()` 에서 컴파일 오류 시:

```cpp
// 버전 A (source_key, target_key, voxelmap, source_cloud)
auto f = make_shared<IntegratedVGICPFactor>(X(0), X(0), vmap, scan);

// 버전 B (source_key, voxelmap, source_cloud)
auto f = make_shared<IntegratedVGICPFactor>(X(0), vmap, scan);
```

gtsam_points include 디렉토리에서 헤더 확인:
```bash
grep -r "IntegratedVGICPFactor" \
  ~/box/vln-large-scale-farm/ros2_ws/glim_ws/src/*/include --include="*.hpp" | head -5
```

### SubMap 필드
`map_update_node.cpp` 에서 SubMap을 직접 생성함.
SubMap 구조체 필드가 버전마다 다를 수 있으므로 `glim/mapping/sub_map.hpp` 확인 필요.
