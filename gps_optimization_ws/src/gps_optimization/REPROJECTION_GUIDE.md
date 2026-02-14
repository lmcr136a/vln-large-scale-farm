# GPS Optimization - Re-projection 구현

## 🎯 주요 변경사항

### 제거된 토픽 (의미 없는 것들)
- ❌ `/odometry/gps_fused` - 제거
- ❌ `/Laser_map` 구독 - 제거
- ❌ `/Laser_map_gps_fixed` (통째로 변환) - 제거
- ❌ `/path` 구독 - 제거
- ❌ `/path_in_map` - 제거

### 새로 추가된 기능

✅ **/cloud_registered 구독**
- FAST-LIO2의 각 스캔을 timestamp와 함께 저장
- 각 스캔을 현재 node_index와 매칭

✅ **GTSAM Optimized Poses 저장**
- 최적화된 모든 pose를 `std::map<int, gtsam::Pose3>`에 저장
- node_index → 최적화된 Pose 매핑

✅ **주기적 맵 재구성 (Re-projection)**
- 10초마다 (설정 가능) 전체 맵을 재구성
- 각 스캔을 해당 시점의 **최적화된 pose**로 변환
- 모든 변환된 스캔을 합쳐서 전역 맵 생성
- `/Laser_map_gps_fused`로 발행

✅ **/path_gps_fused 발행**
- 최적화된 pose들로 경로 생성
- map 프레임 기준

## 📊 작동 원리

### 이전 (잘못된 방식)
```
FAST-LIO2 맵 (휘어진 맵, camera_init 프레임)
    ↓
현재 TF로 통째로 변환
    ↓
/Laser_map_gps_fixed (여전히 휘어진 맵, map 프레임)
```

### 현재 (올바른 Re-projection)
```
각 스캔 저장 (timestamp, cloud, node_index)
    ↓
각 node의 최적화된 pose 저장 (GTSAM)
    ↓
주기적으로:
  for each scan:
    optimized_pose = poses[scan.node_index]
    transformed_scan = transform(scan.cloud, optimized_pose)
    global_map += transformed_scan
    ↓
/Laser_map_gps_fused (GPS 보정된 정확한 맵, map 프레임)
```

## 🔧 설정 파일 (config/config.yaml)

```yaml
/**:
  ros__parameters:
    # Input topics
    odom_topic: "/Odometry"
    gps_topic: "/ublox_driver/receiver_lla"
    cloud_registered_topic: "/cloud_registered"  # <- 새로 추가
    
    # Output topics
    laser_map_gps_fused_topic: "/Laser_map_gps_fused"
    path_gps_fused_topic: "/path_gps_fused"
    
    # Frame IDs
    map_frame: "map"
    camera_init_frame: "camera_init"
    
    # Map reconstruction settings
    map_update_interval: 10.0  # 맵 재구성 주기 (초)
    max_scan_history: 1000  # 메모리에 유지할 최대 스캔 개수
    
    # GPS/Odom noise models
    gps_noise_x: 2.0
    gps_noise_y: 2.0
    gps_noise_z: 4.0
    # ... (동일)
```

## 🚀 빌드 및 실행

```bash
# 기존 파일 백업 (선택사항)
cd ~/box/vln-large-scale-farm/gps_optimization_ws/src/gps_optimization
cp -r . ../gps_optimization_backup

# 새 파일로 교체
cp <다운로드경로>/config/config.yaml config/
cp <다운로드경로>/src/gps_fusion_node.cpp src/

# 빌드
cd ~/box/vln-large-scale-farm/gps_optimization_ws
colcon build --packages-select gps_optimization --symlink-install
source install/setup.bash

# 실행
ros2 launch gps_optimization gps_fusion.launch.py
```

## 📡 토픽 구조

### 입력 토픽
- `/Odometry` - FAST-LIO2 오도메트리 (camera_init 프레임)
- `/ublox_driver/receiver_lla` - GPS 데이터
- `/cloud_registered` - FAST-LIO2 각 스캔 (camera_init 프레임)

### 출력 토픽
- `/Laser_map_gps_fused` - GPS 보정된 재구성 맵 (map 프레임)
  - 10초마다 업데이트 (설정 가능)
  - 진짜 GPS 보정이 반영된 정확한 맵!
- `/path_gps_fused` - GPS 최적화된 경로 (map 프레임)

### TF
- `map → camera_init` - GPS 보정량

## 🎨 RViz2 시각화

```bash
rviz2
```

**설정**:
- Fixed Frame: `map`

**추가할 디스플레이**:
1. **PointCloud2**: `/Laser_map_gps_fused`
   - Color: 녹색
   - Size: 0.05
   - **이게 진짜 GPS 보정된 맵입니다!**

2. **Path**: `/path_gps_fused`
   - Color: 청록색
   - GPS 최적화된 경로

3. **TF**: 프레임 관계 확인

## ✅ 확인 방법

### 1. 맵 재구성 로그
```
[gps_fusion_node]: Reconstructing map from 234 scans with 234 optimized poses...
[gps_fusion_node]: Map reconstructed: 2,456,789 points from 234 scans
[gps_fusion_node]: Published GPS-corrected map with 2,456,789 points
```

### 2. 토픽 확인
```bash
# 맵이 발행되는지
ros2 topic hz /Laser_map_gps_fused
# 예상: 0.1 Hz (10초마다)

# 맵 프레임 확인
ros2 topic echo /Laser_map_gps_fused --field header.frame_id
# 출력: map

# 경로 확인
ros2 topic echo /path_gps_fused --field poses --once | wc -l
# 출력: 현재까지의 pose 개수
```

### 3. 메모리 사용량
```bash
# 노드 메모리 확인
top -p $(pgrep -f gps_fusion_node)
```

**참고**: max_scan_history가 1000이면 약 1000개 스캔 저장
- 각 스캔이 ~50,000 포인트 → 약 5천만 포인트 메모리 사용

## ⚙️ 성능 튜닝

### 맵 재구성 주기 조정
```yaml
map_update_interval: 5.0  # 5초마다 (더 자주)
map_update_interval: 30.0  # 30초마다 (덜 자주)
```

### 스캔 히스토리 크기 조정
```yaml
max_scan_history: 500   # 메모리 절약
max_scan_history: 2000  # 더 긴 히스토리
```

## 🔍 차이점 확인

**이전 (잘못된 구현)**:
- `/Laser_map_gps_fixed`는 휘어진 맵이 통째로 이동만 함
- GPS 보정 효과 없음

**현재 (올바른 구현)**:
- `/Laser_map_gps_fused`는 각 스캔이 최적화된 위치에서 재투영됨
- 맵이 정확하고 휘어지지 않음
- 진짜 GPS 보정 효과!

## 📝 로그 예시

```
[gps_fusion_node]: GPS Fusion Node initialized
[gps_fusion_node]:   Odom topic: /Odometry
[gps_fusion_node]:   GPS topic: /ublox_driver/receiver_lla
[gps_fusion_node]:   Cloud topic: /cloud_registered
[gps_fusion_node]:   Map output: /Laser_map_gps_fused
[gps_fusion_node]:   Path output: /path_gps_fused
[gps_fusion_node]:   Map update interval: 10.0 seconds
[gps_fusion_node]: GPS origin set: Lat=34.123456, Lon=-118.234567, Alt=100.50
[gps_fusion_node]: First camera_init pose saved
[gps_fusion_node]: Stored scan 1 with 45623 points (node 1)
[gps_fusion_node]: GPS factor added at node 5
[gps_fusion_node]: Optimized pose stored (node 10, total poses: 10)
[gps_fusion_node]: Reconstructing map from 10 scans with 10 optimized poses...
[gps_fusion_node]: Map reconstructed: 456,789 points from 10 scans
[gps_fusion_node]: Published GPS-corrected map with 456,789 points
```

## 🎉 이제 진짜입니다!

- ✅ 각 스캔이 최적화된 pose로 재투영
- ✅ GPS 보정이 맵에 진짜 반영됨
- ✅ 휘어지지 않은 정확한 맵
- ✅ LIO-SAM 방식과 동일한 원리

이제 **진짜** GPS 보정된 맵을 얻을 수 있습니다!
