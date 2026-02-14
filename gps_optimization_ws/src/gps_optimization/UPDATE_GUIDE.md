# GPS Optimization 업데이트 - 맵 재생성 기능 추가

## 변경된 파일 목록

업데이트된 파일만 복사하여 붙여넣으면 됩니다:

### 1. `src/gps_fusion_node.cpp` (전체 교체)
- **추가된 기능**:
  - `/Laser_map` 구독하여 GPS 보정된 맵 생성
  - `/Laser_map_gps_fixed` 발행 (map 프레임 기준)
  - `/path_gps_fused` 발행 (GPS 보정된 경로)
  - TF listener 추가 (camera_init → map 변환)
  
### 2. `config/config.yaml` (일부 수정)
- **추가된 파라미터**:
  ```yaml
  laser_map_topic: "/Laser_map"
  laser_map_gps_fixed_topic: "/Laser_map_gps_fixed"
  path_topic: "/path"
  path_gps_fused_topic: "/path_gps_fused"
  camera_init_frame: "camera_init"
  ```

### 3. `CMakeLists.txt` (일부 수정)
- **추가된 의존성**:
  - `tf2_eigen` (TF 변환용)
  - `PCL` (Point Cloud 처리용)

### 4. `package.xml` (일부 수정)
- **추가된 의존성**:
  ```xml
  <depend>tf2_eigen</depend>
  <depend>libpcl-all-dev</depend>
  <depend>pcl_conversions</depend>
  ```

## 실행 방법

### 1. 빌드
```bash
cd ~/box/vln-large-scale-farm/gps_optimization_ws
colcon build --packages-select gps_optimization --symlink-install
source install/setup.bash
```

### 2. Static TF 실행 (이미 실행 중)
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map camera_init
```

### 3. GPS Fusion 노드 실행
```bash
ros2 launch gps_optimization gps_fusion.launch.py
```

## RViz2 시각화 설정

### Fixed Frame
```
Fixed Frame: map
```

### 추가할 디스플레이

#### 1. 오도메트리 비교
- **원본**: `/Odometry` (camera_init 프레임)
  - Add → Odometry
  - Topic: `/Odometry`
  - Color: 빨강
  
- **GPS 보정**: `/odometry/gps_fused` (map 프레임)
  - Add → Odometry
  - Topic: `/odometry/gps_fused`
  - Color: 파랑

#### 2. 맵 비교
- **원본 맵**: `/Laser_map` (camera_init 프레임)
  - Add → PointCloud2
  - Topic: `/Laser_map`
  - Color: 회색
  - Size: 0.05
  
- **GPS 보정 맵**: `/Laser_map_gps_fixed` (map 프레임)
  - Add → PointCloud2
  - Topic: `/Laser_map_gps_fixed`
  - Color: 녹색
  - Size: 0.05

#### 3. 경로 비교
- **원본 경로**: `/path` (camera_init 프레임)
  - Add → Path
  - Topic: `/path`
  - Color: 주황
  
- **GPS 보정 경로**: `/path_gps_fused` (map 프레임)
  - Add → Path
  - Topic: `/path_gps_fused`
  - Color: 청록

#### 4. TF 트리
- Add → TF
  - 전체 프레임 관계 시각화

## 예상 TF 트리

```
map (GPS 전역 좌표계)
├── camera_init (static, FAST-LIO2 world)
│   └── body/base_link (FAST-LIO2가 발행)
└── odom (GPS fusion 노드가 발행)
```

## 토픽 목록

### 입력 토픽
- `/Odometry` - FAST-LIO2 오도메트리 (camera_init 프레임)
- `/ublox_driver/receiver_lla` - GPS 데이터
- `/Laser_map` - FAST-LIO2 누적 맵 (camera_init 프레임)
- `/path` - FAST-LIO2 경로 (camera_init 프레임)

### 출력 토픽
- `/odometry/gps_fused` - GPS 보정된 오도메트리 (map 프레임)
- `/Laser_map_gps_fixed` - GPS 보정된 맵 (map 프레임)
- `/path_gps_fused` - GPS 보정된 경로 (map 프레임)

## 작동 원리

### 맵 재생성 방식
1. FAST-LIO2의 `/Laser_map` 구독 (camera_init 프레임)
2. TF를 사용하여 `camera_init → map` 변환 획득
3. Point Cloud 전체를 map 프레임으로 변환
4. `/Laser_map_gps_fixed`로 발행

### 경로 생성 방식
1. 오도메트리 콜백마다 최적화된 pose를 저장
2. Path 메시지에 누적
3. `/path_gps_fused`로 발행

## 확인 방법

```bash
# 토픽 확인
ros2 topic list | grep -E "Laser_map|path|odometry"

# 원본 맵
ros2 topic echo /Laser_map --field header.frame_id
# 출력: camera_init

# GPS 보정 맵
ros2 topic echo /Laser_map_gps_fixed --field header.frame_id
# 출력: map

# TF 확인
ros2 run tf2_ros tf2_echo map camera_init
ros2 run tf2_ros tf2_echo map odom
```

## 문제 해결

### 맵이 보이지 않음
- FAST-LIO2가 `/Laser_map`을 발행하는지 확인
- static TF (map → camera_init)가 실행 중인지 확인

### 맵이 이상한 위치에 표시됨
- TF 트리 확인: `ros2 run tf2_tools view_frames`
- GPS 원점이 설정되었는지 로그 확인

### 경로가 표시되지 않음
- GPS 원점 설정 후 로봇이 움직이고 있는지 확인
- `/odometry/gps_fused` 토픽이 발행되는지 확인
