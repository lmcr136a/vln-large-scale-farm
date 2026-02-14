# GPS Optimization - FAST-LIO2 및 GPS 융합 패키지

ROS 2 Humble 환경에서 FAST-LIO2 LiDAR 오도메트리와 u-blox GPS 데이터를 GTSAM을 이용하여 융합하는 패키지입니다.

## 주요 기능

- **FAST-LIO2 오도메트리와 GPS 데이터 융합**: GTSAM의 Factor Graph를 이용한 최적화
- **ISAM2 증분 최적화**: 실시간 성능을 위한 효율적인 최적화
- **강건한 GPS 처리**: 노이즈가 큰 GPS 신호에도 LiDAR 오도메트리 궤적이 망가지지 않도록 설계
- **유연한 설정**: YAML 파일을 통한 토픽 이름 및 파라미터 변경
- **TF 브로드캐스팅**: map과 odom 프레임 간의 변환 제공

## 시스템 요구사항

- Ubuntu 22.04
- ROS 2 Humble
- GTSAM 4.0+
- GeographicLib
- Eigen3

## 의존성 설치

### 1. GTSAM 설치

```bash
# 필요한 패키지 설치
sudo apt-get install -y libboost-all-dev libtbb-dev

# GTSAM 소스 다운로드 및 빌드
git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF
make -j$(nproc)
sudo make install
```

### 2. GeographicLib 설치

```bash
sudo apt-get install -y libgeographic-dev
```

### 3. Eigen3 설치 (일반적으로 이미 설치되어 있음)

```bash
sudo apt-get install -y libeigen3-dev
```

## 패키지 빌드

```bash
# ROS 2 워크스페이스로 이동
cd ~/ros2_ws/src

# 이 패키지를 복사
cp -r gps_optimization .

# 워크스페이스 루트로 이동하여 빌드
cd ~/ros2_ws
colcon build --packages-select gps_optimization

# 환경 설정
source install/setup.bash
```

## 사용 방법

### 1. 설정 파일 수정 (선택사항)

`config/config.yaml` 파일을 열어 토픽 이름과 파라미터를 수정할 수 있습니다:

```yaml
odom_topic: "/Odometry"              # FAST-LIO2 오도메트리 토픽
gps_topic: "/ublox_driver/receiver_lla"  # GPS NavSatFix 토픽
output_odom_topic: "/odometry/gps_fused"  # 융합된 결과 토픽

gps_noise_x: 2.0  # GPS 가로 방향 노이즈 (미터)
gps_noise_y: 2.0  # GPS 세로 방향 노이즈 (미터)
gps_noise_z: 4.0  # GPS 수직 방향 노이즈 (미터)
```

### 2. 노드 실행

#### Launch 파일 사용 (권장)

```bash
ros2 launch gps_optimization gps_fusion.launch.py
```

#### 직접 실행

```bash
ros2 run gps_optimization gps_fusion_node --ros-args --params-file config/config.yaml
```

### 3. 입력 토픽

노드는 다음 토픽을 구독합니다:

- `/Odometry` (nav_msgs/msg/Odometry): FAST-LIO2가 발행하는 고주파 오도메트리
- `/ublox_driver/receiver_lla` (sensor_msgs/msg/NavSatFix): u-blox GPS 수신기의 위치 데이터

### 4. 출력 토픽

노드는 다음을 발행합니다:

- `/odometry/gps_fused` (nav_msgs/msg/Odometry): GPS와 융합된 최적화된 오도메트리
- TF: `map` → `odom` 변환

## 작동 원리

1. **GPS 원점 설정**: 첫 번째 유효한 GPS 신호를 받으면 해당 위치를 ENU(East-North-Up) 좌표계의 원점으로 설정합니다.

2. **오도메트리 처리**: 
   - FAST-LIO2의 각 오도메트리 메시지마다 새로운 노드를 Factor Graph에 추가
   - 이전 노드와의 상대 변환을 BetweenFactor로 표현

3. **GPS 융합**: 
   - GPS 신호가 들어오면 해당 시간과 가장 가까운 오도메트리 노드에 GPSFactor 추가
   - 큰 노이즈 모델 (가로 2m, 세로 4m)을 사용하여 GPS가 튀어도 궤적이 안정적으로 유지

4. **최적화**: 
   - GTSAM의 ISAM2를 사용한 증분 최적화
   - 실시간 성능을 위해 효율적으로 업데이트

5. **결과 발행**: 
   - 최적화된 Pose를 Odometry 메시지로 발행
   - map-odom TF 브로드캐스트

## 시각화

RViz2를 사용하여 결과를 시각화할 수 있습니다:

```bash
rviz2
```

다음을 추가하세요:
- **Odometry**: `/odometry/gps_fused` 토픽 (융합된 궤적)
- **Odometry**: `/Odometry` 토픽 (원본 FAST-LIO2 궤적, 비교용)
- **TF**: 프레임 변환 시각화

## 문제 해결

### GPS 원점이 설정되지 않음
- GPS 수신 상태 확인: `ros2 topic echo /ublox_driver/receiver_lla`
- GPS fix 상태가 0 이상인지 확인

### 최적화가 느림
- config.yaml에서 노이즈 파라미터 조정
- ISAM2 파라미터 튜닝 (`gps_fusion_node.cpp`의 ISAM2Params)

### GPS가 튀어도 궤적이 이상함
- `gps_noise_x`, `gps_noise_y`, `gps_noise_z` 값을 더 크게 설정하여 GPS 영향을 줄임

## 파일 구조

```
gps_optimization/
├── CMakeLists.txt           # 빌드 설정
├── package.xml              # 패키지 메타데이터
├── README.md                # 이 파일
├── config/
│   └── config.yaml          # 파라미터 설정 파일
├── include/
│   └── gps_optimization/
│       └── GNSS_Processing.hpp  # GPS 처리 클래스 (ROS 2 호환)
├── launch/
│   └── gps_fusion.launch.py     # Launch 파일
└── src/
    └── gps_fusion_node.cpp      # 메인 노드 소스 코드
```

## 참고 자료

- [GTSAM Documentation](https://gtsam.org/)
- [GeographicLib](https://geographiclib.sourceforge.io/)
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [LIO-SAM](https://github.com/TixiaoShan/LIO-SAM)

## 라이선스

MIT License

## 기여

이슈 및 풀 리퀘스트를 환영합니다!
