# GPS Optimization 패키지 - 빠른 설치 가이드

## 1. 압축 파일 다운로드 및 압축 해제

다운로드한 `gps_optimization.tar.gz` 파일을 ROS 2 워크스페이스의 src 디렉토리에 압축 해제합니다:

```bash
cd ~/ros2_ws/src
tar -xzf ~/Downloads/gps_optimization.tar.gz
```

## 2. 의존성 설치

### GTSAM 설치 (약 10-15분 소요)

```bash
sudo apt-get install -y libboost-all-dev libtbb-dev

git clone https://github.com/borglab/gtsam.git
cd gtsam
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF
make -j$(nproc)
sudo make install
cd ../..
```

### GeographicLib 설치

```bash
sudo apt-get install -y libgeographic-dev
```

### Eigen3 설치 (보통 이미 설치되어 있음)

```bash
sudo apt-get install -y libeigen3-dev
```

## 3. 패키지 빌드

```bash
cd ~/ros2_ws
colcon build --packages-select gps_optimization
source install/setup.bash
```

## 4. 설정 파일 수정 (필요시)

토픽 이름을 변경하려면 `config/config.yaml` 파일을 수정하세요:

```bash
nano ~/ros2_ws/src/gps_optimization/config/config.yaml
```

주요 설정:
- `odom_topic`: FAST-LIO2 오도메트리 토픽 이름
- `gps_topic`: GPS NavSatFix 토픽 이름
- `gps_noise_x/y/z`: GPS 노이즈 모델 (기본값: 2m, 2m, 4m)

## 5. 실행

```bash
# Launch 파일 사용 (권장)
ros2 launch gps_optimization gps_fusion.launch.py

# 또는 직접 실행
ros2 run gps_optimization gps_fusion_node
```

## 6. 확인

다른 터미널에서 토픽 확인:

```bash
# 입력 토픽 확인
ros2 topic echo /Odometry
ros2 topic echo /ublox_driver/receiver_lla

# 출력 토픽 확인
ros2 topic echo /odometry/gps_fused
```

## 7. 시각화 (RViz2)

```bash
rviz2
```

RViz2에서 다음을 추가:
1. Fixed Frame: "map"
2. Add → Odometry → Topic: `/odometry/gps_fused`
3. Add → Odometry → Topic: `/Odometry` (비교용)
4. Add → TF (프레임 변환 시각화)

## 문제 해결

**Q: GPS 원점이 설정되지 않습니다**
A: GPS 신호가 수신되는지 확인하세요. `ros2 topic echo /ublox_driver/receiver_lla`로 확인 가능합니다.

**Q: 컴파일 에러가 발생합니다**
A: GTSAM이 제대로 설치되었는지 확인하세요. `pkg-config --modversion gtsam` 명령어로 확인 가능합니다.

**Q: GPS가 튀어도 궤적이 흔들립니다**
A: config.yaml에서 `gps_noise_x`, `gps_noise_y`, `gps_noise_z` 값을 더 크게 설정하세요 (예: 5.0, 5.0, 10.0).

## 상세 정보

자세한 내용은 `README.md` 파일을 참조하세요.

## 주요 파일 위치

- 소스 코드: `src/gps_fusion_node.cpp`
- GPS 처리 클래스: `include/gps_optimization/GNSS_Processing.hpp`
- 설정 파일: `config/config.yaml`
- Launch 파일: `launch/gps_fusion.launch.py`
