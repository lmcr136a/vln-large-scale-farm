# VLN Large-Scale Farm

GPS 기반 농장 자율주행 시스템. Jetson AGX Orin + Scout Mini 로봇, RTK GPS, Xsens IMU, RoboSense LiDAR, ZED 스테레오 카메라 4대로 구성.

---

## 시스템 구성

```
Jetson (로봇)
  ├── RTK GPS (u-blox F9P, ROS2 노드)
  ├── Xsens IMU (ROS2 드라이버)
  ├── RoboSense LiDAR (ROS2 드라이버)
  ├── ZED 카메라 x4 (front/back/left/right)
  ├── Scout Mini (CAN 통신)
  └── tools_control_panel/jetson/jetson_main.py  ← 메인 프로세스

Lab PC (원격)
  └── tools_control_panel/lab_pc/remote_server.py  ← 웹 패널 서버

Local PC (현장 노트북, 선택)
  └── tools_control_panel/local_pc/radio_bridge_linux.py  ← 라디오 브릿지
```

---

## 실행 방법

### Jetson (로봇)
```bash
bash run_all.sh
```
tmux 세션 `vln`으로 아래 윈도우 자동 실행:
- `0 Sensors` : LiDAR + Xsens IMU + RTK GPS ROS2 노드
- `1 Claude`  : Claude Code
- `2 Main`    : `launch_files/control_panel_jetson.sh` (60초 딜레이 후 시작)
- `3 Uploader`: 데이터 자동 업로드

웹 패널: `http://localhost:8000/control.html`

### Lab PC
```bash
bash launch_files/control_panel_lab_pc.sh
```
Tailscale IP로 외부에서 접근 가능.

### Local PC (라디오 브릿지)
```bash
bash launch_files/control_panel_local_pc.sh
```

### SLAM (별도 실행, 필요 시)
```bash
bash launch_files/launch_slam.sh fargo   # fargo / outdoor / indoor / lab / oxnard
```

### 지면 분리 (Patchwork++)
```bash
bash launch_files/launch_ground_seg.sh
```
`vertical_structure_filter.py` + patchworkpp 동시 실행.

### 자동시작 (부팅 시)
```bash
bash deploy/install_autostart.sh
```
`deploy/vln.service` 등록 → 부팅 후 `run_all.sh` 자동 실행.

---

## 주요 파일

### tools_control_panel/

| 파일 | 설명 |
|------|------|
| `jetson/jetson_main.py` | 메인 진입점. 모든 모듈 연결 |
| `jetson/commander.py` | 수동/자율 모드 전환, 웹/라디오 명령 처리 |
| `jetson/internet_comm.py` | Lab PC WebSocket 통신 |
| `jetson/radio_comm.py` | 시리얼 라디오 통신 (현장 로컬 PC) |
| `jetson/station_uploader.py` | 충전 스테이션 복귀 시 SCP 자동 업로드 |
| `jetson/telemetry_node.py` | ROS2 텔레메트리 노드 |
| `gps/gps_localizer.py` | RTK GPS + Xsens IMU → ENU PoseStamped. 헤딩 퓨전 |
| `autonomous/autonomous_mode.py` | 자율주행 컨트롤러 (RTK 홀드, 스턱 복구 등) |
| `autonomous/autonomous_driving.py` | 경로 추종 제어 루프 (제너레이터) |
| `autonomous/auto_nav_logger.py` | 세션별 자율주행 로그 |
| `perception/landmark_map.py` | GPS 기반 탑뷰 맵 PNG 생성 (3초 주기) |
| `perception/landmark_detector.py` | YOLO + ZED 뎁스 → 랜드마크 탐지 |
| `perception/landmark_store.py` | 랜드마크 `landmarks.json` 저장/로드 |
| `sensor/recorder.py` | ZED 카메라 멀티 레코딩 |
| `sensor/svo_player.py` | SVO2 파일 리플레이 |
| `sensor/safety_guard.py` | 장애물 감지 시 자율주행 일시 정지/후진 |
| `sensor/safety_checker.py` | 별도 프로세스로 실행되는 안전 체크 |
| `mapping/save_map_glim_groundseg.py` | GLIM 맵 저장 (지면 분리 적용) |
| `mapping/save_map_glim.py` | GLIM 맵 저장 (지면 분리 없음) |
| `lab_pc/remote_server.py` | Lab PC 웹 서버 (WebSocket + HTTP) |
| `lab_pc/server_to_panel.py` | 웹 패널 파일 서빙 |
| `local_pc/radio_bridge_linux.py` | Linux용 라디오 브릿지 |
| `local_pc/radio_bridge_win.py` | Windows용 라디오 브릿지 |
| `web/control.html` + `control.js` | 메인 컨트롤 패널 UI |
| `web/path_plan.html` + `path_plan.js` | 경로 계획 UI |

### scripts/

| 파일 | 설명 |
|------|------|
| `rtk_gps_node.py` | RTK GPS ROS2 노드 (run_all.sh에서 실행) |
| `uploader.py` | WiFi 연결 시 `data/` Box 업로드 (run_all.sh에서 실행) |
| `vertical_structure_filter.py` | LiDAR 포인트 클라우드 수직 구조물 필터 |
| `restart_f9p.py` | u-blox F9P 재시작 유틸리티 |
| `plot_gps_xy.py` | GPS 트랙 진단용 플롯 |
| `imu_lidar_diag.py` | IMU/LiDAR 진단 플롯 |

### data/ 및 설정 파일

| 파일 | 설명 |
|------|------|
| `tools_control_panel/config/farm_config.yaml` | 메인 설정 (자율주행 파라미터, 카메라, GPS, 네트워크 등) |
| `tools_control_panel/data/crop_field.json` | 필드 경계 정의 (`ref_x`, `y_south`, `y_north`, `rotation_deg`) |
| `tools_control_panel/data/gps_origin.json` | ENU 원점 (lat/lon) |
| `tools_control_panel/data/gps_track.json` | 현재 세션 GPS 트랙 |
| `tools_control_panel/data/landmarks.json` | 탐지된 랜드마크 (lat/lon 저장) |
| `tools_control_panel/data/past_paths/` | 과거 주행 GPS 트랙 (맵에 흐린 선으로 표시) |
| `tools_control_panel/config/mission.json` | 자율주행 웨이포인트 (ENU x/y) |
| `data/crop_field.json` | `tools_control_panel/data/crop_field.json`과 동일 내용 (두 곳 동기화 필요) |

### ros2_ws/

| 워크스페이스 | 설명 |
|-------------|------|
| `scout_ws/` | Scout Mini CAN 드라이버 |
| `xsens_ws/` | Xsens MTi IMU 드라이버 |
| `robosense_ws/` | RoboSense LiDAR 드라이버 |
| `glim_ws/` | GLIM LiDAR-IMU SLAM |
| `glim_ws/config_fargo/` | **현재 사용 중인** SLAM 설정 |

---

## 사용하지 않는 것

| 항목 | 이유 |
|------|------|
| `tools_control_panel/perception/scene_describer.py` | VLM 비활성화됨 (`scene_describer = None`으로 명시) |
| `tools_control_panel/perception/_viz_landmarks.py` | 수동 진단용 스크립트, 런타임에 사용 안 함 |
| `tools_control_panel/mapping/map_creation.py` | 미완성/미사용 |
| `tools_control_panel/mapping/save_map_glim_groundseg copy.py` | 복사본, 삭제해도 무방 |
| `tools_control_panel/sensor/all_zeds.py` | 수동 진단 (동시 카메라 테스트), 런타임 미사용 |
| `tools_control_panel/sensor/snapshot_all_zeds.py` | 수동 스냅샷, 런타임 미사용 |
| `tools_control_panel/print_radio.py` | 수동 라디오 디버그 |
| `tools_control_panel/test.py` | 개발용 테스트 스크립트 |
| `scripts/vertical_structure_filter copy.py` | 복사본, 삭제해도 무방 |
| `ros2_ws/livox_driver2_ws/` | Livox LiDAR용, RoboSense 쓰므로 미사용 |
| `ros2_ws/glim_ws/config_outdoor copy/` | 설정 복사본 |
| `ros2_ws/direct_visual_lidar_calibration/` | 초기 캘리브레이션 시에만 사용 |
| `installation/` | 초기 설치 시에만 사용 |
| `run_all_local.sh` | `remote_server.py` + `radio_bridge_linux.py`만 띄우는 단순 스크립트 (현재는 lab pc / local pc 각각 개별 실행) |
| `config/local_config.yaml` | `farm_config.yaml`로 통합, 미사용 |
| `tools_control_panel/config/crop_field.json` | `tools_control_panel/data/crop_field.json`이 실제 사용됨 |

---

## 좌표계

- ENU (East-North-Up), ROS2 표준
- `x` = East (m), `y` = North (m)
- `yaw = 0` → 동쪽, `yaw = π/2` → 북쪽
- 원점: `data/gps_origin.json`에 저장된 첫 번째 GPS 수신 위치 (또는 수동 설정)

## 맵 설정 (`crop_field.json`)

```json
{
  "ref_x":        -2.35,   // 초록 줄 시작 x (East, m) — 오른쪽으로 갈수록 증가
  "y_south":     -54.0,    // 필드 남쪽 경계 (m)
  "y_north":      72.7,    // 필드 북쪽 경계 (m)
  "rotation_deg": 0.5      // 줄 방향 회전각 (도) — 실제 이랑 방향 보정
}
```

`tools_control_panel/data/crop_field.json`과 `data/crop_field.json` 두 곳을 항상 같이 수정할 것.
