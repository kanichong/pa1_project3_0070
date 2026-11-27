# pa1_project3_0070

프로젝트 3 — 재활용품 분리수거 미션

Waffle Pi + OpenMANIPULATOR-X 팀 레포. 지금은 큰 틀과 로봇 모델(URDF)만 들어 있다.

## 폴더

| 폴더 | 내용 | 상태 |
|---|---|---|
| `turtlebot3_manipulation_description/` | 로봇 URDF 패키지 (xacro, 메시, ros2_control, launch) | 들어 있음 |
| `perception/` | 재활용품 인식 (카메라, 검출/VLM) | 비어 있음 |
| `manipulation/` | 집기·놓기 (팔, 그리퍼) | 비어 있음 |
| `navigation/` | 주행, 지도, 분리수거함 위치 | 비어 있음 |
| `mission/` | 전체 순서 (명령 → 인식 → 집기 → 이동 → 버리기) | 비어 있음 |
| `config/` | 파라미터 | 비어 있음 |
| `docs/` | 미션 규칙, 역할 분담, 실험 기록 | 비어 있음 |

## 로봇 모델 (`turtlebot3_manipulation_description`)

- Onshape로 만든 팀 팔 메시를 쓴다. 손목 회전(J5) 모터 없음, 그리퍼는 크랭크·로드 링크 포함, 라이다는 LDS-03 메시.
- ROS 2 패키지라서 워크스페이스 `src/` 아래에 두고 빌드한다.
- 이 패키지만으로는 로봇이 안 움직인다. 튜터 레포(`SpartaPA/waffle-mpx-lyrical`)의 아래 패키지가 같이 필요하다.
  - `turtlebot3_manipulation_bringup`, `turtlebot3_manipulation_moveit_config`
  - `turtlebot3_manipulation_hardware` (`ros2_control` xacro가 이 플러그인을 부른다)
  - `topic_based_ros2_control` (Isaac 시뮬 모드)
- 튜터 레포의 같은 이름 패키지와 한 워크스페이스에 같이 두면 이름이 겹친다. 둘 중 하나만 빌드한다.

### 그리퍼 끝 값 (실측 아님)

`urdf/open_manipulator_x.urdf.xacro`, link5(joint4 회전축) 기준 x 방향:

| 항목 | 이 URDF | 튜터 RM-X52 (`SpartaPA/waffle_rmx52_261127`) |
|---|---|---|
| `end_effector_joint` | 135.3 mm | 126.0 mm |
| 손가락 메시 맨 끝 | 145.3 mm | 146.6 mm |
| 손가락 안쪽 패드 중심 | 약 124.8 mm | 약 126.3 mm |
| 손가락 joint origin (x, ±y) | 70.6, ±28.3 mm | 81.7, ±21 mm |
| 손가락 한계 | −3 .. 20 mm | −10 .. 19 mm |

- 위 값은 모두 CAD/메시에서 계산한 값이고 **실물을 자로 잰 값이 아니다.**
- 이 URDF의 `end_effector`(135.3)는 "손가락 끝에서 10 mm 안쪽"으로 만든 값이다. RM-X52의 126은 패드 중심이다.
- 실물에서 joint4 회전축 중심 → 닫힌 손가락 맨 끝을 재서 145 mm 근처인지 확인한 뒤 쓰는 게 안전하다.

## 올리지 않는 것 (`.gitignore`)

- 모델 가중치, 체크포인트, 학습 데이터, rosbag
- 와이파이 비밀번호, 토큰, 키, 개인 로봇 IP가 적힌 설정

## 출처 / 라이선스

`turtlebot3_manipulation_description`은 ROBOTIS `turtlebot3_manipulation`(Apache 2.0)을 튜터 레포가 고친 것을 다시 팀 팔에 맞게 고친 것이다.
