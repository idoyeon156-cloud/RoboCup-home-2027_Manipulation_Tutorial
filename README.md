# Jetson 최신 관제센터 소스 사본

2026-10-15 실행 중인 Jetson의 `SW/control_center/`를 읽기 전용으로 복사했다. 원본 서버, 센서, 네비게이션, 팔의 소스·설정·프로세스는 변경하지 않았다.

- 원본 프로젝트: https://github.com/kimsh039/INHA-RoboCup-Home-2th-2027
- 원본 Jetson checkout HEAD: `cd59540` (폴더 내 미커밋 수정 및 미추적 소스 포함)
- 원본 관제센터 설명: [SW/control_center/README.md](SW/control_center/README.md)
- 디텍션·접근 설명: [SW/control_center/DETECTION.md](SW/control_center/DETECTION.md)
- 원본 작성자들의 코드 사본이다. 미커밋 코드의 작성자는 Git 기록만으로 확인하지 못했으며, 이 복사를 새 저작권/라이선스 부여로 해석하지 않는다.

## 포함 및 제외

웹 화면, HTTP 서버, ROS bridge, YOLO+MID-360 좌표/접근 코드, 시작·설치 스크립트, 예제 설정을 포함한다. 실제 `config.json`, 설정 백업, Python 캐시, 실행 로그, SLAM 지도·누적 점군은 복사하지 않았다. 소스별 SHA-256은 `SOURCE_MANIFEST.sha256`에 기록했다.

## 실행 의존성

이 저장소는 관제센터 폴더의 소스 스냅샷이며 독립된 로봇 전체 설치본이 아니다. 기존 실행기는 같은 프로젝트의 `SW/robot` (physical.launch.py, 충돌 보호/SLAM/TF 등) 및 `SW/detection/head` (YOLO backend, ROS overlay), `SW/setup`, Jetson의 ROS Humble·센서 드라이버·GPU 가상환경·모델·캘리브레이션을 재사용한다. 해당 디렉터리·모델·설치 산출물은 이번 사본에 포함하지 않았다. 원본 팀 checkout에 덮어쓰거나 실행 중인 서버를 재시작하지 않았다.

새 환경에서 이 폴더만 실행하면 전체 센서·주행·디텍션을 구동할 수 없다. 의존 소스/설정은 원본 프로젝트와 실제 장치 구성을 기준으로 별도 준비한다. 시작 스크립트는 하드웨어 프로세스를 시작/종료할 수 있으므로 파일 검토 없이 실행하지 않는다.

## 복사 후 검사

Python AST, shell 문법, JSON 예제 형식을 검사했다. 서버·ROS·센서·하드웨어 실행 검증은 하지 않았다. 관제센터 입력과 매니퓰레이션의 완전한 시스템 통합 성공을 의미하지 않는다.
