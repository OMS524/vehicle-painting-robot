# Painting Trajectory Reviewer

차량 부품의 PLY 표면, 도장 trajectory, 분사 상태, planner 파라미터를 오프라인에서 확인하는 읽기 전용 데스크톱 애플리케이션이다.

현재 버전은 검토 기능에 집중하며 경로 데이터에 안정적인 row/point ID를 부여한다. 이후 같은 데이터 모델에 점 이동, 행 이동, 분사 상태 변경, Undo/Redo 및 재계산 기능을 추가할 수 있다.

## 구성

```text
path_editor/
├── src/                 # React + TypeScript + Three.js 화면
├── src-tauri/           # Windows/Linux 데스크톱 런타임과 안전한 파일 접근
└── data/
    └── <project_name>/
        ├── <surface>.ply
        ├── painting_trajectory.csv
        └── painting_trajectory.yaml
```

각 프로젝트에는 PLY 파일 한 개, `painting_trajectory.csv`,
`painting_trajectory.yaml`이 필요하다. 프로젝트를 바꾸면 해당 디렉터리의
YAML도 함께 불러온다. `trajectory_*.csv`와 기존 debug HTML은 Reviewer가 읽지 않는다.

## Ubuntu 완성 앱 실행

`Painting_Trajectory_Reviewer_0.1.0_amd64.AppImage`와 `data` 디렉터리를 같은
디렉터리에 둔다. 파일 관리자에서 AppImage를 더블클릭하거나 다음과 같이 실행한다.

```bash
./Painting_Trajectory_Reviewer_0.1.0_amd64.AppImage
```

AppImage는 로컬 `data`를 직접 읽으며 별도 서버나 브라우저 주소를 사용하지 않는다.
계약사에 전달할 때도 AppImage와 `data` 디렉터리를 함께 전달한다.

## 개발 실행

필요한 시스템 패키지, Node.js LTS 및 Rust stable 설치 후 실행한다.

Ubuntu 22.04 이상:

```bash
sudo apt update
sudo apt install libwebkit2gtk-4.1-dev build-essential curl wget file \
  libxdo-dev libssl-dev libayatana-appindicator3-dev librsvg2-dev \
  patchelf libfuse2
```

Node.js는 LTS 버전, Rust는 `rustup`의 stable toolchain을 사용한다.

```bash
npm install
npm run tauri dev
```

Linux 시스템 패키지가 준비되지 않은 개발 환경에서는 `npm run dev`로 브라우저 미리보기를 실행할 수 있다. 이때 Vite 개발 서버가 동일한 `data` 디렉터리를 읽으며, production 앱의 Tauri command와 같은 응답 형식을 사용한다.

이 저장소에 설치된 로컬 Node.js와 Rust를 사용하는 Ubuntu 실행 예시는 다음과 같다.

```bash
cd /home/oms/vehicle_painting_robot/painting_trajectory_planner/path_editor
export PATH="$HOME/.local/node-v22.17.0-linux-x64/bin:$HOME/.cargo/bin:$PATH"
npm run dev
```

브라우저에서 `http://127.0.0.1:1420`을 연다. 위 시스템 패키지를 설치한 뒤에는
마지막 명령을 `npm run tauri dev`로 바꾸면 데스크톱 창으로 실행된다.

## 빌드

Linux에서는 AppImage와 deb 패키지를 생성한다.

```bash
npm run tauri build
```

Windows 빌드는 같은 소스를 Windows 환경에서 실행해 생성한다. Windows와 Linux 프로젝트 데이터 형식은 동일하다.

## 데이터 위치

개발 중에는 이 디렉터리의 `data`를 자동으로 찾는다. 배포 시에는 실행 파일 또는 AppImage 옆에 동일한 `data` 디렉터리를 둔다. 별도 위치를 사용할 경우 다음 환경변수를 지정할 수 있다.

```bash
PATH_EDITOR_DATA_DIR=/absolute/path/to/data
```

데이터 접근은 Rust command에서 프로젝트명과 파일 종류를 검증하며, 화면에서 임의의 파일 시스템 경로를 읽을 수 없다.
