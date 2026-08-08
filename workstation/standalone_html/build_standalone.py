#!/usr/bin/env python3
"""부품 데이터와 Reviewer 프론트엔드를 하나의 오프라인 HTML로 묶는다.

기본 실행:
    python3 build_standalone.py

특정 부품만 생성:
    python3 build_standalone.py ES300h_hood ES300h_trunk

이미 빌드된 dist를 사용해 데이터만 다시 내장:
    python3 build_standalone.py --skip-frontend-build
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path


SCRIPT_DIR = Path(__file__).resolve().parent
EDITOR_ROOT = SCRIPT_DIR.parent
DEFAULT_DATA_ROOT = EDITOR_ROOT / "data"
DEFAULT_DIST_ROOT = EDITOR_ROOT / "dist"
DEFAULT_OUTPUT_ROOT = SCRIPT_DIR
APP_MARKER = "__STANDALONE_PROJECT_AND_APP__"


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="PLY, 전체 경로 CSV, YAML과 Reviewer UI를 단일 HTML로 생성합니다.",
    )
    parser.add_argument(
        "projects",
        nargs="*",
        metavar="PROJECT",
        help="생성할 data 하위 디렉토리명. 생략하면 모든 부품을 생성합니다.",
    )
    parser.add_argument(
        "--skip-frontend-build",
        action="store_true",
        help="npm 빌드를 생략하고 현재 dist의 JavaScript/CSS를 사용합니다.",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=DEFAULT_DATA_ROOT,
        help=f"부품 데이터 루트. 기본값: {DEFAULT_DATA_ROOT}",
    )
    parser.add_argument(
        "--dist-root",
        type=Path,
        default=DEFAULT_DIST_ROOT,
        help=f"Vite 빌드 결과 루트. 기본값: {DEFAULT_DIST_ROOT}",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help=f"HTML 저장 디렉토리. 기본값: {DEFAULT_OUTPUT_ROOT}",
    )
    return parser.parse_args()


def find_npm() -> Path | None:
    npm_on_path = shutil.which("npm")
    if npm_on_path:
        return Path(npm_on_path).absolute()

    local_candidates = list(
        (Path.home() / ".local").glob("node-*-linux-*/bin/npm"),
    )
    local_candidates.extend((Path.home() / ".nvm/versions/node").glob("*/bin/npm"))
    candidates = [path for path in local_candidates if path.is_file()]
    if not candidates:
        return None
    return max(candidates, key=lambda path: path.stat().st_mtime).absolute()


def build_frontend(dist_root: Path) -> None:
    npm = find_npm()
    if npm is None:
        if (dist_root / "index.html").is_file():
            print(
                "[standalone] npm을 찾지 못해 기존 dist를 사용합니다. "
                "최신 UI가 필요하면 Node.js/npm을 설치하거나 PATH에 추가하세요.",
                file=sys.stderr,
            )
            return
        raise RuntimeError("npm과 기존 dist/index.html을 모두 찾지 못했습니다.")

    environment = os.environ.copy()
    environment["PATH"] = f"{npm.parent}{os.pathsep}{environment.get('PATH', '')}"
    print(f"[standalone] 프론트엔드 빌드: {npm} run build")
    subprocess.run(
        [str(npm), "run", "build"],
        cwd=EDITOR_ROOT,
        env=environment,
        check=True,
    )


def require_match(source: str, pattern: str, description: str) -> re.Match[str]:
    match = re.search(pattern, source)
    if match is None:
        raise RuntimeError(f"dist/index.html에서 {description}을 찾지 못했습니다.")
    return match


def safe_inline_script(source: str) -> str:
    return source.replace("</script", "<\\/script")


def safe_inline_style(source: str) -> str:
    return source.replace("</style", "<\\/style")


def read_frontend_bundle(dist_root: Path) -> tuple[str, str]:
    index_path = dist_root / "index.html"
    if not index_path.is_file():
        raise FileNotFoundError(
            f"{index_path}이 없습니다. 먼저 프론트엔드를 빌드해야 합니다.",
        )

    index_source = index_path.read_text(encoding="utf-8")
    script_match = require_match(
        index_source,
        r'<script type="module" crossorigin src="([^"]+)"></script>',
        "JavaScript 번들",
    )
    style_match = require_match(
        index_source,
        r'<link rel="stylesheet" crossorigin href="([^"]+)">',
        "CSS 번들",
    )
    script_path = dist_root / script_match.group(1).lstrip("/")
    style_path = dist_root / style_match.group(1).lstrip("/")
    app_script = script_path.read_text(encoding="utf-8")
    app_style = style_path.read_text(encoding="utf-8")
    frontend_template = index_source.replace(
        style_match.group(0),
        f"<style>{safe_inline_style(app_style)}</style>",
    ).replace(script_match.group(0), APP_MARKER)

    if APP_MARKER not in frontend_template:
        raise RuntimeError("standalone 앱 삽입 위치를 만들지 못했습니다.")
    return frontend_template, app_script


def discover_projects(data_root: Path, requested: list[str]) -> list[str]:
    data_root = data_root.resolve()
    if not data_root.is_dir():
        raise NotADirectoryError(f"데이터 디렉토리가 아닙니다: {data_root}")

    if requested:
        project_names = requested
    else:
        project_names = sorted(
            path.name for path in data_root.iterdir() if path.is_dir()
        )

    if not project_names:
        raise RuntimeError("standalone HTML을 생성할 프로젝트가 없습니다.")

    for project_name in project_names:
        project_dir = (data_root / project_name).resolve()
        if project_dir.parent != data_root or not project_dir.is_dir():
            raise RuntimeError(f"유효한 data 하위 프로젝트가 아닙니다: {project_name}")
    return project_names


def find_project_files(data_root: Path, project_name: str) -> tuple[Path, Path, Path]:
    project_dir = data_root / project_name
    surfaces = sorted(
        (path for path in project_dir.iterdir() if path.suffix.lower() == ".ply"),
        key=lambda path: path.name.casefold(),
    )
    surface_path = surfaces[0] if surfaces else None
    trajectory_path = project_dir / "painting_trajectory.csv"
    parameters_path = project_dir / "painting_trajectory.yaml"
    missing = [
        name
        for name, path in (
            ("PLY", surface_path),
            ("painting_trajectory.csv", trajectory_path),
            ("painting_trajectory.yaml", parameters_path),
        )
        if path is None or not path.is_file()
    ]
    if missing:
        raise RuntimeError(f"{project_name}: 필요한 파일이 없습니다: {', '.join(missing)}")
    assert surface_path is not None
    return surface_path, trajectory_path, parameters_path


def output_file_name(project_name: str) -> str:
    safe_name = re.sub(r"[^a-zA-Z0-9_.-]", "_", project_name)
    if not safe_name or safe_name in {".", ".."}:
        raise RuntimeError(
            f"HTML 파일명으로 사용할 수 없는 프로젝트명입니다: {project_name}",
        )
    return f"{safe_name}.html"


def encode_file(path: Path) -> str:
    return base64.b64encode(path.read_bytes()).decode("ascii")


def build_project_html(
    frontend_template: str,
    app_script: str,
    data_root: Path,
    output_root: Path,
    project_name: str,
) -> tuple[Path, int]:
    surface_path, trajectory_path, parameters_path = find_project_files(
        data_root,
        project_name,
    )
    embedded_project = {
        "info": {
            "name": project_name,
            "surfaceFile": surface_path.name,
            "trajectoryFile": trajectory_path.name,
            "surfaceBytes": surface_path.stat().st_size,
            "trajectoryBytes": trajectory_path.stat().st_size,
        },
        "surfaceBase64": encode_file(surface_path),
        "trajectoryBase64": encode_file(trajectory_path),
        "parametersBase64": encode_file(parameters_path),
        "sourceLabel": f"Embedded standalone HTML · {project_name}",
    }
    project_json = json.dumps(
        embedded_project,
        ensure_ascii=False,
        separators=(",", ":"),
    )
    project_script = safe_inline_script(
        f"window.__PAINTING_TRAJECTORY_STANDALONE__={project_json};",
    )
    document = frontend_template.replace(
        "<title>Painting Process Workstation</title>",
        f"<title>{html.escape(project_name, quote=True)} · "
        "Painting Process Workstation</title>",
    ).replace(
        APP_MARKER,
        f"<script>{project_script}</script>"
        f'<script type="module">{safe_inline_script(app_script)}</script>',
    )
    output_root.mkdir(parents=True, exist_ok=True)
    output_path = output_root / output_file_name(project_name)
    output_path.write_text(document, encoding="utf-8")
    return output_path, len(document.encode("utf-8"))


def main() -> int:
    arguments = parse_arguments()
    data_root = arguments.data_root.expanduser().resolve()
    dist_root = arguments.dist_root.expanduser().resolve()
    output_root = arguments.output_root.expanduser().resolve()

    if not arguments.skip_frontend_build:
        build_frontend(dist_root)

    frontend_template, app_script = read_frontend_bundle(dist_root)
    project_names = discover_projects(data_root, arguments.projects)
    for project_name in project_names:
        output_path, byte_count = build_project_html(
            frontend_template,
            app_script,
            data_root,
            output_root,
            project_name,
        )
        print(
            f"[standalone] {project_name}: {output_path} "
            f"({byte_count / 1024 / 1024:.1f} MiB)",
        )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, subprocess.CalledProcessError) as error:
        print(f"[standalone] ERROR: {error}", file=sys.stderr)
        raise SystemExit(1) from error
