#!/usr/bin/env python3

import argparse
import csv
import html
import json
import math
import tempfile
import webbrowser
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CSV_PATH = (
    PROJECT_ROOT
    / "painting_trajectory_planner/test/csv/car_door_small/painting_trajectory.csv"
)

POSITION_COLUMN_CANDIDATES = (
    ("x", "y", "z"),
    ("position_x", "position_y", "position_z"),
    ("px", "py", "pz"),
)


def _normalized_field_map(fieldnames: list[str]) -> dict[str, str]:
    return {field.strip().lower(): field for field in fieldnames if field is not None}


def _find_position_columns(fieldnames: list[str]) -> tuple[str, str, str]:
    normalized = _normalized_field_map(fieldnames)
    for candidate in POSITION_COLUMN_CANDIDATES:
        if all(name in normalized for name in candidate):
            return tuple(normalized[name] for name in candidate)
    raise ValueError(
        "CSV에서 좌표 컬럼을 찾지 못했습니다. "
        "x,y,z 또는 position_x,position_y,position_z 컬럼이 필요합니다."
    )


def _float_or_none(value: str | None) -> float | None:
    if value is None:
        return None
    text = value.strip()
    if not text:
        return None
    try:
        number = float(text)
    except ValueError:
        return None
    if not math.isfinite(number):
        return None
    return number


def load_points(csv_path: Path) -> tuple[list[dict], dict]:
    with csv_path.open("r", newline="", encoding="utf-8-sig") as csv_file:
        reader = csv.DictReader(csv_file)
        if not reader.fieldnames:
            raise ValueError("CSV 헤더가 없습니다.")

        x_col, y_col, z_col = _find_position_columns(reader.fieldnames)
        normalized = _normalized_field_map(reader.fieldnames)
        path_col = normalized.get("path_index") or normalized.get("row_index")
        paint_col = normalized.get("paint")

        points = []
        skipped_rows = 0
        for row in reader:
            x = _float_or_none(row.get(x_col))
            y = _float_or_none(row.get(y_col))
            z = _float_or_none(row.get(z_col))
            if x is None or y is None or z is None:
                skipped_rows += 1
                continue

            point = {"x": x, "y": y, "z": z}
            if path_col is not None:
                point["path"] = row.get(path_col, "")
            if paint_col is not None:
                point["paint"] = row.get(paint_col, "")
            points.append(point)

    if not points:
        raise ValueError("시각화할 수 있는 좌표 데이터가 없습니다.")

    metadata = {
        "source": str(csv_path),
        "point_count": len(points),
        "skipped_rows": skipped_rows,
        "columns": {"x": x_col, "y": y_col, "z": z_col},
        "has_path": path_col is not None,
        "has_paint": paint_col is not None,
    }
    return points, metadata


def build_html(points: list[dict], metadata: dict) -> str:
    points_json = json.dumps(points, ensure_ascii=False, separators=(",", ":"))
    metadata_json = json.dumps(metadata, ensure_ascii=False, separators=(",", ":"))
    title = html.escape(Path(metadata["source"]).name)

    return f"""<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>CSV Point Viewer - {title}</title>
  <style>
    html, body {{
      width: 100%;
      height: 100%;
      margin: 0;
      overflow: hidden;
      background: #111111;
      color: #f2f2f2;
      font-family: Arial, sans-serif;
    }}
    canvas {{
      display: block;
      width: 100vw;
      height: 100vh;
      cursor: grab;
    }}
    canvas:active {{
      cursor: grabbing;
    }}
    .panel {{
      position: fixed;
      left: 12px;
      top: 12px;
      max-width: min(520px, calc(100vw - 24px));
      padding: 10px 12px;
      border: 1px solid rgba(255,255,255,0.18);
      border-radius: 6px;
      background: rgba(17,17,17,0.82);
      backdrop-filter: blur(6px);
      font-size: 13px;
      line-height: 1.45;
      box-sizing: border-box;
    }}
    .title {{
      font-weight: 700;
      margin-bottom: 4px;
      word-break: break-all;
    }}
    .meta {{
      color: #cfcfcf;
    }}
    .actions {{
      position: fixed;
      right: 12px;
      top: 12px;
      display: flex;
      gap: 8px;
    }}
    button {{
      height: 34px;
      padding: 0 12px;
      border: 1px solid rgba(255,255,255,0.22);
      border-radius: 6px;
      background: #2f6f73;
      color: #ffffff;
      font-size: 13px;
      cursor: pointer;
    }}
    button:hover {{
      background: #39858a;
    }}
  </style>
</head>
<body>
  <canvas id="viewer"></canvas>
  <div class="panel">
    <div class="title">{title}</div>
    <div class="meta" id="meta"></div>
  </div>
  <div class="actions">
    <button id="reset">Reset</button>
  </div>
  <script>
    const points = {points_json};
    const metadata = {metadata_json};
    const canvas = document.getElementById("viewer");
    const ctx = canvas.getContext("2d");
    const resetButton = document.getElementById("reset");
    const meta = document.getElementById("meta");

    const bounds = points.reduce((acc, p) => {{
      acc.minX = Math.min(acc.minX, p.x);
      acc.maxX = Math.max(acc.maxX, p.x);
      acc.minY = Math.min(acc.minY, p.y);
      acc.maxY = Math.max(acc.maxY, p.y);
      acc.minZ = Math.min(acc.minZ, p.z);
      acc.maxZ = Math.max(acc.maxZ, p.z);
      return acc;
    }}, {{
      minX: Infinity, maxX: -Infinity,
      minY: Infinity, maxY: -Infinity,
      minZ: Infinity, maxZ: -Infinity
    }});

    const size = Math.max(
      Math.abs(bounds.minX),
      Math.abs(bounds.maxX),
      Math.abs(bounds.minY),
      Math.abs(bounds.maxY),
      Math.abs(bounds.minZ),
      Math.abs(bounds.maxZ),
      1e-9
    ) * 2;

    let state = {{
      yaw: -0.75,
      pitch: 0.55,
      zoom: 1.0,
      panX: 0,
      panY: 0,
      dragging: false,
      lastX: 0,
      lastY: 0,
      moved: false
    }};

    meta.textContent =
      `${{metadata.point_count.toLocaleString()}} points` +
      (metadata.skipped_rows ? `, ${{metadata.skipped_rows.toLocaleString()}} skipped` : "") +
      ` | columns: ${{metadata.columns.x}}, ${{metadata.columns.y}}, ${{metadata.columns.z}}`;

    function resize() {{
      const ratio = window.devicePixelRatio || 1;
      canvas.width = Math.floor(window.innerWidth * ratio);
      canvas.height = Math.floor(window.innerHeight * ratio);
      ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
      draw();
    }}

    function colorFor(point) {{
      if (metadata.has_paint && String(point.paint) === "1") {{
        return "#f2c94c";
      }}
      if (metadata.has_path && point.path !== undefined && point.path !== "") {{
        const hue = (Number(point.path) * 53 + 190) % 360;
        return `hsl(${{hue}}, 72%, 62%)`;
      }}
      return "#62d2a2";
    }}

    function project(point) {{
      const x0 = point.x;
      const y0 = point.y;
      const z0 = point.z;

      const cy = Math.cos(state.yaw);
      const sy = Math.sin(state.yaw);
      const cp = Math.cos(state.pitch);
      const sp = Math.sin(state.pitch);

      const x1 = cy * x0 + sy * z0;
      const z1 = -sy * x0 + cy * z0;
      const y1 = cp * y0 - sp * z1;
      const z2 = sp * y0 + cp * z1;

      const scale = Math.min(window.innerWidth, window.innerHeight) * 0.72 * state.zoom / size;
      return {{
        x: window.innerWidth / 2 + state.panX + x1 * scale,
        y: window.innerHeight / 2 + state.panY - y1 * scale,
        z: z2
      }};
    }}

    function drawAxes() {{
      const axisLength = size * 0.25;
      const axes = [
        [{{x: 0, y: 0, z: 0}}, {{x: axisLength, y: 0, z: 0}}, "#ff6b6b", "X"],
        [{{x: 0, y: 0, z: 0}}, {{x: 0, y: axisLength, z: 0}}, "#8fd16a", "Y"],
        [{{x: 0, y: 0, z: 0}}, {{x: 0, y: 0, z: axisLength}}, "#6ea8fe", "Z"]
      ];
      ctx.lineWidth = 2;
      ctx.font = "12px Arial";
      for (const [a, b, color, label] of axes) {{
        const pa = project(a);
        const pb = project(b);
        ctx.strokeStyle = color;
        ctx.fillStyle = color;
        ctx.beginPath();
        ctx.moveTo(pa.x, pa.y);
        ctx.lineTo(pb.x, pb.y);
        ctx.stroke();
        ctx.fillText(label, pb.x + 5, pb.y - 5);
      }}
    }}

    function draw() {{
      ctx.clearRect(0, 0, window.innerWidth, window.innerHeight);
      ctx.fillStyle = "#111111";
      ctx.fillRect(0, 0, window.innerWidth, window.innerHeight);

      drawAxes();

      const projected = points.map((point, index) => {{
        const p = project(point);
        p.index = index;
        p.source = point;
        return p;
      }}).sort((a, b) => a.z - b.z);

      let lastPath = null;
      let lastPoint = null;
      if (metadata.has_path) {{
        ctx.lineWidth = 1.2;
        const ordered = points.map((point) => ({{...project(point), source: point}}));
        for (const p of ordered) {{
          const path = String(p.source.path ?? "");
          if (lastPoint && path === lastPath) {{
            ctx.strokeStyle = "rgba(255,255,255,0.22)";
            ctx.beginPath();
            ctx.moveTo(lastPoint.x, lastPoint.y);
            ctx.lineTo(p.x, p.y);
            ctx.stroke();
          }}
          lastPath = path;
          lastPoint = p;
        }}
      }}

      for (const p of projected) {{
        ctx.fillStyle = colorFor(p.source);
        ctx.beginPath();
        ctx.arc(p.x, p.y, metadata.has_path ? 2.6 : 1.45, 0, Math.PI * 2);
        ctx.fill();
      }}
    }}

    canvas.addEventListener("mousedown", (event) => {{
      state.dragging = true;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      state.moved = false;
    }});

    window.addEventListener("mousemove", (event) => {{
      if (!state.dragging) {{
        return;
      }}
      const dx = event.clientX - state.lastX;
      const dy = event.clientY - state.lastY;
      state.lastX = event.clientX;
      state.lastY = event.clientY;
      state.moved = true;
      if (event.shiftKey) {{
        state.panX += dx;
        state.panY += dy;
      }} else {{
        state.yaw += dx * 0.008;
        state.pitch = Math.max(-1.45, Math.min(1.45, state.pitch + dy * 0.008));
      }}
      draw();
    }});

    window.addEventListener("mouseup", () => {{
      state.dragging = false;
    }});

    canvas.addEventListener("wheel", (event) => {{
      event.preventDefault();
      const factor = Math.exp(-event.deltaY * 0.001);
      state.zoom = Math.max(0.08, Math.min(40, state.zoom * factor));
      draw();
    }}, {{passive: false}});

    resetButton.addEventListener("click", () => {{
      state.yaw = -0.75;
      state.pitch = 0.55;
      state.zoom = 1.0;
      state.panX = 0;
      state.panY = 0;
      draw();
    }});

    window.addEventListener("resize", resize);
    resize();
  </script>
</body>
</html>
"""


def write_html_viewer(points: list[dict], metadata: dict, output_path: Path | None) -> Path:
    html_text = build_html(points, metadata)
    if output_path is None:
        safe_name = Path(metadata["source"]).stem.replace(" ", "_")
        temp_dir = Path(tempfile.gettempdir())
        output_path = temp_dir / f"{safe_name}_csv_viewer.html"

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(html_text, encoding="utf-8")
    return output_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="CSV 파일의 3D 포인트를 브라우저에서 시각화합니다."
    )
    parser.add_argument(
        "csv_path",
        type=Path,
        nargs="?",
        default=None,
        help=f"시각화할 CSV 파일 경로. 생략하면 DEFAULT_CSV_PATH 사용: {DEFAULT_CSV_PATH}",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="생성할 HTML 파일 경로. 기본값은 /tmp/<csv_name>_csv_viewer.html",
    )
    parser.add_argument(
        "--no-browser",
        action="store_true",
        help="HTML만 생성하고 브라우저를 열지 않습니다.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    selected_csv_path = args.csv_path if args.csv_path is not None else DEFAULT_CSV_PATH
    csv_path = selected_csv_path.expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV 파일이 없습니다: {csv_path}")

    points, metadata = load_points(csv_path)
    html_path = write_html_viewer(points, metadata, args.output)
    print(f"[csv_checker] points={metadata['point_count']} skipped={metadata['skipped_rows']}")
    print(f"[csv_checker] html={html_path}")

    if not args.no_browser:
        webbrowser.open(html_path.as_uri())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
