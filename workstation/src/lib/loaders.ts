import { parse as parseYaml } from "yaml";
import type {
  TrajectoryDataset,
  TrajectoryPoint,
  TrajectoryRow,
} from "../types";

const textDecoder = new TextDecoder("utf-8");

function numberAt(values: string[], index: number): number {
  const value = Number(values[index]);
  return Number.isFinite(value) ? value : 0;
}

export function parseParameterDocument(source: string): Record<string, unknown> {
  const parsed = parseYaml(source);
  if (!parsed || typeof parsed !== "object" || Array.isArray(parsed)) {
    throw new Error("파라미터 YAML의 최상위 값이 객체가 아닙니다.");
  }
  const root = parsed as Record<string, unknown>;
  const parameters = root.painting_trajectory;
  if (!parameters || typeof parameters !== "object" || Array.isArray(parameters)) {
    throw new Error("YAML에 painting_trajectory 항목이 없습니다.");
  }
  return root;
}

export function parseTrajectoryCsv(buffer: ArrayBuffer): TrajectoryDataset {
  const text = textDecoder.decode(buffer);
  const lines = text.split(/\r?\n/);
  if (lines.length < 2) {
    throw new Error("trajectory CSV에 데이터가 없습니다.");
  }

  const header = lines[0].split(",");
  const column = new Map(header.map((name, index) => [name.trim(), index]));
  const required = [
    "row_index",
    "point_index",
    "s",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "paint",
  ];
  for (const name of required) {
    if (!column.has(name)) {
      throw new Error(`trajectory CSV 필수 열이 없습니다: ${name}`);
    }
  }

  const rowsByIndex = new Map<number, TrajectoryPoint[]>();
  let paintPointCount = 0;
  for (let lineIndex = 1; lineIndex < lines.length; lineIndex += 1) {
    const line = lines[lineIndex].trim();
    if (!line) {
      continue;
    }
    const values = line.split(",");
    const rowIndex = numberAt(values, column.get("row_index")!);
    const pointIndex = numberAt(values, column.get("point_index")!);
    const paintValue = values[column.get("paint")!]?.trim().toLowerCase();
    const paint = paintValue === "1" || paintValue === "true";
    if (paint) {
      paintPointCount += 1;
    }

    const point: TrajectoryPoint = {
      id: `row-${rowIndex}-point-${pointIndex}`,
      rowId: rowIndex,
      pointId: pointIndex,
      position: [
        numberAt(values, column.get("position_x")!),
        numberAt(values, column.get("position_y")!),
        numberAt(values, column.get("position_z")!),
      ],
      orientation: [
        numberAt(values, column.get("orientation_x")!),
        numberAt(values, column.get("orientation_y")!),
        numberAt(values, column.get("orientation_z")!),
        numberAt(values, column.get("orientation_w")!),
      ],
      distance: numberAt(values, column.get("s")!),
      paint,
    };
    const row = rowsByIndex.get(rowIndex);
    if (row) {
      row.push(point);
    } else {
      rowsByIndex.set(rowIndex, [point]);
    }
  }

  const rows: TrajectoryRow[] = [...rowsByIndex.entries()]
    .sort(([left], [right]) => left - right)
    .map(([rowIndex, points]) => {
      points.sort((left, right) => left.pointId - right.pointId);
      const first = points[0];
      const last = points[points.length - 1];
      const rowPaintPoints = points.reduce(
        (count, point) => count + Number(point.paint),
        0,
      );
      return {
        id: `row-${rowIndex}`,
        rowIndex,
        points,
        length: Math.max(0, last.distance - first.distance),
        paintRatio: points.length ? rowPaintPoints / points.length : 0,
      };
    });

  const pointCount = rows.reduce((count, row) => count + row.points.length, 0);
  return {
    rows,
    pointCount,
    paintPointCount,
  };
}
