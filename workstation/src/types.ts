export type Vector3Tuple = [number, number, number];
export type QuaternionTuple = [number, number, number, number];

export interface TrajectoryPoint {
  id: string;
  rowId: number;
  pointId: number;
  position: Vector3Tuple;
  orientation: QuaternionTuple;
  distance: number;
  paint: boolean;
}

export interface TrajectoryRow {
  id: string;
  rowIndex: number;
  points: TrajectoryPoint[];
  length: number;
  paintRatio: number;
}

export interface TrajectoryDataset {
  rows: TrajectoryRow[];
  pointCount: number;
  paintPointCount: number;
}

export interface EditableControlPoint {
  id: string;
  rowIndex: number;
  pointIndex: number;
  position: Vector3Tuple;
  orientation: QuaternionTuple;
  paint: boolean;
  planeOrigin: Vector3Tuple;
  planeNormal: Vector3Tuple;
}

export interface ControlPointDataset {
  points: EditableControlPoint[];
}

export interface LayerVisibility {
  surface: boolean;
  trajectory: boolean;
  sprayOff: boolean;
  sprayDirections: boolean;
  controlPoints: boolean;
  axes: boolean;
}

export interface VisualizationSettings {
  surfaceStyle: "original" | "inspection";
  background: "gray" | "black";
  rimLight: boolean;
  lightIntensity: number;
}

export interface BackendInfo {
  plannerRoot: string;
  logRoot: string;
  backendScript: string;
  pythonCommand: string;
}

export interface ScanInfo {
  fileName: string;
  byteCount: number;
  sessionDirectory: string;
}

export interface GenerationSummary {
  rowCount: number;
  pointCount: number;
  controlPointCount: number;
}

export interface CompletionSummary {
  completedAt: string;
  scanFile: string;
  trajectoryFile: string;
  outputDirectory: string;
}
