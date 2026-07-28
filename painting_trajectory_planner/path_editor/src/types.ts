export interface ProjectInfo {
  name: string;
  surfaceFile: string;
  trajectoryFile: string;
  surfaceBytes: number;
  trajectoryBytes: number;
}

export interface TrajectoryPoint {
  id: string;
  rowId: number;
  pointId: number;
  position: [number, number, number];
  orientation: [number, number, number, number];
  time: number;
  distance: number;
  speed: number;
  acceleration: number;
  paint: boolean;
}

export interface TrajectoryRow {
  id: string;
  rowIndex: number;
  points: TrajectoryPoint[];
  length: number;
  duration: number;
  paintRatio: number;
}

export interface TrajectoryDataset {
  rows: TrajectoryRow[];
  pointCount: number;
  paintPointCount: number;
}

export interface LayerVisibility {
  surface: boolean;
  trajectory: boolean;
  sprayOff: boolean;
  axes: boolean;
}

export interface VisualizationSettings {
  surfaceStyle: "original" | "inspection";
  background: "gray" | "black";
  rimLight: boolean;
  lightIntensity: number;
}

export interface ProjectData {
  info: ProjectInfo;
  surfaceBuffer: ArrayBuffer;
  trajectory: TrajectoryDataset;
  parameters: Record<string, unknown>;
}
