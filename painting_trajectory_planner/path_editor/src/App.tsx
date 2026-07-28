import { useEffect, useMemo, useRef, useState } from "react";
import type { ReactNode } from "react";
import { TrajectoryViewer } from "./components/TrajectoryViewer";
import { getDataRoot, listProjects, readParameters, readProjectFile } from "./lib/api";
import { parseParameters, parseTrajectoryCsv } from "./lib/loaders";
import type {
  LayerVisibility,
  ProjectData,
  ProjectInfo,
  TrajectoryRow,
  VisualizationSettings,
} from "./types";

const initialLayers: LayerVisibility = {
  surface: true,
  trajectory: true,
  sprayOff: true,
  axes: true,
};

const initialVisualization: VisualizationSettings = {
  surfaceStyle: "original",
  background: "gray",
  rimLight: true,
  lightIntensity: 1,
};

const parameterUnits: Record<string, string> = {
  spline_start_offset: "m",
  spline_spacing: "m",
  spline_half_width: "m",
  geodesic_band_half_width: "m",
  curvature_slice_angle_deg: "deg",
  quantize_step: "m",
  component_min_arc_length: "m",
  component_merge_backtrack_tolerance: "m",
  support_point_spacing: "m",
  tangent_half_width: "m",
  anchor_max_distance: "m",
  offset_point_spacing: "m",
  offset_distance: "m",
  spline_point_spacing: "m",
  endpoint_extension_length: "m",
  paint_check_distance: "m",
  paint_check_half_width: "m",
  paint_min_false_length: "m",
  paint_min_true_length: "m",
  desired_speed: "m/s",
  control_dt: "s",
  max_acceleration: "m/s²",
};

interface FlatParameter {
  group: string;
  key: string;
  value: string;
  unit: string;
}

function Foldout({
  title,
  children,
  className = "",
}: {
  title: string;
  children: ReactNode;
  className?: string;
}) {
  const [open, setOpen] = useState(true);
  return (
    <section className={`foldout ${open ? "open" : ""} ${className}`.trim()}>
      <button
        className="foldout-heading"
        type="button"
        aria-expanded={open}
        onClick={() => setOpen((current) => !current)}
      >
        <i />
        <span>{title}</span>
      </button>
      {open && <div className="foldout-content">{children}</div>}
    </section>
  );
}

function formatNumber(value: number, digits = 3): string {
  return new Intl.NumberFormat("ko-KR", {
    maximumFractionDigits: digits,
  }).format(value);
}

function flattenParameters(
  value: Record<string, unknown>,
  path: string[] = [],
): FlatParameter[] {
  const result: FlatParameter[] = [];
  for (const [key, child] of Object.entries(value)) {
    if (child && typeof child === "object" && !Array.isArray(child)) {
      result.push(
        ...flattenParameters(child as Record<string, unknown>, [...path, key]),
      );
      continue;
    }
    result.push({
      group: path.join(" / ") || "general",
      key,
      value: typeof child === "string" ? child : JSON.stringify(child),
      unit: parameterUnits[key] ?? "",
    });
  }
  return result;
}

function Toggle({
  checked,
  label,
  color,
  onChange,
}: {
  checked: boolean;
  label: string;
  color?: string;
  onChange: (checked: boolean) => void;
}) {
  return (
    <label className="layer-toggle">
      <input
        type="checkbox"
        checked={checked}
        onChange={(event) => onChange(event.target.checked)}
      />
      <span className="toggle-track"><span /></span>
      {color && <i style={{ backgroundColor: color }} />}
      <span>{label}</span>
    </label>
  );
}

function RowList({
  rows,
  selectedRow,
  onSelect,
}: {
  rows: TrajectoryRow[];
  selectedRow: number | null;
  onSelect: (row: number | null) => void;
}) {
  return (
    <div className="row-list">
      {rows.map((row) => (
        <button
          className={selectedRow === row.rowIndex ? "row-item selected" : "row-item"}
          key={row.id}
          type="button"
          onClick={() => onSelect(selectedRow === row.rowIndex ? null : row.rowIndex)}
        >
          <span className="row-index">{String(row.rowIndex).padStart(2, "0")}</span>
          <span>
            <strong>{formatNumber(row.length)} m</strong>
            <small>{formatNumber(row.paintRatio * 100, 1)}% spray</small>
          </span>
          <span className="row-points">{formatNumber(row.points.length, 0)} pt</span>
        </button>
      ))}
    </div>
  );
}

function ParameterGroup({
  group,
  parameters,
}: {
  group: string;
  parameters: FlatParameter[];
}) {
  const [open, setOpen] = useState(true);
  return (
    <section className={open ? "parameter-section open" : "parameter-section"}>
      <button
        type="button"
        className="parameter-section-heading"
        aria-expanded={open}
        onClick={() => setOpen((current) => !current)}
      >
        <i />
        <span>{group}</span>
        <small>{parameters.length}</small>
      </button>
      {open && parameters.map((parameter) => (
        <div className="parameter-item" key={`${parameter.group}.${parameter.key}`}>
          <strong title={parameter.key}>{parameter.key}</strong>
          <span title={parameter.value}>
            {parameter.value}
            {parameter.unit && <em>{parameter.unit}</em>}
          </span>
        </div>
      ))}
    </section>
  );
}

export default function App() {
  const [projects, setProjects] = useState<ProjectInfo[]>([]);
  const [selectedProject, setSelectedProject] = useState("");
  const [projectData, setProjectData] = useState<ProjectData | null>(null);
  const [layers, setLayers] = useState(initialLayers);
  const [visualization, setVisualization] = useState(initialVisualization);
  const [selectedRow, setSelectedRow] = useState<number | null>(null);
  const [activeTab, setActiveTab] = useState<"rows" | "parameters">("rows");
  const [parameterQuery, setParameterQuery] = useState("");
  const [dataRoot, setDataRoot] = useState("");
  const [status, setStatus] = useState("프로젝트를 찾는 중입니다.");
  const [error, setError] = useState("");
  const [projectMenuOpen, setProjectMenuOpen] = useState(false);
  const projectPickerRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    let cancelled = false;
    Promise.all([listProjects(), getDataRoot()])
      .then(([items, root]) => {
        if (cancelled) {
          return;
        }
        setProjects(items);
        setDataRoot(root);
        if (items.length > 0) {
          setSelectedProject(items[0].name);
          setStatus(`${items.length}개 프로젝트를 찾았습니다.`);
        } else {
          setStatus("표시할 프로젝트가 없습니다.");
        }
      })
      .catch((reason: unknown) => {
        if (!cancelled) {
          setError(String(reason));
          setStatus("데이터 디렉터리를 확인하지 못했습니다.");
        }
      });
    return () => {
      cancelled = true;
    };
  }, []);

  useEffect(() => {
    if (!selectedProject) {
      return;
    }
    const info = projects.find((project) => project.name === selectedProject);
    if (!info) {
      return;
    }
    let cancelled = false;
    setError("");
    setProjectData(null);
    setSelectedRow(null);
    setStatus(`${selectedProject} 데이터를 불러오는 중입니다.`);

    Promise.all([
      readProjectFile(selectedProject, "surface"),
      readProjectFile(selectedProject, "trajectory"),
      readParameters(selectedProject),
    ])
      .then(([surfaceBuffer, trajectoryBuffer, parameterSource]) => {
        if (cancelled) {
          return;
        }
        const trajectory = parseTrajectoryCsv(trajectoryBuffer);
        const parameters = parseParameters(parameterSource);
        setProjectData({ info, surfaceBuffer, trajectory, parameters });
        setStatus(
          `${trajectory.rows.length} rows · ${formatNumber(trajectory.pointCount, 0)} trajectory points`,
        );
      })
      .catch((reason: unknown) => {
        if (!cancelled) {
          setError(String(reason));
          setStatus(`${selectedProject} 로드에 실패했습니다.`);
        }
      });
    return () => {
      cancelled = true;
    };
  }, [projects, selectedProject]);

  useEffect(() => {
    const closeProjectMenu = (event: PointerEvent) => {
      if (!projectPickerRef.current?.contains(event.target as Node)) {
        setProjectMenuOpen(false);
      }
    };
    const closeProjectMenuWithEscape = (event: KeyboardEvent) => {
      if (event.key === "Escape") {
        setProjectMenuOpen(false);
      }
    };
    document.addEventListener("pointerdown", closeProjectMenu);
    document.addEventListener("keydown", closeProjectMenuWithEscape);
    return () => {
      document.removeEventListener("pointerdown", closeProjectMenu);
      document.removeEventListener("keydown", closeProjectMenuWithEscape);
    };
  }, []);

  const flatParameters = useMemo(() => {
    if (!projectData) {
      return [];
    }
    const query = parameterQuery.trim().toLowerCase();
    return flattenParameters(projectData.parameters).filter((parameter) =>
      !query
      || parameter.key.toLowerCase().includes(query)
      || parameter.group.toLowerCase().includes(query)
      || parameter.value.toLowerCase().includes(query),
    );
  }, [parameterQuery, projectData]);

  const parameterGroups = useMemo(() => {
    const groups = new Map<string, FlatParameter[]>();
    for (const parameter of flatParameters) {
      const group = groups.get(parameter.group);
      if (group) {
        group.push(parameter);
      } else {
        groups.set(parameter.group, [parameter]);
      }
    }
    return [...groups.entries()];
  }, [flatParameters]);

  const setLayer = (key: keyof LayerVisibility, value: boolean) => {
    setLayers((current) => ({ ...current, [key]: value }));
  };

  const setVisualizationValue = <Key extends keyof VisualizationSettings>(
    key: Key,
    value: VisualizationSettings[Key],
  ) => {
    setVisualization((current) => ({ ...current, [key]: value }));
  };

  return (
    <main className="app-shell">
      <section className="workspace">
        <aside className="left-panel panel">
          <div className="panel-titlebar">
            <span className="panel-title-icon">▦</span>
            <strong>Hierarchy</strong>
            <span className="panel-badge">READ ONLY</span>
          </div>
          <div className="left-panel-content">
            <div className="workspace-object">
              <span className="object-icon">P</span>
              <span>
                <strong>Painting Trajectory</strong>
                <small>Review workspace</small>
              </span>
            </div>

            <Foldout title="Project Data" className="project-section">
              <div
                className={projectMenuOpen ? "project-picker open" : "project-picker"}
                ref={projectPickerRef}
              >
                <button
                  className="project-select-button"
                  type="button"
                  aria-haspopup="listbox"
                  aria-expanded={projectMenuOpen}
                  onClick={() => setProjectMenuOpen((open) => !open)}
                >
                  <span>{selectedProject || "Select project"}</span>
                  <i />
                </button>
                {projectMenuOpen && (
                  <div className="project-options" role="listbox" aria-label="Project data">
                    {projects.map((project) => (
                      <button
                        className={project.name === selectedProject ? "selected" : ""}
                        key={project.name}
                        type="button"
                        role="option"
                        aria-selected={project.name === selectedProject}
                        onClick={() => {
                          setSelectedProject(project.name);
                          setProjectMenuOpen(false);
                        }}
                      >
                        <span>{project.name}</span>
                        {project.name === selectedProject && <i>✓</i>}
                      </button>
                    ))}
                  </div>
                )}
              </div>
            </Foldout>

            <Foldout title="Scene Layers">
              <div className="layer-list">
                <Toggle
                  checked={layers.surface}
                  label="Part surface"
                  onChange={(value) => setLayer("surface", value)}
                />
                <Toggle
                  checked={layers.trajectory}
                  label="Spray ON path"
                  color="#20e66a"
                  onChange={(value) => setLayer("trajectory", value)}
                />
                <Toggle
                  checked={layers.sprayOff}
                  label="Spray OFF path"
                  color="#ffc857"
                  onChange={(value) => setLayer("sprayOff", value)}
                />
                <Toggle
                  checked={layers.axes}
                  label="Coordinate axes"
                  onChange={(value) => setLayer("axes", value)}
                />
              </div>
            </Foldout>

            <Foldout title="Visualization">
              <div className="inspector-property">
                <span>Surface</span>
                <div className="segmented-control">
                  <button
                    className={visualization.surfaceStyle === "original" ? "active" : ""}
                    type="button"
                    onClick={() => setVisualizationValue("surfaceStyle", "original")}
                  >
                    Original
                  </button>
                  <button
                    className={visualization.surfaceStyle === "inspection" ? "active" : ""}
                    type="button"
                    onClick={() => setVisualizationValue("surfaceStyle", "inspection")}
                  >
                    Inspection
                  </button>
                </div>
              </div>
              <div className="inspector-property">
                <span>Background</span>
                <div className="segmented-control">
                  <button
                    className={visualization.background === "gray" ? "active" : ""}
                    type="button"
                    onClick={() => setVisualizationValue("background", "gray")}
                  >
                    Gray
                  </button>
                  <button
                    className={visualization.background === "black" ? "active" : ""}
                    type="button"
                    onClick={() => setVisualizationValue("background", "black")}
                  >
                    Black
                  </button>
                </div>
              </div>
              <label className="unity-toggle">
                <span>Rim Light</span>
                <input
                  type="checkbox"
                  checked={visualization.rimLight}
                  onChange={(event) => setVisualizationValue("rimLight", event.target.checked)}
                />
                <i />
              </label>
              <label className="light-control">
                <span>
                  <strong>Scene Light</strong>
                  <output>{visualization.lightIntensity.toFixed(1)}×</output>
                </span>
                <input
                  type="range"
                  min="0.5"
                  max="1.8"
                  step="0.1"
                  value={visualization.lightIntensity}
                  onChange={(event) =>
                    setVisualizationValue("lightIntensity", Number(event.target.value))}
                />
              </label>
            </Foldout>

            <Foldout title="Data Source" className="data-source-foldout">
              <div className="source-row">
                <span>Mesh</span>
                <strong title={projectData?.info.surfaceFile}>
                  {projectData?.info.surfaceFile ?? "—"}
                </strong>
              </div>
              <div className="source-row">
                <span>Path</span>
                <strong title={projectData?.info.trajectoryFile}>
                  {projectData?.info.trajectoryFile ?? "—"}
                </strong>
              </div>
              <p className="data-root" title={dataRoot}>{dataRoot || "데이터 경로 확인 중"}</p>
            </Foldout>
          </div>
        </aside>

        <section className="viewer-panel">
          <div className="scene-titlebar">
            <span className="scene-tab active">Scene</span>
            <span className="scene-project">{selectedProject || "No project"}</span>
          </div>
          <div className="scene-body">
            <TrajectoryViewer
              surfaceBuffer={projectData?.surfaceBuffer ?? null}
              trajectory={projectData?.trajectory ?? null}
              layers={layers}
              visualization={visualization}
              selectedRowId={selectedRow}
            />
            {!projectData && !error && (
              <div className="loading-card">
                <span className="loader" />
                <strong>Loading project</strong>
                <p>{status}</p>
              </div>
            )}
            {error && (
              <div className="error-card">
                <strong>프로젝트를 열 수 없습니다.</strong>
                <p>{error}</p>
              </div>
            )}
          </div>
        </section>

        <aside className="right-panel panel">
          <div className="panel-titlebar inspector-titlebar">
            <span className="panel-title-icon">☷</span>
            <strong>Inspector</strong>
            <span className="panel-menu">⋮</span>
          </div>
          <div className="tabs">
            <button
              type="button"
              className={activeTab === "rows" ? "active" : ""}
              onClick={() => setActiveTab("rows")}
            >
              PATH ROWS
            </button>
            <button
              type="button"
              className={activeTab === "parameters" ? "active" : ""}
              onClick={() => setActiveTab("parameters")}
            >
              PARAMETERS
            </button>
          </div>

          {activeTab === "rows" ? (
            <div className="tab-content">
              <div className="tab-heading">
                <div>
                  <p className="eyebrow">TRAJECTORY</p>
                  <h2>{selectedRow === null ? "All path rows" : `Row ${selectedRow}`}</h2>
                </div>
                {selectedRow !== null && (
                  <button type="button" className="text-button" onClick={() => setSelectedRow(null)}>
                    CLEAR
                  </button>
                )}
              </div>
              <RowList
                rows={projectData?.trajectory.rows ?? []}
                selectedRow={selectedRow}
                onSelect={setSelectedRow}
              />
            </div>
          ) : (
            <div className="tab-content parameters-content">
              <div className="tab-heading">
                <div>
                  <p className="eyebrow">PROJECT CONFIG</p>
                  <h2>Applied parameters</h2>
                </div>
              </div>
              <input
                className="parameter-search"
                type="search"
                value={parameterQuery}
                placeholder="Search parameters"
                onChange={(event) => setParameterQuery(event.target.value)}
              />
              <div className="parameter-list">
                {parameterGroups.map(([group, parameters]) => (
                  <ParameterGroup group={group} parameters={parameters} key={group} />
                ))}
              </div>
            </div>
          )}
        </aside>
      </section>

      <footer className="statusbar">
        <span className={error ? "status-dot error" : "status-dot"} />
        <span>{status}</span>
        <span className="status-version">Reviewer v0.1.0 · Schema v1</span>
      </footer>
    </main>
  );
}
