import { useEffect, useMemo, useRef, useState } from "react";
import type { ReactNode } from "react";
import { TrajectoryViewer } from "./components/TrajectoryViewer";
import { getDataRoot, listProjects, readParameters, readProjectFile } from "./lib/api";
import { parseParameters, parseTrajectoryCsv } from "./lib/loaders";
import type {
  LayerVisibility,
  ProjectData,
  ProjectInfo,
  VisualizationSettings,
} from "./types";

const initialLayers: LayerVisibility = {
  surface: true,
  trajectory: true,
  sprayOff: true,
  sprayDirections: true,
  axes: true,
};

const initialVisualization: VisualizationSettings = {
  surfaceStyle: "original",
  background: "gray",
  rimLight: true,
  lightIntensity: 1,
};

interface ParameterDefinition {
  path: readonly [string, string];
  label: string;
  unit?: "mm" | "°";
  scale?: number;
}

interface DisplayParameter {
  id: string;
  label: string;
  value: string;
  unit: string;
}

const displayedParameterDefinitions: readonly ParameterDefinition[] = [
  { path: ["project", "name"], label: "프로젝트 이름" },
  { path: ["project", "paint_dir"], label: "도장 방향 벡터" },
  { path: ["project", "scan_dir"], label: "스캔 방향 벡터" },
  { path: ["surface_extraction", "slicing_method"], label: "슬라이싱 방식" },
  { path: ["surface_extraction", "spline_start_side"], label: "슬라이싱 시작 방향" },
  {
    path: ["surface_extraction", "spline_start_offset"],
    label: "슬라이싱 시작 오프셋",
    unit: "mm",
    scale: 1000,
  },
  {
    path: ["surface_extraction", "spline_spacing"],
    label: "슬라이싱 간격",
    unit: "mm",
    scale: 1000,
  },
  {
    path: ["surface_extraction", "curvature_slice_angle_deg"],
    label: "곡률 보정 기준 각도",
    unit: "°",
  },
  {
    path: ["offset", "offset_distance"],
    label: "표면 오프셋 거리",
    unit: "mm",
    scale: 1000,
  },
  {
    path: ["spline", "final_point_spacing"],
    label: "최종 경로 포인트 간격",
    unit: "mm",
    scale: 1000,
  },
  {
    path: ["spline", "endpoint_extension_length"],
    label: "경로 끝점 연장 길이",
    unit: "mm",
    scale: 1000,
  },
  { path: ["spline", "raster_zigzag"], label: "지그재그 경로 사용" },
  {
    path: ["spline", "paint_min_false_length"],
    label: "최소 미분사 구간 길이",
    unit: "mm",
    scale: 1000,
  },
  {
    path: ["spline", "paint_min_true_length"],
    label: "최소 분사 구간 길이",
    unit: "mm",
    scale: 1000,
  },
];

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

function formatParameterValue(
  parameters: Record<string, unknown>,
  definition: ParameterDefinition,
): string {
  const [sectionName, parameterName] = definition.path;
  const section = parameters[sectionName];
  if (!section || typeof section !== "object" || Array.isArray(section)) {
    return "—";
  }
  const value = (section as Record<string, unknown>)[parameterName];
  if (value === undefined || value === null) {
    return "—";
  }
  if (typeof value === "number") {
    return formatNumber(value * (definition.scale ?? 1), 6);
  }
  if (typeof value === "boolean") {
    return value ? "사용" : "사용 안 함";
  }
  if (Array.isArray(value)) {
    return `[${value.map((item) => String(item)).join(", ")}]`;
  }
  return typeof value === "string" ? value : JSON.stringify(value);
}

function selectDisplayedParameters(
  parameters: Record<string, unknown>,
): DisplayParameter[] {
  return displayedParameterDefinitions.map((definition) => ({
    id: definition.path.join("."),
    label: definition.label,
    value: formatParameterValue(parameters, definition),
    unit: definition.unit ?? "",
  }));
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

export default function App() {
  const [projects, setProjects] = useState<ProjectInfo[]>([]);
  const [selectedProject, setSelectedProject] = useState("");
  const [projectData, setProjectData] = useState<ProjectData | null>(null);
  const [layers, setLayers] = useState(initialLayers);
  const [visualization, setVisualization] = useState(initialVisualization);
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

  const visibleParameters = useMemo(() => {
    if (!projectData) {
      return [];
    }
    const query = parameterQuery.trim().toLowerCase();
    return selectDisplayedParameters(projectData.parameters).filter((parameter) =>
      !query
      || parameter.label.toLowerCase().includes(query)
      || parameter.value.toLowerCase().includes(query),
    );
  }, [parameterQuery, projectData]);

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
                  checked={layers.sprayDirections}
                  label="Spray directions"
                  color="#4cc9f0"
                  onChange={(value) => setLayer("sprayDirections", value)}
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
            <button type="button" className="active">
              PARAMETERS
            </button>
          </div>

          <div className="tab-content parameters-content">
            <div className="tab-heading">
              <div>
                <p className="eyebrow">프로젝트 설정</p>
                <h2>적용 파라미터</h2>
              </div>
            </div>
            <input
              className="parameter-search"
              type="search"
              value={parameterQuery}
              placeholder="파라미터 검색"
              onChange={(event) => setParameterQuery(event.target.value)}
            />
            <div className="parameter-list">
              {visibleParameters.map((parameter) => (
                <div className="parameter-item" key={parameter.id}>
                  <strong title={parameter.label}>{parameter.label}</strong>
                  <span title={`${parameter.value}${parameter.unit ? ` ${parameter.unit}` : ""}`}>
                    {parameter.value}
                    {parameter.unit && <em>{parameter.unit}</em>}
                  </span>
                </div>
              ))}
            </div>
          </div>
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
