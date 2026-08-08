import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import type {
  EditableControlPoint,
  LayerVisibility,
  TrajectoryDataset,
  VisualizationSettings,
} from "../types";

interface TrajectoryViewerProps {
  surfaceBuffer: ArrayBuffer | null;
  surfaceFileName: string;
  trajectory: TrajectoryDataset | null;
  controlPoints: EditableControlPoint[];
  selectedControlPointId: string | null;
  editingEnabled: boolean;
  layers: LayerVisibility;
  visualization: VisualizationSettings;
  onSelectControlPoint: (id: string | null) => void;
  onMoveControlPoint: (id: string, position: [number, number, number]) => void;
}

interface DragState {
  point: EditableControlPoint;
  controlPointIndex: number;
  plane: THREE.Plane;
  offset: THREE.Vector3;
  moved: boolean;
}

interface ViewerRuntime {
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  scene: THREE.Scene;
  surfaceGroup: THREE.Group;
  pathGroup: THREE.Group;
  sprayOffGroup: THREE.Group;
  sprayDirectionGroup: THREE.Group;
  controlPointGroup: THREE.Group;
  editPlaneGroup: THREE.Group;
  controlPointData: EditableControlPoint[];
  dragState: DragState | null;
  axes: THREE.AxesHelper;
  hemisphereLight: THREE.HemisphereLight;
  keyLight: THREE.DirectionalLight;
  fillLight: THREE.DirectionalLight;
  rimLight: THREE.DirectionalLight;
  rimTarget: THREE.Object3D;
  resizeObserver: ResizeObserver;
  animationFrame: number;
}

function disposeObject(object: THREE.Object3D): void {
  object.traverse((child) => {
    if (
      child instanceof THREE.Mesh
      || child instanceof THREE.Points
      || child instanceof THREE.Line
      || child instanceof THREE.LineSegments
    ) {
      child.geometry?.dispose();
      const materials = Array.isArray(child.material) ? child.material : [child.material];
      for (const material of materials) {
        material?.dispose();
      }
    }
  });
}

function clearGroup(group: THREE.Group): void {
  for (const child of [...group.children]) {
    group.remove(child);
    disposeObject(child);
  }
}

function fitCamera(runtime: ViewerRuntime): void {
  const bounds = new THREE.Box3();
  for (const group of [runtime.surfaceGroup, runtime.pathGroup, runtime.controlPointGroup]) {
    bounds.expandByObject(group);
  }
  if (bounds.isEmpty()) {
    return;
  }
  const center = bounds.getCenter(new THREE.Vector3());
  const size = bounds.getSize(new THREE.Vector3());
  const radius = Math.max(size.length() * 0.5, 0.1);
  const distance = radius / Math.tan(THREE.MathUtils.degToRad(runtime.camera.fov * 0.5));
  runtime.camera.position.copy(
    center.clone().add(new THREE.Vector3(0.75, -1.25, 0.85).normalize().multiplyScalar(distance * 1.15)),
  );
  runtime.camera.near = Math.max(distance / 1000, 0.0001);
  runtime.camera.far = Math.max(distance * 100, 100);
  runtime.camera.updateProjectionMatrix();
  runtime.controls.target.copy(center);
  runtime.controls.update();
}

function buildTrajectorySegments(
  trajectory: TrajectoryDataset,
  paintState: boolean,
  includeRow: (rowIndex: number) => boolean = () => true,
): THREE.BufferGeometry {
  const positions: number[] = [];
  for (const row of trajectory.rows) {
    if (!includeRow(row.rowIndex)) {
      continue;
    }
    for (let index = 1; index < row.points.length; index += 1) {
      const previous = row.points[index - 1];
      const current = row.points[index];
      if ((previous.paint && current.paint) !== paintState) {
        continue;
      }
      positions.push(...previous.position, ...current.position);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  return geometry;
}

interface DirectionSample {
  position: [number, number, number];
  orientation: [number, number, number, number];
  paint: boolean;
}

function buildDirectionGlyphs(
  samples: readonly DirectionSample[],
  opacity = 0.82,
): THREE.Group {
  const directionLength = 0.08;
  const linePositions: number[] = [];
  const lineColors: number[] = [];
  const tipPositions: number[] = [];
  const tipColors: number[] = [];
  const sprayDirection = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  const paintColor = new THREE.Color(0x4cc9f0);
  const offColor = new THREE.Color(0x55798a);

  for (const point of samples) {
    quaternion.fromArray(point.orientation).normalize();
    sprayDirection.set(0, 0, 1).applyQuaternion(quaternion).normalize();
    const origin = new THREE.Vector3(...point.position);
    const tip = origin.clone().addScaledVector(sprayDirection, directionLength);
    const color = point.paint ? paintColor : offColor;
    linePositions.push(...origin.toArray(), ...tip.toArray());
    lineColors.push(color.r, color.g, color.b, color.r, color.g, color.b);
    tipPositions.push(...tip.toArray());
    tipColors.push(color.r, color.g, color.b);
  }

  const group = new THREE.Group();
  const lineGeometry = new THREE.BufferGeometry();
  lineGeometry.setAttribute("position", new THREE.Float32BufferAttribute(linePositions, 3));
  lineGeometry.setAttribute("color", new THREE.Float32BufferAttribute(lineColors, 3));
  const directionLines = new THREE.LineSegments(
    lineGeometry,
    new THREE.LineBasicMaterial({ vertexColors: true, transparent: true, opacity }),
  );
  directionLines.renderOrder = 5;
  group.add(directionLines);
  const tipGeometry = new THREE.BufferGeometry();
  tipGeometry.setAttribute("position", new THREE.Float32BufferAttribute(tipPositions, 3));
  tipGeometry.setAttribute("color", new THREE.Float32BufferAttribute(tipColors, 3));
  const directionTips = new THREE.Points(
    tipGeometry,
    new THREE.PointsMaterial({
      vertexColors: true,
      size: 0.006,
      sizeAttenuation: true,
      transparent: true,
      opacity,
    }),
  );
  directionTips.renderOrder = 6;
  group.add(directionTips);
  return group;
}

function buildSprayDirections(trajectory: TrajectoryDataset): THREE.Group {
  return buildDirectionGlyphs(trajectory.rows.flatMap((row) => row.points));
}

function buildEditableDirections(
  points: EditableControlPoint[],
  selectedRowIndex: number | null,
): THREE.Group {
  if (selectedRowIndex === null) {
    return buildDirectionGlyphs(points);
  }
  const group = new THREE.Group();
  group.add(buildDirectionGlyphs(
    points.filter((point) => point.rowIndex !== selectedRowIndex),
    0.12,
  ));
  group.add(buildDirectionGlyphs(
    points.filter((point) => point.rowIndex === selectedRowIndex),
    0.9,
  ));
  return group;
}

function parseCsvSurface(buffer: ArrayBuffer): THREE.BufferGeometry {
  const source = new TextDecoder("utf-8").decode(buffer);
  const lines = source.split(/\r?\n/).filter((line) => line.trim());
  const positions: number[] = [];
  for (let index = 0; index < lines.length; index += 1) {
    const values = lines[index].split(",").slice(0, 3).map(Number);
    if (values.length !== 3 || !values.every(Number.isFinite)) {
      if (index === 0) {
        continue;
      }
      throw new Error(`스캔 CSV ${index + 1}행의 XYZ 좌표가 올바르지 않습니다.`);
    }
    positions.push(values[0], values[1], values[2]);
  }
  if (!positions.length) {
    throw new Error("스캔 CSV에 표시할 XYZ 포인트가 없습니다.");
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geometry.computeBoundingBox();
  return geometry;
}

function buildControlPointObject(
  points: EditableControlPoint[],
  pointIndices: number[],
  selectedId: string | null,
  opacity: number,
  renderOrder: number,
): THREE.Points {
  const positions: number[] = [];
  const colors: number[] = [];
  const normalColor = new THREE.Color(0xff4fa3);
  const selectedColor = new THREE.Color(0xffffff);
  for (const pointIndex of pointIndices) {
    const point = points[pointIndex];
    positions.push(...point.position);
    const color = point.id === selectedId ? selectedColor : normalColor;
    colors.push(color.r, color.g, color.b);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  geometry.setAttribute("color", new THREE.Float32BufferAttribute(colors, 3));
  const controlPointObject = new THREE.Points(
    geometry,
    new THREE.PointsMaterial({
      vertexColors: true,
      size: 0.022,
      sizeAttenuation: true,
      depthTest: false,
      transparent: true,
      opacity,
    }),
  );
  controlPointObject.userData.controlPointIndices = pointIndices;
  controlPointObject.renderOrder = renderOrder;
  return controlPointObject;
}

function addControlPointObjects(
  group: THREE.Group,
  points: EditableControlPoint[],
  selectedId: string | null,
  selectedRowIndex: number | null,
): void {
  const allIndices = points.map((_, index) => index);
  if (selectedRowIndex === null) {
    group.add(buildControlPointObject(points, allIndices, selectedId, 1, 7));
    return;
  }
  const otherIndices = allIndices.filter(
    (index) => points[index].rowIndex !== selectedRowIndex,
  );
  const selectedIndices = allIndices.filter(
    (index) => points[index].rowIndex === selectedRowIndex,
  );
  if (otherIndices.length) {
    group.add(buildControlPointObject(points, otherIndices, selectedId, 0.14, 6));
  }
  if (selectedIndices.length) {
    group.add(buildControlPointObject(points, selectedIndices, selectedId, 1, 7));
  }
}

function renderedControlPoint(
  runtime: ViewerRuntime,
  controlPointIndex: number,
): { object: THREE.Points; geometryIndex: number } | null {
  for (const child of runtime.controlPointGroup.children) {
    if (!(child instanceof THREE.Points)) {
      continue;
    }
    const indices = child.userData.controlPointIndices as number[] | undefined;
    const geometryIndex = indices?.indexOf(controlPointIndex) ?? -1;
    if (geometryIndex >= 0) {
      return { object: child, geometryIndex };
    }
  }
  return null;
}

function showEditingPlane(
  runtime: ViewerRuntime,
  point: EditableControlPoint | undefined,
  trajectory: TrajectoryDataset | null,
): void {
  clearGroup(runtime.editPlaneGroup);
  if (!point) {
    return;
  }
  const normal = new THREE.Vector3(...point.planeNormal).normalize();
  const planeOrigin = new THREE.Vector3(...point.planeOrigin);
  const planeOrientation = new THREE.Quaternion().setFromUnitVectors(
    new THREE.Vector3(0, 0, 1),
    normal,
  );
  const inverseOrientation = planeOrientation.clone().invert();
  const trajectoryRow = trajectory?.rows.find((row) => row.rowIndex === point.rowIndex);
  const worldPoints = trajectoryRow?.points.map(
    (trajectoryPoint) => new THREE.Vector3(...trajectoryPoint.position),
  ) ?? [new THREE.Vector3(...point.position)];
  const localPoints = worldPoints.map(
    (worldPoint) => worldPoint.clone().sub(planeOrigin).applyQuaternion(inverseOrientation),
  );
  const bounds = new THREE.Box3().setFromPoints(localPoints);
  const size = bounds.getSize(new THREE.Vector3());
  const center = bounds.getCenter(new THREE.Vector3());
  center.z = 0;
  const longestSpan = Math.max(size.x, size.y, 0.35);
  const padding = Math.max(0.08, longestSpan * 0.08);
  const width = Math.max(size.x + padding * 2, 0.35);
  const height = Math.max(size.y + padding * 2, 0.35);

  const planeGroup = new THREE.Group();
  planeGroup.position.copy(
    planeOrigin.clone().add(center.clone().applyQuaternion(planeOrientation)),
  );
  planeGroup.quaternion.copy(planeOrientation);

  const fill = new THREE.Mesh(
    new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({
      color: 0x2f9ee5,
      transparent: true,
      opacity: 0.13,
      depthTest: false,
      depthWrite: false,
      side: THREE.DoubleSide,
    }),
  );
  fill.renderOrder = 1;
  planeGroup.add(fill);

  const gridPositions: number[] = [];
  const gridDivisions = 12;
  for (let index = 0; index <= gridDivisions; index += 1) {
    const x = -width / 2 + (width * index) / gridDivisions;
    const y = -height / 2 + (height * index) / gridDivisions;
    gridPositions.push(x, -height / 2, 0, x, height / 2, 0);
    gridPositions.push(-width / 2, y, 0, width / 2, y, 0);
  }
  const gridGeometry = new THREE.BufferGeometry();
  gridGeometry.setAttribute(
    "position",
    new THREE.Float32BufferAttribute(gridPositions, 3),
  );
  const grid = new THREE.LineSegments(
    gridGeometry,
    new THREE.LineBasicMaterial({
      color: 0x61bfff,
      transparent: true,
      opacity: 0.55,
      depthTest: false,
      depthWrite: false,
    }),
  );
  grid.renderOrder = 2;
  planeGroup.add(grid);

  const borderGeometry = new THREE.BufferGeometry().setFromPoints([
    new THREE.Vector3(-width / 2, -height / 2, 0),
    new THREE.Vector3(width / 2, -height / 2, 0),
    new THREE.Vector3(width / 2, height / 2, 0),
    new THREE.Vector3(-width / 2, height / 2, 0),
  ]);
  const border = new THREE.LineLoop(
    borderGeometry,
    new THREE.LineBasicMaterial({
      color: 0x8bd2ff,
      transparent: true,
      opacity: 0.95,
      depthTest: false,
      depthWrite: false,
    }),
  );
  border.renderOrder = 3;
  planeGroup.add(border);

  runtime.editPlaneGroup.add(planeGroup);
}

export function TrajectoryViewer({
  surfaceBuffer,
  surfaceFileName,
  trajectory,
  controlPoints,
  selectedControlPointId,
  editingEnabled,
  layers,
  visualization,
  onSelectControlPoint,
  onMoveControlPoint,
}: TrajectoryViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const runtimeRef = useRef<ViewerRuntime | null>(null);
  const interactionRef = useRef({
    editingEnabled,
    onSelectControlPoint,
    onMoveControlPoint,
  });
  interactionRef.current = { editingEnabled, onSelectControlPoint, onMoveControlPoint };

  useEffect(() => {
    const container = containerRef.current;
    if (!container) {
      return;
    }
    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x1a1c20);
    const camera = new THREE.PerspectiveCamera(42, 1, 0.001, 1000);
    camera.up.set(0, 0, 1);
    camera.position.set(2, -2, 1.5);
    const renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    renderer.outputColorSpace = THREE.SRGBColorSpace;
    renderer.toneMapping = THREE.ACESFilmicToneMapping;
    renderer.toneMappingExposure = 1.15;
    container.appendChild(renderer.domElement);
    const controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.08;
    controls.screenSpacePanning = true;

    const hemisphereLight = new THREE.HemisphereLight(0xdcecff, 0x273140, 2.2);
    const keyLight = new THREE.DirectionalLight(0xffffff, 3);
    keyLight.position.set(2, -3, 5);
    const fillLight = new THREE.DirectionalLight(0x8db7ff, 1.7);
    fillLight.position.set(-4, 1, 2);
    const rimLight = new THREE.DirectionalLight(0xe9f1ff, 2.4);
    const rimTarget = new THREE.Object3D();
    scene.add(hemisphereLight, keyLight, fillLight, rimLight, rimTarget);
    rimLight.target = rimTarget;

    const surfaceGroup = new THREE.Group();
    const pathGroup = new THREE.Group();
    const sprayOffGroup = new THREE.Group();
    const sprayDirectionGroup = new THREE.Group();
    const controlPointGroup = new THREE.Group();
    const editPlaneGroup = new THREE.Group();
    controlPointGroup.renderOrder = 10;
    editPlaneGroup.renderOrder = 5;
    const axes = new THREE.AxesHelper(0.35);
    scene.add(
      surfaceGroup,
      pathGroup,
      sprayOffGroup,
      sprayDirectionGroup,
      editPlaneGroup,
      controlPointGroup,
      axes,
    );

    const runtime: ViewerRuntime = {
      renderer,
      camera,
      controls,
      scene,
      surfaceGroup,
      pathGroup,
      sprayOffGroup,
      sprayDirectionGroup,
      controlPointGroup,
      editPlaneGroup,
      controlPointData: [],
      dragState: null,
      axes,
      hemisphereLight,
      keyLight,
      fillLight,
      rimLight,
      rimTarget,
      resizeObserver: new ResizeObserver(() => undefined),
      animationFrame: 0,
    };

    const pointer = new THREE.Vector2();
    const raycaster = new THREE.Raycaster();
    raycaster.params.Points = { threshold: 0.025 };
    const updateRay = (event: PointerEvent) => {
      const bounds = renderer.domElement.getBoundingClientRect();
      pointer.set(
        ((event.clientX - bounds.left) / bounds.width) * 2 - 1,
        -((event.clientY - bounds.top) / bounds.height) * 2 + 1,
      );
      raycaster.setFromCamera(pointer, camera);
    };
    const pointerDown = (event: PointerEvent) => {
      if (!interactionRef.current.editingEnabled || !runtime.controlPointGroup.visible) {
        return;
      }
      updateRay(event);
      const hits = raycaster.intersectObject(runtime.controlPointGroup, true);
      const hit = hits.find(
        (candidate) => candidate.index !== undefined && candidate.object instanceof THREE.Points,
      );
      if (!hit || hit.index === undefined || !(hit.object instanceof THREE.Points)) {
        // Keep the current point and slicing plane selected while the user
        // rotates, pans, or zooms the Scene view.
        return;
      }
      const renderedIndices = hit.object.userData.controlPointIndices as number[] | undefined;
      const controlPointIndex = renderedIndices?.[hit.index];
      if (controlPointIndex === undefined) {
        return;
      }
      const point = runtime.controlPointData[controlPointIndex];
      if (!point) {
        return;
      }
      interactionRef.current.onSelectControlPoint(point.id);
      const normal = new THREE.Vector3(...point.planeNormal).normalize();
      const origin = new THREE.Vector3(...point.planeOrigin);
      const plane = new THREE.Plane().setFromNormalAndCoplanarPoint(normal, origin);
      const planeHit = raycaster.ray.intersectPlane(plane, new THREE.Vector3());
      const pointPosition = new THREE.Vector3(...point.position);
      runtime.dragState = {
        point,
        controlPointIndex,
        plane,
        offset: planeHit ? pointPosition.sub(planeHit) : new THREE.Vector3(),
        moved: false,
      };
      runtime.controls.enabled = false;
      renderer.domElement.setPointerCapture(event.pointerId);
      event.preventDefault();
    };
    const pointerMove = (event: PointerEvent) => {
      const drag = runtime.dragState;
      if (!drag) {
        return;
      }
      updateRay(event);
      const hit = raycaster.ray.intersectPlane(drag.plane, new THREE.Vector3());
      if (!hit) {
        return;
      }
      hit.add(drag.offset);
      drag.moved = drag.moved
        || hit.distanceToSquared(new THREE.Vector3(...drag.point.position)) > 1.0e-12;
      const rendered = renderedControlPoint(runtime, drag.controlPointIndex);
      if (drag.moved && rendered) {
        const positions = rendered.object.geometry.getAttribute("position") as THREE.BufferAttribute;
        positions.setXYZ(rendered.geometryIndex, hit.x, hit.y, hit.z);
        positions.needsUpdate = true;
      }
    };
    const pointerUp = (event: PointerEvent) => {
      const drag = runtime.dragState;
      if (!drag) {
        return;
      }
      const rendered = renderedControlPoint(runtime, drag.controlPointIndex);
      if (rendered) {
        const positions = rendered.object.geometry.getAttribute("position") as THREE.BufferAttribute;
        interactionRef.current.onMoveControlPoint(drag.point.id, [
          positions.getX(rendered.geometryIndex),
          positions.getY(rendered.geometryIndex),
          positions.getZ(rendered.geometryIndex),
        ]);
      }
      runtime.dragState = null;
      runtime.controls.enabled = true;
      if (renderer.domElement.hasPointerCapture(event.pointerId)) {
        renderer.domElement.releasePointerCapture(event.pointerId);
      }
    };
    renderer.domElement.addEventListener("pointerdown", pointerDown);
    renderer.domElement.addEventListener("pointermove", pointerMove);
    renderer.domElement.addEventListener("pointerup", pointerUp);
    renderer.domElement.addEventListener("pointercancel", pointerUp);

    const resize = () => {
      const width = Math.max(container.clientWidth, 1);
      const height = Math.max(container.clientHeight, 1);
      renderer.setSize(width, height, false);
      camera.aspect = width / height;
      camera.updateProjectionMatrix();
    };
    runtime.resizeObserver = new ResizeObserver(resize);
    runtime.resizeObserver.observe(container);
    resize();
    const rimDirection = new THREE.Vector3();
    const animate = () => {
      controls.update();
      rimDirection.copy(camera.position).sub(controls.target).normalize();
      rimLight.position.copy(controls.target).addScaledVector(rimDirection, -5);
      rimTarget.position.copy(controls.target);
      renderer.render(scene, camera);
      runtime.animationFrame = requestAnimationFrame(animate);
    };
    animate();
    runtimeRef.current = runtime;

    return () => {
      cancelAnimationFrame(runtime.animationFrame);
      runtime.resizeObserver.disconnect();
      renderer.domElement.removeEventListener("pointerdown", pointerDown);
      renderer.domElement.removeEventListener("pointermove", pointerMove);
      renderer.domElement.removeEventListener("pointerup", pointerUp);
      renderer.domElement.removeEventListener("pointercancel", pointerUp);
      controls.dispose();
      for (const group of [surfaceGroup, pathGroup, sprayOffGroup, sprayDirectionGroup, controlPointGroup, editPlaneGroup]) {
        clearGroup(group);
      }
      renderer.dispose();
      renderer.domElement.remove();
      runtimeRef.current = null;
    };
  }, []);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime || !surfaceBuffer) {
      return;
    }
    clearGroup(runtime.surfaceGroup);
    const geometry = surfaceFileName.toLowerCase().endsWith(".csv")
      ? parseCsvSurface(surfaceBuffer.slice(0))
      : new PLYLoader().parse(surfaceBuffer.slice(0));
    geometry.computeBoundingBox();
    if (!geometry.getAttribute("normal") && geometry.index) {
      geometry.computeVertexNormals();
    }
    const hasColors = Boolean(geometry.getAttribute("color"));
    const inspection = visualization.surfaceStyle === "inspection";
    let surface: THREE.Object3D;
    if (geometry.index) {
      surface = new THREE.Mesh(
        geometry,
        new THREE.MeshStandardMaterial({
          vertexColors: hasColors && !inspection,
          color: inspection ? 0xaeb4ba : hasColors ? 0xffffff : 0x98a7b7,
          roughness: inspection ? 0.62 : 0.72,
          metalness: inspection ? 0.04 : 0.08,
          side: THREE.DoubleSide,
        }),
      );
    } else {
      surface = new THREE.Points(
        geometry,
        new THREE.PointsMaterial({
          vertexColors: hasColors && !inspection,
          color: inspection ? 0xc2c6ca : hasColors ? 0xffffff : 0x98a7b7,
          size: 0.004,
          sizeAttenuation: true,
        }),
      );
    }
    runtime.surfaceGroup.add(surface);
    fitCamera(runtime);
  }, [surfaceBuffer, surfaceFileName, visualization.surfaceStyle]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    runtime.scene.background = new THREE.Color(
      visualization.background === "gray" ? 0x1a1c20 : 0x090a0c,
    );
    runtime.hemisphereLight.intensity = 2.2 * visualization.lightIntensity;
    runtime.keyLight.intensity = 3 * visualization.lightIntensity;
    runtime.fillLight.intensity = 1.7 * visualization.lightIntensity;
    runtime.rimLight.intensity = 2.4 * visualization.lightIntensity;
    runtime.rimLight.visible = visualization.rimLight;
  }, [visualization]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    clearGroup(runtime.pathGroup);
    clearGroup(runtime.sprayOffGroup);
    if (!trajectory) return;
    const selectedRowIndex = controlPoints.find(
      (point) => point.id === selectedControlPointId,
    )?.rowIndex ?? null;
    if (selectedRowIndex === null) {
      runtime.pathGroup.add(new THREE.LineSegments(
        buildTrajectorySegments(trajectory, true),
        new THREE.LineBasicMaterial({ color: 0x20e66a }),
      ));
      runtime.sprayOffGroup.add(new THREE.LineSegments(
        buildTrajectorySegments(trajectory, false),
        new THREE.LineBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.85 }),
      ));
    } else {
      const otherRow = (rowIndex: number) => rowIndex !== selectedRowIndex;
      const selectedRow = (rowIndex: number) => rowIndex === selectedRowIndex;
      const otherPaintPath = new THREE.LineSegments(
        buildTrajectorySegments(trajectory, true, otherRow),
        new THREE.LineBasicMaterial({
          color: 0x20e66a,
          transparent: true,
          opacity: 0.12,
          depthWrite: false,
        }),
      );
      const selectedPaintPath = new THREE.LineSegments(
        buildTrajectorySegments(trajectory, true, selectedRow),
        new THREE.LineBasicMaterial({
          color: 0x20e66a,
          transparent: true,
          opacity: 1,
          depthTest: false,
          depthWrite: false,
        }),
      );
      otherPaintPath.renderOrder = 1;
      selectedPaintPath.renderOrder = 4;
      runtime.pathGroup.add(
        otherPaintPath,
        selectedPaintPath,
      );
      const otherSprayOffPath = new THREE.LineSegments(
        buildTrajectorySegments(trajectory, false, otherRow),
        new THREE.LineBasicMaterial({
          color: 0xffc857,
          transparent: true,
          opacity: 0.12,
          depthWrite: false,
        }),
      );
      const selectedSprayOffPath = new THREE.LineSegments(
        buildTrajectorySegments(trajectory, false, selectedRow),
        new THREE.LineBasicMaterial({
          color: 0xffc857,
          transparent: true,
          opacity: 0.9,
          depthTest: false,
          depthWrite: false,
        }),
      );
      otherSprayOffPath.renderOrder = 1;
      selectedSprayOffPath.renderOrder = 4;
      runtime.sprayOffGroup.add(
        otherSprayOffPath,
        selectedSprayOffPath,
      );
    }
    fitCamera(runtime);
  }, [trajectory, controlPoints, selectedControlPointId]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    clearGroup(runtime.sprayDirectionGroup);
    const selectedRowIndex = controlPoints.find(
      (point) => point.id === selectedControlPointId,
    )?.rowIndex ?? null;
    if (editingEnabled && controlPoints.length) {
      runtime.sprayDirectionGroup.add(
        buildEditableDirections(controlPoints, selectedRowIndex),
      );
    } else if (trajectory) {
      runtime.sprayDirectionGroup.add(buildSprayDirections(trajectory));
    }
  }, [trajectory, controlPoints, selectedControlPointId, editingEnabled]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    clearGroup(runtime.controlPointGroup);
    runtime.controlPointData = controlPoints;
    if (controlPoints.length) {
      const selectedRowIndex = controlPoints.find(
        (point) => point.id === selectedControlPointId,
      )?.rowIndex ?? null;
      addControlPointObjects(
        runtime.controlPointGroup,
        controlPoints,
        selectedControlPointId,
        selectedRowIndex,
      );
    }
    showEditingPlane(
      runtime,
      controlPoints.find((point) => point.id === selectedControlPointId),
      trajectory,
    );
  }, [controlPoints, selectedControlPointId, trajectory]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    runtime.surfaceGroup.visible = layers.surface;
    runtime.pathGroup.visible = layers.trajectory;
    runtime.sprayOffGroup.visible = layers.trajectory && layers.sprayOff;
    runtime.sprayDirectionGroup.visible = layers.sprayDirections;
    runtime.controlPointGroup.visible = layers.controlPoints && editingEnabled;
    runtime.editPlaneGroup.visible = layers.controlPoints && editingEnabled;
    runtime.axes.visible = layers.axes;
  }, [layers, editingEnabled]);

  const setView = (direction: THREE.Vector3) => {
    const runtime = runtimeRef.current;
    if (!runtime) return;
    const target = runtime.controls.target.clone();
    const distance = runtime.camera.position.distanceTo(target);
    runtime.camera.position.copy(target.clone().add(direction.normalize().multiplyScalar(distance)));
    runtime.camera.up.set(0, 0, 1);
    runtime.controls.update();
  };

  return (
    <div className="viewer-shell">
      <div ref={containerRef} className="viewer-canvas" />
      <div className="view-controls" aria-label="카메라 시점">
        <button type="button" onClick={() => setView(new THREE.Vector3(0, 0, 1))}>TOP</button>
        <button type="button" onClick={() => setView(new THREE.Vector3(1, 0, 0))}>X</button>
        <button type="button" onClick={() => setView(new THREE.Vector3(0, -1, 0))}>Y</button>
        <button type="button" onClick={() => runtimeRef.current && fitCamera(runtimeRef.current)}>FIT</button>
      </div>
      <div className="viewer-hint">
        {editingEnabled
          ? "포인트 선택/이동: 좌클릭 드래그 · 이동은 현재 슬라이싱 평면으로 제한"
          : "회전: 좌클릭 · 이동: 우클릭 · 확대: 휠"}
      </div>
    </div>
  );
}
