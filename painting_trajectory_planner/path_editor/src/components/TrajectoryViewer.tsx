import { useEffect, useRef } from "react";
import * as THREE from "three";
import { OrbitControls } from "three/addons/controls/OrbitControls.js";
import { PLYLoader } from "three/addons/loaders/PLYLoader.js";
import type {
  LayerVisibility,
  TrajectoryDataset,
  VisualizationSettings,
} from "../types";

interface TrajectoryViewerProps {
  surfaceBuffer: ArrayBuffer | null;
  trajectory: TrajectoryDataset | null;
  layers: LayerVisibility;
  visualization: VisualizationSettings;
  selectedRowId: number | null;
}

interface ViewerRuntime {
  renderer: THREE.WebGLRenderer;
  camera: THREE.PerspectiveCamera;
  controls: OrbitControls;
  scene: THREE.Scene;
  surfaceGroup: THREE.Group;
  pathGroup: THREE.Group;
  sprayOffGroup: THREE.Group;
  selectedGroup: THREE.Group;
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
    if (child instanceof THREE.Mesh || child instanceof THREE.Points || child instanceof THREE.Line) {
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
  const content = new THREE.Group();
  for (const group of [runtime.surfaceGroup, runtime.pathGroup, runtime.sprayOffGroup]) {
    for (const child of group.children) {
      content.add(child.clone());
    }
  }
  const bounds = new THREE.Box3().setFromObject(content);
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
): THREE.BufferGeometry {
  const positions: number[] = [];
  for (const row of trajectory.rows) {
    for (let index = 1; index < row.points.length; index += 1) {
      const previous = row.points[index - 1];
      const current = row.points[index];
      const isPaintSegment = previous.paint && current.paint;
      if (isPaintSegment !== paintState) {
        continue;
      }
      positions.push(...previous.position, ...current.position);
    }
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
  return geometry;
}

export function TrajectoryViewer({
  surfaceBuffer,
  trajectory,
  layers,
  visualization,
  selectedRowId,
}: TrajectoryViewerProps) {
  const containerRef = useRef<HTMLDivElement>(null);
  const runtimeRef = useRef<ViewerRuntime | null>(null);

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
    const keyLight = new THREE.DirectionalLight(0xffffff, 3.0);
    keyLight.position.set(2, -3, 5);
    const fillLight = new THREE.DirectionalLight(0x8db7ff, 1.7);
    fillLight.position.set(-4, 1, 2);
    const rimLight = new THREE.DirectionalLight(0xe9f1ff, 2.4);
    const rimTarget = new THREE.Object3D();
    scene.add(hemisphereLight, keyLight, fillLight, rimLight, rimTarget);
    rimLight.target = rimTarget;

    const surfaceGroup = new THREE.Group();
    surfaceGroup.name = "surface";
    const pathGroup = new THREE.Group();
    pathGroup.name = "trajectory";
    const sprayOffGroup = new THREE.Group();
    sprayOffGroup.name = "spray-off";
    const selectedGroup = new THREE.Group();
    selectedGroup.name = "selected-row";
    const axes = new THREE.AxesHelper(0.35);
    scene.add(surfaceGroup, pathGroup, sprayOffGroup, selectedGroup, axes);

    const runtime: ViewerRuntime = {
      renderer,
      camera,
      controls,
      scene,
      surfaceGroup,
      pathGroup,
      sprayOffGroup,
      selectedGroup,
      axes,
      hemisphereLight,
      keyLight,
      fillLight,
      rimLight,
      rimTarget,
      resizeObserver: new ResizeObserver(() => undefined),
      animationFrame: 0,
    };
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
      controls.dispose();
      clearGroup(surfaceGroup);
      clearGroup(pathGroup);
      clearGroup(sprayOffGroup);
      clearGroup(selectedGroup);
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

    const geometry = new PLYLoader().parse(surfaceBuffer.slice(0));
    geometry.computeBoundingBox();
    if (!geometry.getAttribute("normal") && geometry.index) {
      geometry.computeVertexNormals();
    }
    const hasColors = Boolean(geometry.getAttribute("color"));
    let surface: THREE.Object3D;
    const inspection = visualization.surfaceStyle === "inspection";
    if (geometry.index) {
      const material = new THREE.MeshStandardMaterial({
        vertexColors: hasColors && !inspection,
        color: inspection ? 0xaeb4ba : hasColors ? 0xffffff : 0x98a7b7,
        roughness: inspection ? 0.62 : 0.72,
        metalness: inspection ? 0.04 : 0.08,
        side: THREE.DoubleSide,
      });
      surface = new THREE.Mesh(geometry, material);
    } else {
      const material = new THREE.PointsMaterial({
        vertexColors: hasColors && !inspection,
        color: inspection ? 0xc2c6ca : hasColors ? 0xffffff : 0x98a7b7,
        size: 0.004,
        sizeAttenuation: true,
      });
      surface = new THREE.Points(geometry, material);
    }
    runtime.surfaceGroup.add(surface);
    fitCamera(runtime);
  }, [surfaceBuffer, visualization.surfaceStyle]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) {
      return;
    }
    runtime.scene.background = new THREE.Color(
      visualization.background === "gray" ? 0x1a1c20 : 0x090a0c,
    );
    runtime.hemisphereLight.intensity = 2.2 * visualization.lightIntensity;
    runtime.keyLight.intensity = 3 * visualization.lightIntensity;
    runtime.fillLight.intensity = 1.7 * visualization.lightIntensity;
    runtime.rimLight.intensity = 2.4 * visualization.lightIntensity;
    runtime.rimLight.visible = visualization.rimLight;
  }, [
    visualization.background,
    visualization.lightIntensity,
    visualization.rimLight,
  ]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime || !trajectory) {
      return;
    }
    clearGroup(runtime.pathGroup);
    clearGroup(runtime.sprayOffGroup);

    const paintGeometry = buildTrajectorySegments(trajectory, true);
    const paintPath = new THREE.LineSegments(
      paintGeometry,
      new THREE.LineBasicMaterial({ color: 0x20e66a }),
    );
    runtime.pathGroup.add(paintPath);

    const offGeometry = buildTrajectorySegments(trajectory, false);
    const offPath = new THREE.LineSegments(
      offGeometry,
      new THREE.LineBasicMaterial({ color: 0xffc857, transparent: true, opacity: 0.85 }),
    );
    runtime.sprayOffGroup.add(offPath);
    fitCamera(runtime);
  }, [trajectory]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) {
      return;
    }
    runtime.surfaceGroup.visible = layers.surface;
    runtime.pathGroup.visible = layers.trajectory;
    runtime.sprayOffGroup.visible = layers.trajectory && layers.sprayOff;
    runtime.selectedGroup.visible = layers.trajectory;
    runtime.axes.visible = layers.axes;
  }, [layers]);

  useEffect(() => {
    const runtime = runtimeRef.current;
    if (!runtime) {
      return;
    }
    clearGroup(runtime.selectedGroup);
    if (selectedRowId === null || !trajectory) {
      return;
    }
    const selected = trajectory.rows.find((row) => row.rowIndex === selectedRowId);
    if (!selected || selected.points.length < 2) {
      return;
    }
    const positions = selected.points.flatMap((point) => point.position);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.Float32BufferAttribute(positions, 3));
    runtime.selectedGroup.add(
      new THREE.Line(
        geometry,
        new THREE.LineBasicMaterial({ color: 0xff4fd8, depthTest: false }),
      ),
    );
  }, [selectedRowId, trajectory]);

  const setView = (direction: THREE.Vector3) => {
    const runtime = runtimeRef.current;
    if (!runtime) {
      return;
    }
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
        <button
          type="button"
          onClick={() => runtimeRef.current && fitCamera(runtimeRef.current)}
        >
          FIT
        </button>
      </div>
      <div className="viewer-hint">회전: 좌클릭 · 이동: 우클릭 · 확대: 휠</div>
    </div>
  );
}
