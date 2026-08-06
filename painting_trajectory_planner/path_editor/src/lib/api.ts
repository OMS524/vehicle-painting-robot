import { invoke } from "@tauri-apps/api/core";
import type { ProjectInfo } from "../types";

type ProjectFileKind = "surface" | "trajectory";

interface StandaloneProject {
  info: ProjectInfo;
  surfaceBase64: string;
  trajectoryBase64: string;
  parametersBase64: string;
  sourceLabel: string;
}

declare global {
  interface Window {
    __PAINTING_TRAJECTORY_STANDALONE__?: StandaloneProject;
  }
}

function standaloneProject(): StandaloneProject | undefined {
  return window.__PAINTING_TRAJECTORY_STANDALONE__;
}

function decodeBase64(base64: string): ArrayBuffer {
  const binary = atob(base64);
  const bytes = new Uint8Array(binary.length);
  for (let index = 0; index < binary.length; index += 1) {
    bytes[index] = binary.charCodeAt(index);
  }
  return bytes.buffer;
}

function validateStandaloneProject(
  project: StandaloneProject,
  projectName: string,
): void {
  if (project.info.name !== projectName) {
    throw new Error(`내장 프로젝트를 찾을 수 없습니다: ${projectName}`);
  }
}

function isTauriRuntime(): boolean {
  return "__TAURI_INTERNALS__" in window;
}

async function checkedFetch(path: string): Promise<Response> {
  const response = await fetch(`/__reviewer${path}`);
  if (!response.ok) {
    throw new Error(await response.text());
  }
  return response;
}

export async function listProjects(): Promise<ProjectInfo[]> {
  const embedded = standaloneProject();
  if (embedded) {
    return [embedded.info];
  }
  if (!isTauriRuntime()) {
    return (await checkedFetch("/projects")).json() as Promise<ProjectInfo[]>;
  }
  return invoke<ProjectInfo[]>("list_projects");
}

export async function readProjectFile(
  projectName: string,
  kind: ProjectFileKind,
): Promise<ArrayBuffer> {
  const embedded = standaloneProject();
  if (embedded) {
    validateStandaloneProject(embedded, projectName);
    return decodeBase64(
      kind === "surface" ? embedded.surfaceBase64 : embedded.trajectoryBase64,
    );
  }
  if (!isTauriRuntime()) {
    const project = encodeURIComponent(projectName);
    return (await checkedFetch(`/project/${project}/${kind}`)).arrayBuffer();
  }
  return invoke<ArrayBuffer>("read_project_file", { projectName, kind });
}

export async function readParameters(projectName: string): Promise<string> {
  const embedded = standaloneProject();
  if (embedded) {
    validateStandaloneProject(embedded, projectName);
    return new TextDecoder("utf-8").decode(
      decodeBase64(embedded.parametersBase64),
    );
  }
  if (!isTauriRuntime()) {
    const project = encodeURIComponent(projectName);
    return (await checkedFetch(`/project/${project}/parameters`)).text();
  }
  return invoke<string>("read_parameters", { projectName });
}

export async function getDataRoot(): Promise<string> {
  const embedded = standaloneProject();
  if (embedded) {
    return embedded.sourceLabel;
  }
  if (!isTauriRuntime()) {
    return (await checkedFetch("/data-root")).text();
  }
  return invoke<string>("get_data_root");
}
