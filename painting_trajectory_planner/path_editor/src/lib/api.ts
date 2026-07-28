import { invoke } from "@tauri-apps/api/core";
import type { ProjectInfo } from "../types";

type ProjectFileKind = "surface" | "trajectory";

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
  if (!isTauriRuntime()) {
    return (await checkedFetch("/projects")).json() as Promise<ProjectInfo[]>;
  }
  return invoke<ProjectInfo[]>("list_projects");
}

export async function readProjectFile(
  projectName: string,
  kind: ProjectFileKind,
): Promise<ArrayBuffer> {
  if (!isTauriRuntime()) {
    const project = encodeURIComponent(projectName);
    return (await checkedFetch(`/project/${project}/${kind}`)).arrayBuffer();
  }
  return invoke<ArrayBuffer>("read_project_file", { projectName, kind });
}

export async function readParameters(projectName: string): Promise<string> {
  if (!isTauriRuntime()) {
    const project = encodeURIComponent(projectName);
    return (await checkedFetch(`/project/${project}/parameters`)).text();
  }
  return invoke<string>("read_parameters", { projectName });
}

export async function getDataRoot(): Promise<string> {
  if (!isTauriRuntime()) {
    return (await checkedFetch("/data-root")).text();
  }
  return invoke<string>("get_data_root");
}
