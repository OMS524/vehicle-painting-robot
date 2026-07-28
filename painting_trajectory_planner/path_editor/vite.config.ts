import { readFile, readdir, stat } from "node:fs/promises";
import { resolve } from "node:path";
import { defineConfig, type Plugin, type PreviewServer, type ViteDevServer } from "vite";
import react from "@vitejs/plugin-react";

const dataRoot = resolve(process.cwd(), "data");

function attachPreviewApi(server: ViteDevServer | PreviewServer): void {
  server.middlewares.use("/__reviewer", async (request, response, next) => {
    try {
      const pathname = decodeURIComponent(
        new URL(request.url ?? "/", "http://reviewer.local").pathname,
      );
      if (pathname === "/projects") {
        const entries = await readdir(dataRoot, { withFileTypes: true });
        const projects = [];
        for (const entry of entries.filter((item) => item.isDirectory())) {
          const projectDir = resolve(dataRoot, entry.name);
          const files = await readdir(projectDir);
          const surfaceFile = files.find((name) => name.toLowerCase().endsWith(".ply"));
          const trajectoryFile = files.find((name) => name === "painting_trajectory.csv");
          const parameterFile = files.find((name) => name === "painting_trajectory.yaml");
          if (!surfaceFile || !trajectoryFile || !parameterFile) {
            continue;
          }
          const [surfaceInfo, trajectoryInfo] = await Promise.all([
            stat(resolve(projectDir, surfaceFile)),
            stat(resolve(projectDir, trajectoryFile)),
          ]);
          projects.push({
            name: entry.name,
            surfaceFile,
            trajectoryFile,
            surfaceBytes: surfaceInfo.size,
            trajectoryBytes: trajectoryInfo.size,
          });
        }
        projects.sort((left, right) => left.name.localeCompare(right.name));
        response.setHeader("Content-Type", "application/json");
        response.end(JSON.stringify(projects));
        return;
      }
      if (pathname === "/data-root") {
        response.setHeader("Content-Type", "text/plain; charset=utf-8");
        response.end(`${dataRoot} (browser preview)`);
        return;
      }

      const match = pathname.match(
        /^\/project\/([^/]+)\/(surface|trajectory|parameters)$/,
      );
      if (match) {
        const [, projectName, kind] = match;
        if (
          !projectName
          || projectName === "."
          || projectName === ".."
          || projectName.includes("\\")
        ) {
          response.statusCode = 400;
          response.end("Invalid project name");
          return;
        }
        const projectDir = resolve(dataRoot, projectName);
        const files = await readdir(projectDir);
        const fileName = kind === "surface"
          ? files.find((name) => name.toLowerCase().endsWith(".ply"))
          : kind === "trajectory"
            ? "painting_trajectory.csv"
            : "painting_trajectory.yaml";
        if (!fileName) {
          response.statusCode = 404;
          response.end("Project file not found");
          return;
        }
        response.setHeader(
          "Content-Type",
          kind === "parameters"
            ? "text/yaml; charset=utf-8"
            : "application/octet-stream",
        );
        response.end(await readFile(resolve(projectDir, fileName)));
        return;
      }
      next();
    } catch (error) {
      response.statusCode = 500;
      response.end(error instanceof Error ? error.message : String(error));
    }
  });
}

function reviewerPreviewApi(): Plugin {
  return {
    name: "reviewer-preview-api",
    configureServer: attachPreviewApi,
    configurePreviewServer: attachPreviewApi,
  };
}

export default defineConfig({
  plugins: [react(), reviewerPreviewApi()],
  clearScreen: false,
  server: {
    port: 1420,
    strictPort: true,
  },
  envPrefix: ["VITE_", "TAURI_"],
  build: {
    target: process.env.TAURI_ENV_PLATFORM === "windows" ? "chrome105" : "safari13",
    minify: process.env.TAURI_ENV_DEBUG ? false : "esbuild",
    sourcemap: Boolean(process.env.TAURI_ENV_DEBUG),
  },
});
