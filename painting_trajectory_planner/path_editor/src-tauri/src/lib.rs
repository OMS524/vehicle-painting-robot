use serde::Serialize;
use std::{
    env, fs,
    path::{Component, Path, PathBuf},
};
use tauri::ipc::Response;

struct DataRoot(PathBuf);

#[derive(Serialize)]
#[serde(rename_all = "camelCase")]
struct ProjectInfo {
    name: String,
    surface_file: String,
    trajectory_file: String,
    surface_bytes: u64,
    trajectory_bytes: u64,
}

fn is_data_root(path: &Path) -> bool {
    fs::read_dir(path).ok().is_some_and(|entries| {
        entries.filter_map(Result::ok).any(|entry| {
            let project_dir = entry.path();
            project_dir.is_dir()
                && project_dir.join("painting_trajectory.csv").is_file()
                && project_dir.join("painting_trajectory.yaml").is_file()
                && fs::read_dir(&project_dir).ok().is_some_and(|files| {
                    files.filter_map(Result::ok).any(|file| {
                        file.path().is_file()
                            && file
                                .path()
                                .extension()
                                .is_some_and(|extension| extension.eq_ignore_ascii_case("ply"))
                    })
                })
        })
    })
}

fn resolve_data_root() -> Result<PathBuf, String> {
    let mut candidates = Vec::new();

    if let Some(configured) = env::var_os("PATH_EDITOR_DATA_DIR") {
        candidates.push(PathBuf::from(configured));
    }
    if let Some(app_image) = env::var_os("APPIMAGE") {
        if let Some(parent) = PathBuf::from(app_image).parent() {
            candidates.push(parent.join("data"));
        }
    }
    if let Ok(executable) = env::current_exe() {
        if let Some(parent) = executable.parent() {
            candidates.push(parent.join("data"));
            candidates.push(parent.join("..").join("data"));
        }
    }
    if let Ok(current_dir) = env::current_dir() {
        candidates.push(current_dir.join("data"));
        candidates.push(current_dir.join("path_editor").join("data"));
    }
    candidates.push(
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("..")
            .join("data"),
    );

    for candidate in candidates {
        let normalized = candidate.canonicalize().unwrap_or(candidate);
        if is_data_root(&normalized) {
            return Ok(normalized);
        }
    }
    Err(
        "에디터 데이터 디렉터리를 찾지 못했습니다. 실행 파일 옆에 data 폴더를 두거나 \
         PATH_EDITOR_DATA_DIR 환경변수를 지정하세요."
            .to_string(),
    )
}

fn validate_project_name(project_name: &str) -> Result<(), String> {
    let path = Path::new(project_name);
    let mut components = path.components();
    match (components.next(), components.next()) {
        (Some(Component::Normal(name)), None)
            if name.to_string_lossy().as_ref() == project_name =>
        {
            Ok(())
        }
        _ => Err("올바르지 않은 프로젝트 이름입니다.".to_string()),
    }
}

fn find_surface_file(project_dir: &Path) -> Result<PathBuf, String> {
    let mut candidates = fs::read_dir(project_dir)
        .map_err(|error| format!("프로젝트 디렉터리를 읽지 못했습니다: {error}"))?
        .filter_map(Result::ok)
        .map(|entry| entry.path())
        .filter(|path| {
            path.is_file()
                && path
                    .extension()
                    .is_some_and(|extension| extension.eq_ignore_ascii_case("ply"))
        })
        .collect::<Vec<_>>();
    candidates.sort();
    candidates
        .into_iter()
        .next()
        .ok_or_else(|| "프로젝트에 PLY 표면 파일이 없습니다.".to_string())
}

fn project_paths(data_root: &Path, project_name: &str) -> Result<(PathBuf, PathBuf), String> {
    validate_project_name(project_name)?;
    let project_dir = data_root.join(project_name);
    if !project_dir.is_dir() {
        return Err(format!("프로젝트를 찾을 수 없습니다: {project_name}"));
    }
    let surface_path = find_surface_file(&project_dir)?;
    let trajectory_path = project_dir.join("painting_trajectory.csv");
    if !trajectory_path.is_file() {
        return Err("프로젝트에 painting_trajectory.csv가 없습니다.".to_string());
    }
    Ok((surface_path, trajectory_path))
}

#[tauri::command]
fn get_data_root(data_root: tauri::State<'_, DataRoot>) -> String {
    data_root.0.display().to_string()
}

#[tauri::command]
fn list_projects(data_root: tauri::State<'_, DataRoot>) -> Result<Vec<ProjectInfo>, String> {
    let mut project_names = fs::read_dir(&data_root.0)
        .map_err(|error| format!("프로젝트 목록을 읽지 못했습니다: {error}"))?
        .filter_map(Result::ok)
        .filter(|entry| entry.path().is_dir())
        .filter_map(|entry| entry.file_name().into_string().ok())
        .collect::<Vec<_>>();
    project_names.sort();

    let mut projects = Vec::new();
    for name in project_names {
        let Ok((surface_path, trajectory_path)) = project_paths(&data_root.0, &name) else {
            continue;
        };
        let surface_metadata = fs::metadata(&surface_path)
            .map_err(|error| format!("PLY 정보를 읽지 못했습니다: {error}"))?;
        let trajectory_metadata = fs::metadata(&trajectory_path)
            .map_err(|error| format!("trajectory 정보를 읽지 못했습니다: {error}"))?;
        projects.push(ProjectInfo {
            name,
            surface_file: surface_path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
            trajectory_file: trajectory_path
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .into_owned(),
            surface_bytes: surface_metadata.len(),
            trajectory_bytes: trajectory_metadata.len(),
        });
    }
    Ok(projects)
}

#[tauri::command]
fn read_project_file(
    data_root: tauri::State<'_, DataRoot>,
    project_name: String,
    kind: String,
) -> Result<Response, String> {
    let (surface_path, trajectory_path) = project_paths(&data_root.0, &project_name)?;
    let file_path = match kind.as_str() {
        "surface" => surface_path,
        "trajectory" => trajectory_path,
        _ => return Err("지원하지 않는 프로젝트 파일 종류입니다.".to_string()),
    };
    let bytes = fs::read(&file_path)
        .map_err(|error| format!("{} 파일을 읽지 못했습니다: {error}", file_path.display()))?;
    Ok(Response::new(bytes))
}

#[tauri::command]
fn read_parameters(
    data_root: tauri::State<'_, DataRoot>,
    project_name: String,
) -> Result<String, String> {
    validate_project_name(&project_name)?;
    let project_dir = data_root.0.join(&project_name);
    if !project_dir.is_dir() {
        return Err(format!("프로젝트를 찾을 수 없습니다: {project_name}"));
    }
    let path = project_dir.join("painting_trajectory.yaml");
    fs::read_to_string(&path)
        .map_err(|error| format!("{} 파일을 읽지 못했습니다: {error}", path.display()))
}

#[cfg_attr(mobile, tauri::mobile_entry_point)]
pub fn run() {
    let data_root = resolve_data_root().unwrap_or_else(|error| panic!("{error}"));
    tauri::Builder::default()
        .manage(DataRoot(data_root))
        .invoke_handler(tauri::generate_handler![
            get_data_root,
            list_projects,
            read_project_file,
            read_parameters
        ])
        .run(tauri::generate_context!())
        .expect("Painting Trajectory Reviewer 실행 중 오류가 발생했습니다.");
}
