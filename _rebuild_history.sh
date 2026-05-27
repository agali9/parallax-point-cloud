#!/usr/bin/env bash
set -euo pipefail
cd "/mnt/c/Users/aniru/OneDrive/Documents/Projects/Point Cloud Processing Pipeline"

export GIT_AUTHOR_NAME="agali9"
export GIT_AUTHOR_EMAIL="agali9@asu.edu"
export GIT_COMMITTER_NAME="agali9"
export GIT_COMMITTER_EMAIL="agali9@asu.edu"

commit_tree() {
  local tree="$1" parent="$2" date="$3" msg="$4"
  export GIT_AUTHOR_DATE="$date"
  export GIT_COMMITTER_DATE="$date"
  if [[ -z "$parent" ]]; then
    git commit-tree "$tree" -m "$msg"
  else
    git commit-tree "$tree" -p "$parent" -m "$msg"
  fi
}

# Core progressive history (same trees/dates as original 3ab4f0c chain)
declare -a SHAS=(
  3959a32 6a20421 7c5cb77 7c347f0 5bab9f7 9c340c6 262c4bb
  50ac4df 596af25 3218980 9568fd5 e76d4a4 17f1415 3ab4f0c
)
declare -a DATES=(
  "2026-03-07T21:14:00-07:00"
  "2026-03-14T18:42:00-07:00"
  "2026-03-21T20:07:00-07:00"
  "2026-03-28T16:35:00-07:00"
  "2026-04-04T22:11:00-07:00"
  "2026-04-11T17:58:00-07:00"
  "2026-04-18T19:26:00-07:00"
  "2026-04-24T23:03:00-07:00"
  "2026-04-30T21:49:00-07:00"
  "2026-05-04T22:37:00-07:00"
  "2026-05-08T20:18:00-07:00"
  "2026-05-11T23:12:00-07:00"
  "2026-05-14T21:44:00-07:00"
  "2026-05-16T18:06:00-07:00"
)
declare -a MSGS=(
  "wip open3d point conversion"
  "scaffold pipeline types and config"
  "filter nan/inf and pass-through bounds"
  "add voxel grid downsampling"
  "euclidean clustering first cut"
  "rough 100k point benchmark"
  "add stage timings to pipeline result"
  "speed up neighbor search with grids"
  "turn it into a cmake shared lib"
  "start pybind11 bindings"
  "numpy zero-copy in/out for bindings"
  "stub ros2 pipeline node"
  "publish processed cloud + markers"
  "docs cleanup before release"
)

parent=""
for i in "${!SHAS[@]}"; do
  tree=$(git rev-parse "${SHAS[$i]}^{tree}")
  parent=$(commit_tree "$tree" "$parent" "${DATES[$i]}" "${MSGS[$i]}")
  echo "OK $parent  ${MSGS[$i]}"
done

git checkout -B main "$parent"
echo "pre-cuda tip=$parent"

# ---- CUDA 1: stubs ----
mkdir -p include/pointcloud_pipeline src
cat > include/pointcloud_pipeline/cuda_backend.hpp <<'EOF'
#pragma once

#include "pointcloud_pipeline/config.hpp"
#include "pointcloud_pipeline/types.hpp"

namespace pointcloud_pipeline {

[[nodiscard]] bool isCudaAvailable() noexcept;
[[nodiscard]] const char* cudaDeviceName() noexcept;

}  // namespace pointcloud_pipeline
EOF

cat > src/cuda_backend.cpp <<'EOF'
#include "pointcloud_pipeline/cuda_backend.hpp"

namespace pointcloud_pipeline {

bool isCudaAvailable() noexcept { return false; }

const char* cudaDeviceName() noexcept { return nullptr; }

}  // namespace pointcloud_pipeline
EOF

python3 - <<'PY'
from pathlib import Path
cfg = Path("include/pointcloud_pipeline/config.hpp").read_text()
if "ExecutionBackend" not in cfg:
    cfg = cfg.replace(
        "namespace pointcloud_pipeline {",
        "namespace pointcloud_pipeline {\n\nenum class ExecutionBackend {\n    Auto,\n    CPU,\n    CUDA,\n};",
        1,
    )
    cfg = cfg.replace(
        "bool enable_downsampling = true;",
        "bool enable_downsampling = true;\n    ExecutionBackend backend = ExecutionBackend::Auto;",
        1,
    )
    Path("include/pointcloud_pipeline/config.hpp").write_text(cfg)

types = Path("include/pointcloud_pipeline/types.hpp").read_text()
if "used_cuda" not in types:
    types = types.replace(
        "double total_ms;",
        "double total_ms;\n    double h2d_ms = 0.0;\n    double d2h_ms = 0.0;\n    bool used_cuda = false;",
        1,
    )
    Path("include/pointcloud_pipeline/types.hpp").write_text(types)

cmake = Path("CMakeLists.txt").read_text().splitlines()
out = []
added_opt = False
added_src = False
for line in cmake:
    if (not added_opt) and "option(POINTCLOUD_PIPELINE_USE_OPEN3D" in line:
        out.append(line)
        out.append('option(POINTCLOUD_PIPELINE_USE_CUDA "Build CUDA GPU backend when the CUDA toolkit is available" ON)')
        added_opt = True
        continue
    if (not added_src) and line.strip() == "src/filtering.cpp":
        out.append("    src/cuda_backend.cpp")
        added_src = True
    out.append(line)
Path("CMakeLists.txt").write_text("\n".join(out) + "\n")
PY

git add include/pointcloud_pipeline/cuda_backend.hpp src/cuda_backend.cpp \
  include/pointcloud_pipeline/config.hpp include/pointcloud_pipeline/types.hpp CMakeLists.txt
parent=$(commit_tree "$(git write-tree)" "$parent" "2026-08-27T22:18:00-07:00" "stub cuda backend + cmake flag")
git reset --hard "$parent"
echo "OK $parent  stub cuda"

# ---- CUDA 2: kernels ----
git checkout 37e04e6 -- src/cuda include/pointcloud_pipeline/cuda_backend.hpp src/cuda_backend.cpp CMakeLists.txt
git add src/cuda include/pointcloud_pipeline/cuda_backend.hpp src/cuda_backend.cpp CMakeLists.txt
parent=$(commit_tree "$(git write-tree)" "$parent" "2026-08-28T14:42:00-07:00" "gpu filter/voxel kernels via thrust")
git reset --hard "$parent"
echo "OK $parent  kernels"

# ---- CUDA 3: wire ----
git checkout 37e04e6 -- \
  src/pipeline.cpp \
  python/bindings.cpp \
  ros2/pointcloud_pipeline_ros/src/pipeline_node.cpp \
  ros2/pointcloud_pipeline_ros/launch/pipeline.launch.py \
  tests/test_cuda_parity.cpp \
  benchmarks/pipeline_benchmark.cpp \
  include/pointcloud_pipeline/pointcloud_pipeline.hpp \
  include/pointcloud_pipeline/config.hpp \
  include/pointcloud_pipeline/types.hpp
git add -A
# never include context file
git rm -f --ignore-unmatch pointcloud-pipeline_context.md >/dev/null 2>&1 || true
rm -f pointcloud-pipeline_context.md
parent=$(commit_tree "$(git write-tree)" "$parent" "2026-08-28T19:05:00-07:00" "wire cuda path through pipeline and tests")
git reset --hard "$parent"
echo "OK $parent  wire"

# ---- CUDA 4: docs/readme final, still no context ----
git checkout 37e04e6 -- .
git rm -f --ignore-unmatch pointcloud-pipeline_context.md >/dev/null 2>&1 || true
rm -f pointcloud-pipeline_context.md

# ensure gitignore covers context + cuda results
if ! grep -q 'pointcloud-pipeline_context.md' .gitignore; then
  printf '\npointcloud-pipeline_context.md\n' >> .gitignore
fi
if ! grep -q 'latest_cuda_results.md' .gitignore; then
  printf 'benchmarks/latest_cuda_results.md\n' >> .gitignore
fi
git rm -f --cached --ignore-unmatch benchmarks/latest_cuda_results.md benchmarks/latest_results.md >/dev/null 2>&1 || true
rm -f benchmarks/latest_cuda_results.md benchmarks/latest_results.md

# drop helper scripts
rm -f _rewrite*.ps1 _rewrite*.sh _fix_tip.* _msg.txt _rebuild*.sh

git add -A
git rm -f --cached --ignore-unmatch pointcloud-pipeline_context.md >/dev/null 2>&1 || true
parent=$(commit_tree "$(git write-tree)" "$parent" "2026-09-03T12:48:00-07:00" "readme cuda tables + docs")
git reset --hard "$parent"
rm -f _rebuild_history.sh pointcloud-pipeline_context.md

echo "==== LOG ===="
git log --reverse --format='%h | %ai | %s'
echo "count=$(git rev-list --count HEAD)"
if git ls-files | grep -q 'pointcloud-pipeline_context.md'; then
  echo "ERROR: context file still tracked"
  exit 1
else
  echo "OK: context file not tracked"
fi
