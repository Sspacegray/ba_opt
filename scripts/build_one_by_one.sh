#!/usr/bin/env z sh
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${ZSH_SOURCE[0]}")/.." && pwd)"

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.zsh" ]]; then
  # Use the user's ROS_DISTRO if set.
  source "/opt/ros/${ROS_DISTRO}/setup.zsh"
elif [[ -f /opt/ros/humble/setup.zsh ]]; then
  source /opt/ros/humble/setup.zsh
elif [[ -f /opt/ros/jazzy/setup.zsh ]]; then
  source /opt/ros/jazzy/setup.zsh
else
  echo "ERROR: ROS setup.zsh not found in /opt/ros. Set ROS_DISTRO or install ROS." >&2
  exit 1
fi

# Configurable paths and knobs.
BUILD_BASE="${BUILD_BASE:-${ROOT_DIR}/build}"
INSTALL_BASE="${INSTALL_BASE:-${ROOT_DIR}/install}"
LOG_DIR="${LOG_DIR:-${ROOT_DIR}/build_logs}"
PARALLEL_WORKERS="${PARALLEL_WORKERS:-1}"
CMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE:-Release}"

# Cleanup behavior:
#   CLEAN_ALL=1       -> remove build/install/log before starting
#   CLEAN_BUILD_DIRS=1 -> remove build/<pkg> and log/<pkg> after each package
CLEAN_ALL="${CLEAN_ALL:-0}"
CLEAN_BUILD_DIRS="${CLEAN_BUILD_DIRS:-0}"
STOP_ON_FAIL="${STOP_ON_FAIL:-1}"

mkdir -p "${LOG_DIR}"

if [[ "${CLEAN_ALL}" == "1" ]]; then
  rm -rf "${BUILD_BASE}" "${INSTALL_BASE}" "${ROOT_DIR}/log"
fi

# Force single-threaded builds unless overridden.
export CMAKE_BUILD_PARALLEL_LEVEL="${CMAKE_BUILD_PARALLEL_LEVEL:-1}"
export MAKEFLAGS="${MAKEFLAGS:--j1}"

pkg_list_file="${LOG_DIR}/package_list.txt"
colcon list --names-only > "${pkg_list_file}"
: > "${LOG_DIR}/build_summary.txt"

CMAKE_ARGS=(
  -DCMAKE_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
  -DBUILD_TESTING=OFF
  -DBUILD_EXAMPLES=OFF
  -DGTSAM_BUILD_TESTS=OFF
  -DGTSAM_BUILD_EXAMPLES=OFF
  -DGTSAM_BUILD_UNSTABLE=OFF
  -DBUILD_SOPHUS_TESTS=OFF
  -DBUILD_SOPHUS_EXAMPLES=OFF
)

while IFS= read -r pkg; do
  echo "==> Building ${pkg}" | tee -a "${LOG_DIR}/build_summary.txt"
  log_file="${LOG_DIR}/${pkg}.log"

  if colcon build \
      --packages-select "${pkg}" \
      --build-base "${BUILD_BASE}" \
      --install-base "${INSTALL_BASE}" \
      --executor sequential \
      --parallel-workers "${PARALLEL_WORKERS}" \
      --cmake-clean-cache \
      --cmake-args "${CMAKE_ARGS[@]}" \
      >"${log_file}" 2>&1; then
    echo "OK: ${pkg}" | tee -a "${LOG_DIR}/build_summary.txt"
  else
    echo "FAIL: ${pkg}" | tee -a "${LOG_DIR}/build_summary.txt"
    if [[ "${STOP_ON_FAIL}" == "1" ]]; then
      break
    fi
  fi

  if [[ "${CLEAN_BUILD_DIRS}" == "1" ]]; then
    rm -rf "${BUILD_BASE}/${pkg}" "${ROOT_DIR}/log/${pkg}"
  fi
  echo | tee -a "${LOG_DIR}/build_summary.txt"
done < "${pkg_list_file}"

echo "Done. Summary in ${LOG_DIR}/build_summary.txt"
