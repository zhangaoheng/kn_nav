#!/usr/bin/env bash
set -euo pipefail

IMAGE="${1:-cross-floor-nav:amd-v1}"
CONTAINER_NAME="${CONTAINER_NAME:-kn_nav_humble}"
NAV_MAP_DIR="${NAV_MAP_DIR:-/home/unitree/nav_map}"
KN_NAV_DIR="${KN_NAV_DIR:-/home/code/work_space/kn_nav}"

BOOTSTRAP_CONTAINER=""

cleanup_bootstrap_container() {
  if [[ -n "${BOOTSTRAP_CONTAINER}" ]]; then
    docker rm "${BOOTSTRAP_CONTAINER}" >/dev/null 2>&1 || true
  fi
}
trap cleanup_bootstrap_container EXIT

bootstrap_kn_nav_workspace() {
  if [[ -d "${KN_NAV_DIR}/src" ]]; then
    return
  fi

  if [[ -e "${KN_NAV_DIR}" && ! -d "${KN_NAV_DIR}" ]]; then
    echo "KN_NAV_DIR is not a directory: ${KN_NAV_DIR}" >&2
    exit 1
  fi

  if [[ -d "${KN_NAV_DIR}" ]] &&
     [[ -n "$(find "${KN_NAV_DIR}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    echo "KN navigation workspace is incomplete: ${KN_NAV_DIR}" >&2
    echo "The directory is not empty, but ${KN_NAV_DIR}/src does not exist." >&2
    echo "It will not be overwritten automatically. Check it or set KN_NAV_DIR." >&2
    exit 1
  fi

  if ! mkdir -p "${KN_NAV_DIR}"; then
    echo "Cannot create the host workspace: ${KN_NAV_DIR}" >&2
    echo "Run these commands once, then run this script again:" >&2
    echo "  sudo mkdir -p '${KN_NAV_DIR}'" >&2
    echo "  sudo chown -R '$(id -u):$(id -g)' '${KN_NAV_DIR}'" >&2
    exit 1
  fi

  if ! docker image inspect "${IMAGE}" >/dev/null 2>&1; then
    echo "Docker image was not found: ${IMAGE}" >&2
    exit 1
  fi

  echo "Initializing ${KN_NAV_DIR} from Docker image ${IMAGE} ..."
  BOOTSTRAP_CONTAINER="$(docker create "${IMAGE}")"
  if ! docker cp \
    "${BOOTSTRAP_CONTAINER}:/home/code/work_space/kn_nav/." \
    "${KN_NAV_DIR}/"; then
    echo "Failed to copy the KN navigation workspace from image ${IMAGE}." >&2
    echo "Check the partially initialized directory: ${KN_NAV_DIR}" >&2
    exit 1
  fi
  docker rm "${BOOTSTRAP_CONTAINER}" >/dev/null
  BOOTSTRAP_CONTAINER=""

  if [[ ! -d "${KN_NAV_DIR}/src" ]]; then
    echo "The image does not contain /home/code/work_space/kn_nav/src: ${IMAGE}" >&2
    exit 1
  fi

  echo "KN navigation workspace initialized: ${KN_NAV_DIR}"
}

bootstrap_kn_nav_workspace

docker run -d \
  --name "${CONTAINER_NAME}" \
  --restart unless-stopped \
  --network host \
  --ipc host \
  --privileged \
  -v "${NAV_MAP_DIR}:/home/nav_map:ro" \
  -v "${KN_NAV_DIR}:/home/code/work_space/kn_nav" \
  "${IMAGE}"

echo "Container started: ${CONTAINER_NAME}"
echo "KN navigation workspace: ${KN_NAV_DIR} -> /home/code/work_space/kn_nav"
echo "Enter it with: docker exec -it ${CONTAINER_NAME} bash"
