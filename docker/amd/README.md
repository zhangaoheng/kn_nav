docker buildx build \
  --network=host \
  --platform=linux/amd64 \
  --load \
  --build-arg GIT_REFRESH="$(date -u +%Y%m%d%H%M%S)" \
  -f work_space/kn_nav_ws/docker/Dockerfile \
  -t cross-floor-nav:amd-v1 .


