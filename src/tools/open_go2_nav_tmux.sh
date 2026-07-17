#!/usr/bin/env bash

# Open a 2-column x 4-row tmux workspace.  The four panes in the left
# column enter the robot Docker container and receive commands without
# pressing Enter; the four panes in the right column stay on the host.

set -euo pipefail

SESSION_NAME="${SESSION_NAME:-go2_nav}"
CONTAINER_ID="${CONTAINER_ID:-f3b82610c6d7}"
STARTUP_WAIT="${STARTUP_WAIT:-1}"

LEFT_TITLES=(livox pct_scan goal_points enable_go2)
LEFT_COMMANDS=(
  "ros2 launch livox_ros_driver2 msg_MID360s_launch.py"
  "ros2 launch pct_scan_navigation unitree_go2w_pct_scan_navigation.launch.py"
  "python3 work_space/kn_nav_ws/src/tools/goal_points_cli.py"
  "ros2 service call /go2_cmd_vel_bridge/enable std_srvs/srv/SetBool '{data: true}'"
)

if ! command -v tmux >/dev/null 2>&1; then
  echo "tmux is not installed. Install it with: sudo apt install tmux" >&2
  exit 1
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "docker command was not found." >&2
  exit 1
fi

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
  echo "Attaching to existing tmux session: ${SESSION_NAME}"
  exec tmux attach-session -t "${SESSION_NAME}"
fi

tmux new-session -d -s "${SESSION_NAME}" -n navigation
left_0="$(tmux display-message -p -t "${SESSION_NAME}:navigation.0" '#{pane_id}')"

# First make two equal-width columns.
right_0="$(tmux split-window -h -p 50 -t "${left_0}" -P -F '#{pane_id}')"

# Split one full-height column into four nearly equal-height panes.
split_column() {
  local top="$1"
  local second third fourth
  second="$(tmux split-window -v -p 75 -t "${top}" -P -F '#{pane_id}')"
  third="$(tmux split-window -v -p 67 -t "${second}" -P -F '#{pane_id}')"
  fourth="$(tmux split-window -v -p 50 -t "${third}" -P -F '#{pane_id}')"
  printf '%s %s %s %s\n' "${top}" "${second}" "${third}" "${fourth}"
}

read -r -a left_panes <<< "$(split_column "${left_0}")"
read -r -a right_panes <<< "$(split_column "${right_0}")"

tmux set-option -t "${SESSION_NAME}" mouse on
tmux set-option -t "${SESSION_NAME}" pane-border-status top
tmux set-option -t "${SESSION_NAME}" pane-border-format \
  ' #[fg=cyan]#{pane_title} #[default]'

for index in "${!left_panes[@]}"; do
  tmux select-pane -t "${left_panes[index]}" -T "${LEFT_TITLES[index]}"
  tmux select-pane -t "${right_panes[index]}" -T "host_$((index + 1))"

  # Enter the container first.  C-m is Enter for this command only.
  tmux send-keys -t "${left_panes[index]}" \
    "docker exec -it ${CONTAINER_ID} bash" C-m
done

# Give Docker time to present the container shell, then insert the four
# commands literally.  No C-m is sent here, so none of them is executed.
sleep "${STARTUP_WAIT}"
for index in "${!left_panes[@]}"; do
  tmux send-keys -l -t "${left_panes[index]}" "${LEFT_COMMANDS[index]}"
done

tmux select-pane -t "${left_panes[0]}"
exec tmux attach-session -t "${SESSION_NAME}"
