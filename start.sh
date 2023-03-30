#!/usr/bin/bash

# Change name also in stop.sh
TMUX_NAME=gaze-control-tmux
DOCKER_CONTAINER_NAME=ergocub_gaze_control_container

# Start the container with the right options
docker run --gpus=all -v "$(pwd)":/home/ecub -itd --rm \
--gpus=all \
--env DISPLAY=:0 \
--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
--ipc=host \
--network=host --name $DOCKER_CONTAINER_NAME ar0s/ergocub-manipulation bash

# Create tmux session
tmux new-session -d -s $TMUX_NAME
tmux set-option -t $TMUX_NAME status-left-length 140
tmux set -t $TMUX_NAME -g pane-border-status top
tmux set -t $TMUX_NAME -g mouse on

tmux rename-window -t $TMUX_NAME gaze-control

# Human Detection
tmux select-pane -T "Human Detection"
tmux send-keys -t $TMUX_NAME "docker exec -it $DOCKER_CONTAINER_NAME bash" Enter
tmux send-keys -t $TMUX_NAME "vscode"


tmux a -t $TMUX_NAME
