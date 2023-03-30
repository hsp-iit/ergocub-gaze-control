#!/bin/bash

TMUX_NAME=gaze-control-tmux
DOCKER_IMAGE_NAME=ergocub_gaze_control_container

docker rm -f $DOCKER_IMAGE_NAME
tmux kill-session -t $TMUX_NAME
