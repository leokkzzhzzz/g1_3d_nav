#!/bin/bash
set -e

IMAGE="hongtu-fastlio2:noetic"
WS="${WS:-$HOME/g1_3d_nav}"

if [ ! -d "$WS/HongTu" ]; then
    echo "Cloning HongTu into $WS..."
    mkdir -p "$WS"
    git clone https://github.com/yuanqizhiti/HongTu.git "$WS/HongTu"
fi

mkdir -p "$WS/maps"

docker run -it --rm \
    --network host \
    --name hongtu_mapper \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e MID360_IP="${MID360_IP:-192.168.123.120}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
    -v "$WS:/root/g1_3d_nav" \
    "$IMAGE" \
    "$@"
