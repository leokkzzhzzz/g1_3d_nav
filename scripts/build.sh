#!/bin/bash
set -e
IMAGE="hongtu-fastlio2:noetic"
echo "Building ${IMAGE}..."
docker build -t "$IMAGE" "$(dirname "$0")"
echo "Done: ${IMAGE}"
