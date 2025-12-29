#!/bin/bash

DOCKERFILE=Dockerfile
IMAGE_NAME=crazysim_img_student

echo "Building $IMAGE_NAME image."
echo "Dockerfile: $DOCKERFILE"

export DOCKER_BUILDKIT=1
docker build \
    $build_args \
    -f $DOCKERFILE \
    --ssh default \
    -t $IMAGE_NAME .