
# The construct ROS course

This repository contains the projects I realized following some of the ROS courses offered by The Construct.

# Docker

I have created a docker file aimed to create an image with most of the required ros packages.
It uses `nvidia-docker`.

## How to build the image
To create this image run `docker/build_image.sh`.


## How to run the container
The script `docker/container-start.bash` starts an interactive bash in the container with the root user.
This script mounts a volume with this repository at `/catkinws`.
