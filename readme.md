
# The construct ROS course

This repository contains the projects I realized following some of the ROS courses offered by The Construct.

# Structure

I have divided each course in "units", which are different from the units/chapters the course is formally divided*.
*Each unit correspond to a jupiter page displayed in the course.

The file structure of the repository is of the form `course/unit/src/ros_package`.
For each unit corresponds a catkin directory.

# Docker

I have created a docker file aimed to create an image with most of the required ros packages.
It uses `nvidia-docker`.

## How to build the image
To create this image run `docker/build_image.sh`.


## How to run the container
The script `docker/container-start.bash` starts an interactive bash in the container with the root user.
This script mounts a volume with this repository at `/catkinws`.
