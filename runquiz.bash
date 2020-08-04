#!/bin/bash
# Bash script which launches the rosignite quizzes
# Arguments
# ---------
# $1 = unit
# $2 = package
# $3 = launch file
#let script exit if a command fails
set -o errexit 
#let script exit if an unsed variable is used
set -o nounset


function main(){
    if [ "$1" = "clean" ]; then
        find . -type d -name "devel" -exec rm -rf "{}" \;
        exit
    fi

    if [ ! -d "quizzes/$1/devel" ]; then
        cd "quizzes/$1" && catkin_make
    fi
    source "quizzes/$1/devel/setup.bash"
    roslaunch "$2" "$3"
}

main "$@"
