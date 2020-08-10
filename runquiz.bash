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
        find . -type d -name "build" -exec rm -rf "{}" \;
        find . -type d -name ".catkin_workspace" -exec rm -rf "{}" \;
        exit 1
    fi

    if [ ! -d "quizzes/$1/devel" ]; then
        cd "quizzes/$1" && catkin_make
    fi
    roslaunch "$2" "$3"
}

function print_help(){
    
    cat <<EOF
Usage: $0 [options]
    $0 unit package roslaunchfile
EOF
}

if [ "$#" -eq 0 ]
then
    print_help
    exit 1
fi

main "$@"
