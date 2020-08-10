#!/bin/bash
# Bash script which launches the rosignite $1
# Arguments
# ---------
# $1 = package
# $2 = launch file
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

    if is_built $1; then
        roslaunch "$1" "$2"
    else
        build_package $1
        echo "Now you can run roslaunch $1 $2"
        source $1/devel/setup.bash
    fi
}


function build_package(){
    cd "$1" && catkin_make
}

function is_built(){
     [[ -d "$1/devel" && -d "$1/build" ]]
     return $?
}

function print_help(){
    
    cat <<EOF
Usage: $0 [package] [roslaunch]
EOF
}

if [ "$#" -lt 2 ]
then
    print_help
    exit 1
fi

main "$@"
