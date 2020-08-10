#!/bin/bash
# Bash script which launches the rosignite $1
# Arguments
# ---------
# $2 = unit
# $3 = package
# $4 = launch file
#let script exit if a command fails
set -o errexit 
#let script exit if an unsed variable is used
set -o nounset



function main(){
    if [ "$2" = "clean" ]; then
        find . -type d -name "devel" -exec rm -rf "{}" \;
        find . -type d -name "build" -exec rm -rf "{}" \;
        find . -type d -name ".catkin_workspace" -exec rm -rf "{}" \;
        exit 1
    fi

    if is_built $1 $2; then
        roslaunch "$3" "$4"
    else
        build_package $1 $2
        echo "Now you can run roslaunch $3 $4"
    fi
}


function build_package(){
    cd "$1/$2" && catkin_make
}

function is_built(){
     [[ -d "$1/$2/devel" && -d "$1/$2/build" ]]
     return $?
}

function print_help(){
    
    cat <<EOF
Usage: $0 [quizzes, examples] [unit] [package] [roslaunch]
EOF
}

if [ "$#" -lt 4 ]
then
    print_help
    exit 1
fi

main "$@"
