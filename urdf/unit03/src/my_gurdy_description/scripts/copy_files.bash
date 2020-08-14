#!/bin/bash

function main(){
    source_path="/home/simulations/public_sim_ws/src/all/gurdy/gurdy_description/models/gurdy/meshes/"

    if [ -d $source_path ] ; then
        current_path="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
        target_path="$current_path/../models/gurdy/meshes/"
        cp $source_path/* $target_path/
    else
        echo "ERROR the folder $source_path does not exists"
    fi
}

main

exit 0
