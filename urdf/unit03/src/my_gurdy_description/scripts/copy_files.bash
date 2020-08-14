#!/bin/bash

function main(){
    # Copy the meshens from Gurdy into the current packet folder
    source_path='/home/simulations/public_sim_ws/src/all/gurdy/gurdy_description/models/gurdy/meshes/'
    if [ -d "$source_path" ]; then
        current_path="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
        target_path="$current_path/../models/gurdy/meshes/"
        cp "$source_path/*" "$target_path/"
        ech"o "Copying Gurdy meshes"
    else
        echo "Error: ${source_path} not found. Can not continue."
        exit 1
    fi
}

main

exit 0
