#!/bin/bash

function main(){
    source_path='/home/simulations/public_sim_ws/src/all/gurdy/gurdy_description/models/gurdy/meshes/'
    current_path="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
    target_path="$current_path/../models/gurdy/meshes/"
    cp $source_path/* $target_path/
}

main

exit 0
