#!/usr/bin/env vash

myuid=$(id -u $USER)
mygid=$(id -g $USER)
mygroup=$(id -g -n $USER)

docker build -t "rosignateimage" \
    --build-arg myuser="$USER" \
    --build-arg myuid="$myuid" \
    --build-arg mygroup="$mygroup" \
    --build-arg mygid="$mygid" \
      -f ./rosimage.dockerfile .

exit 0

