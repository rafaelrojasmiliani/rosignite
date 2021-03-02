
1. Basic use of `cmvision` [available here](http://wiki.ros.org/cmvision) tools
    - Use of `rosrun cmvision colorgui image:=<image topic>`. This starts a node that  bring up an interface that provides a means for graphically selecting a blob by automatically computing the man RGB and the YUV range of regions in a picture where the user clicks with the mouse.. 
    - Defining the YUV range of of the desired blobs and assigning name to blobs using a file with the `cmvision` format.
    - Basic use of `rosrun cmvision cmvision image:=<image topic>`. This starts a node that broadcasts the blobs detected in the image and their position in the image
