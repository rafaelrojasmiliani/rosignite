

# Basic used of [ORK object recognition](http://wiki.ros.org/object_recognition)

We use the [Table top object detector](http://wg-perception.github.io/object_recognition_core/).
The ORK package `object_recognition_core` provides an infrastructure for easy development and usage of object recognition pipelines. 
That is
- figure out a way to store/query/use your training data
- figure out a way to store/query/use your models
- provide metodologies to train custom object recognition (?)
- data source handling (cameras, ROS rosbag, ROS topics)
- integration with ROS

Object recognition has three main layers of complexity: data capture, training, detection. 

## In ORK everting is in a database (CouchDB)
In ORK everything is stored in a database (CouchDB): objects, models, training data.
**However, simple detectors like tabletop does not require the database**

- **How to add an object**
    1. Add name and descriptors
    ```
    rosrun object_recognition_core object_add.py -n coke -d "A universal can of coke"
    ```
    2. Add mess
    ```
    rosrun object_recognition_core mesh_add.py YOUR_OBJECT_ID `rospack find object_recognition_tutorials`/data/coke.obj --commit
    ```
- **Visualize objects in the database**
```
rosrun object_recognition_core mesh_add.py YOUR_OBJECT_ID `rospack find object_recognition_tutorials`/data/coke.obj --commit
```
- **Deleting an object**
```
rosrun object_recognition_core object_delete.py OBJECT_ID
```

## Table Top Detector

This object detection method has two parts: 
- **table finder** uses Random sample consensus (RANSAC) to find the dominant plane in a point-cloud
- **object recognizer** recognizes objects that are **rotationnally symmetric**. 
Points above the table are considered to belong to objects.
A clustering algorithm groups groups of points that possible are different objects in clusters.
For each cluster, a simple fitting technique is used against meshes in the database.
The current fitting method operates in 2D, since based on our assumptions (object resting upright on the known surface of the table) the other 4 dimensions are fixed.

**The sensor data that we use consists of a point cloud from the narrow stereo or Kinect cameras**.
To integrate with realsense [see here](https://github.com/robinloujun/object_recognition).
To integ
The sensor data that tabletop uses consists of a point cloud from the narrow stereo or Kinect cameras. tabletop performs the following steps:

        segmentation: the table is detected by finding the dominant plane in the point cloud based on analysis of 3D normal vectors; points above the table are considered to belong to graspable objects. We use a clustering algorithm to identify individual objects. We refer to the points corresponding to an individual object as clusters.
        recognition : for each cluster, a simple iterative fitting technique (a distant cousin of ICP) is used to see how well it corresponds to each mesh in our database of models. If a good fit is found, the database id of the model is returned along with the cluster; note that our current fitting method operates in 2D, since based on our assumptions (object resting upright on the known surface of the table) the other 4 dimensions are fixed.


## Manage tabletop's input parameters

`tabletop` manages a certain number of parameters, such as input image flow, database details, and detection thresholds, all of these parameters are defined in a configuration file, given as -c parameter in the tabletop command. The configuration file needs to be in YAML format. 

- **source** defines image topics that tabletop needs for its detection steps. 
Basically, tabletop needs a depth image topic, a color image topic, and the camera information messages for these images as input.

- **sink** defines how the output of tabletop can be processed further.

- **pipeline** takes care of detecting planar surfaces.


You can modify these parameters, such as input image topics, detection threshold, URI of the CouchDB to be used, etc. according to the setting you have in your project.
