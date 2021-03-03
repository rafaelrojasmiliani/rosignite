1. Covert `sensor_msgs.Image` into opencv images using ROS openCV bridge
2. Apply elementary filters to the image and extract features
    - Crop image by choosing the desired matrix slices of the image
    - Convert the image from RGB to HSV
    - Trunk the color distribution of the image. Make so that all pixels with a color within a given range became white and the others black. This is done with a bitwise and operation and a mask. This white part of the image is called blob.
    - Get the centroid of the blob

3. Compute a simplified visual servoing system in velocity able to make a turtlebot follow a colored line on the ground.
The control of the robot is done at the rotation of the robot along th vertical axis.
The dersired orientation of the robot is computed with the centroid of a blob in the image.
We build an angular positioning proportional control to center the centroid of the line, which is transformed into a blob.
The forward control velocity is constant.

4. Finally it has a small example of how to use [ROs PD control](http://wiki.ros.org/pid)
