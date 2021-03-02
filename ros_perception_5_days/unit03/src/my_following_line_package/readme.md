1. Covert `sensor_msgs.Image` into opencv images using ROS openCV bridge
2. Apply elementary filters to the image and extract features
    - Crop image by choosing the desired matrix slices of the image
    - Convert the image from RGB to HSV
    - Trunck the color distribution of the image. Make so that all pixels with a color within a given range became white and the others black. This is done with a bitwise and operation and a mask. This white part of the image is called blob.
    - Get the centroid of the blob
