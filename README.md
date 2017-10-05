# experiment-new-grasp
Collect experiments on our new grasping demo

## experiment-1
The module `experiment-1` implemts a wrapper code for performing the superquadric and grasping pipeline described in [1].
Thiw wrapper code communicates with existing modules developed in [`robotology` repo](https://github.com/robotology) and its structure can be summarized as follow:






1) The wrapper code asks the [object property collector](http://wiki.icub.org/brain/group__objectsPropertiesCollector.html) for the bounding box information of the object.
2) Given that, [lbpExtract module](https://github.com/robotology/segmentation) provides the 2D blob of the object.
3) The wrapper code sends the 2D blob of the object to the [Structure From Motion module](https://github.com/robotology/stereo-vision)for getting the relative 3D point cloud.
4) The 3D point cloud is then sent to the [superquadric-mode](https://github.com/robotology/superquadric-model) for computing the superquadric modeling the object.
5) The wrapper code sends the estimated superquadric to the [superquadric-grasp module](https://github.com/robotology/superquadric-grasp), which computes suitable poses.
6) Finally, the superquadric-grasp is asked to perform the grasping task.

[1] G. Vezzani, U. Pattacini and L. Natale, "A grasping approach based on superquadric models", IEEE-RAS International Conference on Robotics and Automation 2017, pp 1579-1586.
