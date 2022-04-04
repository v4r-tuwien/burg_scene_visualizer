# BURG Scene Visualizer

This is a ROS package for visualizing scenes from the [BURG SetupTool](https://github.com/markus-suchi/burg-setuptool).
The printed marker templates will be recognized and the objects are projected
into the image to facilitate placing the real objects.

### installation

You need to install the [BURG-Toolkit](https://github.com/mrudorfer/burg-toolkit) so that it is found by the Python 
that ROS uses. This is usually the system python.
Then you can simply clone this repository into your catkin workspace and install 
using `catkin_make`.

### usage

The package contains:
- `nodes/visualize_scene.py` The actual node with the SceneVisualizer.
- `launch/visualize.launch` A launch file for exemplary setup with a realsense or kinect2 camera,
    including visualization in rviz.

In any case, you will have to provide at least the `scene_fn` argument, which points to
a scene.yaml file that was created with
[BURG SetupTool](https://github.com/markus-suchi/burg-setuptool)
or 
[BURG-Toolkit](https://github.com/mrudorfer/burg-toolkit).
Refer to those repos for more details on scene creation.