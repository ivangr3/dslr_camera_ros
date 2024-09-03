## DSLR camera with ROS
This package allows the easy connection of DSLR cameras with ROS. The package is based on the [libgphoto2](http://www.gphoto.org/) library. Check compatible cameras [here](http://www.gphoto.org/doc/remote/).

# Dependencies
```bash
sudo apt install libgphoto2-6
```

# Usage
1. Launch the node for the connection
```bash
roslaunch dslr_camera_ros dslr_camera_ros.launch
```

2. Visualize camera preview published in _/<node_namespace>/preview_

3. Take a picture by calling the node service
```bash
rosservice call /<node_namespace>/capture "path: '/<path_to_save>/<img_name>.jpg'"
```

# Future work
- Improve previsualization
- Provide zoom adjustment

