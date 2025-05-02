# crx_description
the ROS2 desription package for Fanuc CRX- robots family

### Usage
To visualize the robot, install the repository, build and source the workspace. In the sourced terminal type

```console
ros2 launch crx_description view_robot.launch.py robot_type:=[crx5ia,crx10ia,crx10ia_l,crx20ia_l,crx25ia,crx30ia]
```

RViz and a GUI will let you visualize and move the virtual robot around, as shown in the image below.

<p align="center">
<img title="RViz vizualization" alt="Alt text" src="../doc/rviz_view.png" width="500" >
</p>



### Sources

The meshes of the CRX5iA, CRX25iA, and CRX30iA are extracted from RoboDK models.

The meshes for the CRX10iA-l and CRX20iA-l are extracted from GrabCAD.


### Considerations

This package assumes CRX10iA-L and CRX20iA-l have same dimensions with the only visible difference according to the datasheets provided by Fanuc regarding joint 6 limitations 380 deg for the CRX10iA-l and 450 deg for the CRX20iA-l, respectively.

This package also assumes CRX25iA and CRX30iA have same dimensions according to the datasheets provided by Fanuc.

### Development

The presented URDFs have been tested and verified on real robots for the CRX10iA-l, CRX20iA-l and CRX25iA only.

For the other modules please be careful. If you can provide us with any feedback, please let us know.

Please double check joint limit values if anything does not work as expected. Datasheets [here](https://www.fanuc.eu/eu-en/crx-series).
