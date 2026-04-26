# Multirobot Bringup

A ROS2 package for managing multi-robot bringup and coordination.

## Project Structure

This project follows the ROS2 standard package structure:

```
proyecto_abp/
├── src/
│   └── multirobot_bringup/          # Python package
│       ├── __init__.py              # Package initialization
│       └── *.py                     # Python modules
├── config/                          # Configuration files (YAML, etc.)
├── launch/                          # ROS2 launch files (.py, .xml)
├── rviz/                            # RViz configuration files (.rviz)
├── urdf/                            # Robot model files (.xacro, .urdf)
├── worlds/                          # Gazebo simulation world files (.sdf, .world)
├── resource/                        # Package resource files
├── test/                            # Test files
├── package.xml                      # ROS2 package metadata
├── setup.py                         # Python package setup
├── setup.cfg                        # Setup configuration
└── README.md                        # This file
```

## Building

To build this package in a colcon workspace:

```bash
colcon build --packages-select multirobot_bringup
```

## Running

Source the setup file and run launch files:

```bash
source install/setup.bash
ros2 launch multirobot_bringup <launch_file_name>
```

## Dependencies

This package requires:
- ROS2 (Humble or newer recommended)
- rclpy
- Standard ROS2 message types

See `package.xml` for complete dependency list.

## License

TODO: Add appropriate license

## Authors

- Aitor - Lead developer
- Juan - Contributor
