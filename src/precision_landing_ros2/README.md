# Precision Lander Node

## Overview

The Precision Lander Node is a ROS2 package designed for the automated precise landing of drones using MAVROS. This package implements a state machine that manages the landing process, including searching for landing targets, centering over them, and executing a controlled descent.

## Features

- **State Machine**: Manages different states of the landing process, including initialization, target searching, centering, descending, and finalizing the landing.
- **PID Control**: Implements PID controllers for precise control of the drone's position during landing.
- **Coordinate Transformations**: Ensures that the drone's position and target position are in the same coordinate system.
- **Configurable Parameters**: Allows for dynamic adjustment of PID coefficients, maximum velocities, and other parameters through a YAML configuration file.

## Installation

To install the Precision Lander Node, follow these steps:

1. Clone the repository:
   ```
   git clone <repository-url>
   cd precision_landing_ros2
   ```

2. Install dependencies:
   ```
   rosdep install -i --from-path src --rosdistro <ros-distro> -y
   ```

3. Build the package:
   ```
   colcon build
   ```

4. Source the setup file:
   ```
   source install/setup.bash
   ```

## Usage

To launch the Precision Lander Node, use the provided launch file:

```
ros2 launch precision_landing precision_landing.launch.py
```

Ensure that the MAVROS node is running and properly configured to communicate with your drone.

## Testing

Unit tests and integration tests are provided to ensure the functionality of the Precision Lander Node. To run the tests, use the following command:

```
pytest
```

## Contributing

Contributions are welcome! Please submit a pull request or open an issue for any enhancements or bug fixes.

## License

This project is licensed under the MIT License. See the LICENSE file for more details.