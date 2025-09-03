#// vectornav_node/README.md
# VectorNav ROS 2 Node

This package provides a ROS 2 wrapper for the VectorNav Python API. It connects to a VectorNav sensor, retrieves sensor data, and publishes it as standard ROS 2 messages.

## Prerequisites

1.  **ROS 2 Humble**: This package is designed for ROS 2 Humble.
    ```bash
    # Follow official installation instructions for ROS 2 Humble
    ```

2.  **VectorNav Python API**: The `vectornav` Python library must be installed. The source files for this library were provided. Navigate to the `python_documentation` directory and install it using pip.

    ```bash
    # Navigate to the directory containing setup.py for the vectornav library
    cd /path/to/your/python_documentation
    
    # Install the library
    pip install .
    ```

3.  **Required ROS 2 Packages**: Ensure you have the common ROS 2 message packages.
    ```bash
    sudo apt-get update
    sudo apt-get install ros-humble-sensor-msgs ros-humble-geometry-msgs ros-humble-nav-msgs ros-humble-diagnostic-msgs
    ```

## Building the Package

1.  Create a ROS 2 workspace (e.g., `vectornav_ws`).
    ```bash
    mkdir -p ~/vectornav_ws/src
    ```

2.  Clone or copy this `vectornav_node` package into the `src` directory.

3.  Build the workspace.
    ```bash
    cd ~/vectornav_ws
    colcon build
    ```

## Configuration

The node's parameters can be configured in the `config/params.yaml` file.

-   **`port`**: The serial port your VectorNav sensor is connected to (e.g., `/dev/ttyUSB0`).
-   **`baudrate`**: The baud rate for the serial connection (e.g., `115200`).
-   **`frame_id`**: The `frame_id` for the sensor's body frame (e.g., `vectornav`).
-   **`global_frame_id`**: The global reference frame. Can be set to `"NED"` (North-East-Down) or `"ENU"` (East-North-Up). If set to `"ENU"`, the node will automatically convert the sensor's NED data to the ENU frame, which is standard for ROS. The frame ID for global topics (like `/odom`) will be constructed from this (e.g., `vectornav_enu`).
-   **`output_rate`**: The desired data output frequency in Hz. The node will calculate the nearest possible `rate_divisor` based on the sensor's internal IMU rate (assumed to be 400Hz). For example, an `output_rate` of 100 will result in a divisor of 4.
-   **`use_binary_output`**: Set to `true` to configure the sensor for high-rate binary output, which is recommended. If `false`, the node will rely on the sensor's pre-configured asynchronous output.

## Running the Node

1.  Source your workspace setup file.
    ```bash
    source ~/vectornav_ws/install/setup.bash
    ```

2.  Launch the node using the provided launch file. This will also load the parameters from `config/params.yaml`.
    ```bash
    ros2 launch vectornav_node vectornav.launch.py
    ```

3.  (Optional) To run with custom parameters:
    ```bash
    ros2 run vectornav_node driver --ros-args -p port:="/dev/ttyUSB1" -p baudrate:=921600
    ```

## Published Topics

The node publishes the following topics with standard ROS 2 message types:

-   **/imu/data** (`sensor_msgs/Imu`): IMU data including orientation, angular velocity, and linear acceleration.
-   **/imu/mag** (`sensor_msgs/MagneticField`): Magnetic field data.
-   **/imu/temp** (`sensor_msgs/Temperature`): Sensor temperature.
-   **/imu/pres** (`sensor_msgs/FluidPressure`): Barometric pressure.
-   **/gps/fix** (`sensor_msgs/NavSatFix`): GNSS position fix.
-   **/gps/vel** (`geometry_msgs/TwistStamped`): GNSS velocity in the configured global frame (NED or ENU).
-   **/odom** (`nav_msgs/Odometry`): Odometry data with orientation, angular velocity, and linear velocity in the configured global frame. Position is set to zero.
-   **/attitude/quat** (`geometry_msgs/QuaternionStamped`): Attitude as a quaternion, relative to the configured global frame.
-   **/attitude/ypr** (`geometry_msgs/Vector3Stamped`): Attitude as Yaw, Pitch, Roll in radians, relative to the configured global frame.
-   **/gnss/compass_heading** (`std_msgs/Float64`): GNSS-aided compass heading (yaw) in radians, relative to the configured global frame.
-   **/diagnostics** (`diagnostic_msgs/DiagnosticArray`): Detailed sensor status for IMU, INS, etc.