import threading
import queue
import time
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, QuaternionStamped
from std_msgs.msg import Float32, Time

from vectornav import Sensor, Registers, VnTypes, Plugins, VnError

# ==============================================================================
# The Configuration Class
# ==============================================================================
@dataclass
class VectorNavConfig:
    """A simple data container for all vectornav-related settings."""
    port: str
    baudrate: int
    output_rate: int
    gnss_antenna_offset: List[float]
    gnss_compass_baseline: List[float]
    frame_id: str

# ==============================================================================
# The VectorNav Sensor Hardware Class
# ==============================================================================
class VectorNavSensor:
    def __init__(self, config: VectorNavConfig, logger):
        self.config = config
        self.logger = logger
    
    def _baud_rate_table(self, baudrate: int) -> Sensor.BaudRate:
        """
        Converts an integer baud rate to its corresponding Sensor.BaudRate enum member.
        This function assumes Sensor.BaudRate is an Enum imported from a library.
        Args:
            baudrate: The integer value of the baud rate (e.g., 115200).
        Returns:
            The matching Sensor.BaudRate enum member.
        Raises:
            ValueError: If the provided baud rate is not supported by the Enum.
        """
        try:
            # Call the Enum with the integer value to get the corresponding member. The Enum itself is a lookup table.
            return Sensor.BaudRate(baudrate)
        except ValueError:
            # This block runs if the integer baudrate does not match any enum member's value.
            supported_rates = ", ".join(str(b.value) for b in Sensor.BaudRate)
            raise ValueError(
                f"Unsupported baud rate: {baudrate}. "
                f"Supported rates are: {supported_rates}"
            )
    
    def connect_to_sensor(self):
        """ Connects to the VectorNav sensor. """
        self.vs = Sensor()
        self.logger().info(f"Connecting to sensor at {self.config.port} with baudrate {self.baudrate}...")
        try:
            self.vs.connect(self.config.port, self._baud_rate_table(self.config.baudrate))
            if not self.vs.verifySensorConnectivity():
                self.logger().warning("Sensor connectivity check failed. Attempting to autoconnect.")
                self.vs.autoconnect(self.config.port)
            connected_port = self.vs.connectedPortName()
            connected_baud = self.vs.connectedBaudRate()
            self.logger().info(f"Successfully connected to the VectorNav sensor at port: {connected_port} with baud rate: {connected_baud}.")
        except Exception as e:
            self.logger().fatal(f"Could not connect to the sensor: {e}")
    
    def configure_sensor(self):
        """Write to Configuration Registers"""
        # Configure gnss baselines
        gnss_antenna_offset = Registers.GNSS.GnssAOffset()
        gnss_antenna_offset.positionX = self.config.gnss_antenna_offset[0]
        gnss_antenna_offset.positionY = self.config.gnss_antenna_offset[1]
        gnss_antenna_offset.positionZ = self.config.gnss_antenna_offset[2]
        self.vs.writeRegister(gnss_antenna_offset)

        gnss_compass_baseline = Registers.GNSSCompass.GnssCompassBaseline()
        gnss_compass_baseline.positionX = self.config.gnss_compass_baseline[0]
        gnss_compass_baseline.positionY = self.config.gnss_compass_baseline[1]
        gnss_compass_baseline.positionZ = self.config.gnss_compass_baseline[2]
        gnss_compass_baseline.uncertaintyX = self.config.gnss_compass_baseline[3]
        gnss_compass_baseline.uncertaintyY = self.config.gnss_compass_baseline[4]
        gnss_compass_baseline.uncertaintyZ = self.config.gnss_compass_baseline[5]
        self.vs.writeRegister(gnss_compass_baseline)

        # Turn off Async Ascii Output
        asyncDataOutputType = Registers.AsyncOutputType()
        asyncDataOutputType.ador = Registers.AsyncOutputType.Ador.OFF
        self.vs.writeRegister(asyncDataOutputType)
        
        # Turn on Async Binary Output
        self.binaryOutput1Register = Registers.BinaryOutput1()
        self.binaryOutput1Register.rateDivisor = int(400 / self.config.output_rate)
        self.binaryOutput1Register.asyncMode.serial1 = 1
        self.binaryOutput1Register.asyncMode.serial2 = 1

        # Enable specific register output in composite data
        # --- Time ---
        self.binaryOutput1Register.time.timeStartup = 1          # [vnpy.Time] The system time since startup measured in nano seconds
        # --- IMU ---
        self.binaryOutput1Register.imu.imuStatus = 1             # [vnpy.ImuStatus]
        self.binaryOutput1Register.imu.temperature = 1           # [float]
        self.binaryOutput1Register.imu.pressure = 1              # [float]
        self.binaryOutput1Register.imu.mag = 1                   # [vnpy.Vec3f]
        self.binaryOutput1Register.imu.accel = 1                 # [vnpy.Vec3f]
        self.binaryOutput1Register.imu.angularRate = 1           # [vnpy.Vec3f]
        # --- GNSS ---
        self.binaryOutput1Register.gnss.gnss1TimeUtc = 1         # [vnpy.GnssTimeUtc] The current UTC time
        self.binaryOutput1Register.gnss.gnss1NumSats = 1         # [uint8_t] The number of tracked GNSS satellites
        self.binaryOutput1Register.gnss.gnss1Fix = 1             # [uint8_t] The current GNSS fix type
        self.binaryOutput1Register.gnss.gnss1PosLla = 1          # [vnpy.Lla] The current GNSS position measurement given as the geodetic latitude, longitude and altitude
        self.binaryOutput1Register.gnss.gnss1PosUncertainty = 1  # [vnpy.Vec3f] The current GNSS position uncertainty in NED
        self.binaryOutput1Register.gnss.gnss1VelNed = 1          # [vnpy.Vec3f] The current GNSS velocity in the North East Down (NED) reference frame
        self.binaryOutput1Register.gnss.gnss1VelUncertainty = 1  # [float] The current GNSS velocity uncertainty
        self.binaryOutput1Register.gnss.gnss1Dop = 1             # [vnpy.GnssDop] Dilution of precision
        self.binaryOutput1Register.gnss.gnss1SatInfo = 1         # [vnpy.GnssSatInfo] Information and measurements pertaining to each GNSS satellite in view
        # --- Attitude ---
        self.binaryOutput1Register.attitude.quaternion = 1       # [vnpy.Quat] NED frame
        self.binaryOutput1Register.attitude.attU = 1             # [float] The estimated uncertainty (1 Sigma) in the current attitude estimate.
        # --- INS ---
        self.binaryOutput1Register.ins.insStatus = 1             # [vnpy.InsStatus]
        self.binaryOutput1Register.ins.posLla = 1                # [vnpy.Lla] The estimated position
        self.binaryOutput1Register.ins.velBody = 1               # [vnpy.Vec3f] The estimated velocity in the body-frame
        self.binaryOutput1Register.ins.velNed = 1                # [vnpy.Vec3f] The estimated velocity in the NED-frame
        self.binaryOutput1Register.ins.posU = 1                  # [float] The estimated uncertainty (1 Sigma) in the current position estimate
        self.binaryOutput1Register.ins.velU = 1                  # [float] The estimated uncertainty (1 Sigma) in the current velocity estimate

        # Write and reset
        self.vs.writeRegister(self.binaryOutput1Register)
        self.vs.reset()
        self.logger.info("Configured Binary Outputs")
    
# ==============================================================================
# The ROS 2 Node Class
# ==============================================================================
class VectorNavNode(Node):
    def __init__(self):
        super().__init__('vectornav_node')
        # --- Parameters ---
        self.declare_parameters(
            # namespace='vectornav_config',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', '115200'),
                ('output_rate', '20'),
                ('gnss_antenna_offset', [1.0, 0.0, 0.0]),
                ('gnss_compass_baseline', [1.0, 0.0, 0.0, 0.0256, 0.0256, 0.0256])
                ('frame_id', 'vn300')
            ]
        )
        self.vn_config = VectorNavConfig()
        self.vn_config.port = self.get_parameter('port').value()
        self.vn_config.baudrate = self.get_parameter('baudrate').value()
        self.vn_config.output_rate = self.get_parameter('output_rate').value()
        self.vn_config.gnss_antenna_offset = self.get_parameter('gnss_antenna_offset').value()
        self.vn_config.gnss_compass_baseline = self.get_parameter('gnss_compass_baseline').value()
        self.vn_config.frame_id = self.get_parameter('frame_id')

        # --- Sensor Class ---
        self.vn_sensor = VectorNavSensor(config=self.vn_config, logger=self.get_logger())
        self.vn_sensor.connect_to_sensor()
        self.vn_sensor.configure_sensor()

        # --- Publisher ---
        self.setup_publishers()

    def setup_publishers(self):
        self.startup_time_publisher = self.create_publisher(Time, "vectornav/startup_time", 10)
        self.imu_publisher = self.create_publisher(Imu, "vectornav/imu_raw", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "vectornav/mag", 10)
        self.temperature_publisher = self.create_publisher(Temperature, "vectornav/temperature", 10)
        self.pressure_publisher = self.create_publisher(FluidPressure, "vectornav/pressure", 10)
        self.gps_fix_publisher = self.create_publisher(NavSatFix, 'vectornav/gps/fix', 10)
        self.ins_fix_publisher = self.create_publisher(NavSatFix, 'vectornav/ins/fix', 10)
        self.ins_vel_body_publisher = self.create_publisher(TwistWithCovarianceStamped, 'vectornav/ins/vel_body', 10)
        self.ins_vel_enu_publisher = self.create_publisher(TwistWithCovarianceStamped, 'vectornav/ins/vel_enu', 10)

    def packet_handling(self):
        compositeData = self.vn_sensor.getNextMeasurement()
        if compositeData.matchesMessage(self.binaryOutput1Register):
            system_timestamp = self.get_clock().now()
            ros_timestamp = system_timestamp.to_msg()

            # -- Time --
            startup_time = Time()
            startup_time.data = compositeData.time.timeStartup.nanoseconds()
            self.startup_time_publisher.publish(startup_time)
            # -- IMU --
            imu_msg = Imu()
            imu_msg.header.stamp = ros_timestamp
            imu_msg.header.frame_id = self.vn_config.frame_id
            imu_msg.angular_velocity.x = compositeData.imu.angularRate[0]
            imu_msg.angular_velocity.y = compositeData.imu.angularRate[1]
            imu_msg.angular_velocity.z = compositeData.imu.angularRate[2]
            imu_msg.linear_acceleration.x = compositeData.imu.Accel[0]
            imu_msg.linear_acceleration.y = compositeData.imu.Accel[1]
            imu_msg.linear_acceleration.z = compositeData.imu.Accel[2]
            self.imu_publisher.publish(imu_msg)