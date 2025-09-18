import threading
import queue
import time
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3Stamped, TwistStamped
from std_msgs.msg import Float64

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
        self.logger.info("Disabled Async Ascii Output")
        
        # Turn on Async Binary Output
        binaryOutput1Register = Registers.BinaryOutput1()
        binaryOutput1Register.rateDivisor = int(400 / self.config.output_rate)
        binaryOutput1Register.asyncMode.serial1 = 1
        binaryOutput1Register.asyncMode.serial2 = 1

        # Enable specific register output in composite data
        # --- Time ---
        binaryOutput1Register.time.timeStartup = 1      # [vnpy.Time] The system time since startup measured in nano seconds
        # --- IMU ---
        binaryOutput1Register.imu.imuStatus = 1         # [vnpy.ImuStatus]
        binaryOutput1Register.imu.temperature = 1       # [float]
        binaryOutput1Register.imu.pressure = 1          # [float]
        binaryOutput1Register.imu.Mag = 1               # [vnpy.Vec3f]
        binaryOutput1Register.imu.Accel = 1             # [vnpy.Vec3f]
        binaryOutput1Register.imu.angularRate = 1       # [vnpy.Vec3f]
        # --- GNSS ---
        binaryOutput1Register.gnss.gnss1TimeUtc = 1         # [vnpy.GnssTimeUtc] The current UTC time
        binaryOutput1Register.gnss.gnss1NumSats = 1         # [uint8_t] The number of tracked GNSS satellites
        binaryOutput1Register.gnss.gnss1Fix = 1             # [uint8_t] The current GNSS fix type
        binaryOutput1Register.gnss.gnss1PosLla = 1          # [vnpy.Lla] The current GNSS position measurement given as the geodetic latitude, longitude and altitude
        binaryOutput1Register.gnss.gnss1PosUncertainty = 1  # [vnpy.Vec3f] The current GNSS position uncertainty in NED
        binaryOutput1Register.gnss.gnss1Dop = 1             # [vnpy.GnssDop] Dilution of precision
        # --- Attitude ---
        binaryOutput1Register.Attitude.quaternion = 1       #[vnpy.Quat] NED frame
        # --- INS ---
        binaryOutput1Register.Ins.insStatus = 1     # [vnpy.InsStatus]
        binaryOutput1Register.Ins.posLla = 1        # [vnpy.Lla] The estimated position
        binaryOutput1Register.Ins.velBody = 1       # [vnpy.Vec3f] The estimated velocity in the body-frame
        binaryOutput1Register.Ins.velNed = 1        # [vnpy.Vec3f] The estimated velocity in the NED-frame
        binaryOutput1Register.Ins.posU = 1          # [float] The estimated uncertainty (1 Sigma) in the current position estimate
        binaryOutput1Register.Ins.velU = 1          # [float] The estimated uncertainty (1 Sigma) in the current velocity estimate

        # Write and reset
        self.vs.writeRegister(binaryOutput1Register)
        self.vs.reset()
    
# ==============================================================================
# The ROS 2 Node Class
# ==============================================================================
class VectorNavNode(Node):
    def __init__(self):
        super().__init__('vectornav_node')

        self.declare_parameters(
            namespace='vectornav_config',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', '115200'),
                ('output_rate', '20'),
                ('gnss_antenna_offset', [1.0, 0.0, 0.0]),
                ('gnss_compass_baseline', [1.0, 0.0, 0.0, 0.0256, 0.0256, 0.0256])
            ]
        )