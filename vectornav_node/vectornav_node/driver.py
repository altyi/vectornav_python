import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3Stamped, TwistStamped
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import yaml

from vectornav import Sensor, Registers, VnTypes, Plugins, VnError

class VectorNavNode(Node):
    """ ROS 2 Node for VectorNav sensors. """

    def __init__(self):
        super().__init__('vectornav_node')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'vectornav')
        self.declare_parameter('global_frame_id', 'NED') # 'NED' or 'ENU'
        self.declare_parameter('output_rate', 20)
        self.declare_parameter('gnss_antenna_offset', (1.0, 0.0, 0.0, 0.0254, 0.0254, 0.0254))
        self.declare_parameter('gnss_compass_baseline', (1.0, 0.0, 0.0, 0.0254, 0.0254, 0.0254))

        self.declare_parameter('save_settings_on_startup', True)
        self.declare_parameter('settings_filename', "vectornav_settings.yaml")
        self.declare_parameter('publish_data', {
            'imu': True, 'mag': True, 'temp': True, 'pres': True, 'gps_fix': True,
            'gps_vel': True, 'odometry': True, 'attitude_quat': True, 'attitude_ypr': True,
            'gnss_compass': True, 'diagnostics': True
        })

        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.global_frame_id = self.get_parameter('global_frame_id').value.upper()
        self.output_rate = self.get_parameter('output_rate').value
        self.gnss_antenna_offset = self.get_parameter('gnss_antenna_offset').value()
        self.gnss_compass_baseline = self.get_parameter('gnss_compass_baseline').value()
        self.save_settings = self.get_parameter('save_settings_on_startup').value
        self.settings_filename = self.get_parameter('settings_filename').value
        self.publish_flags = self.get_parameter('publish_data').value

        # --- Frame ID validation and setup ---
        if self.global_frame_id not in ["NED", "ENU"]:
            self.get_logger().error(f"Invalid global_frame_id '{self.global_frame_id}'. Must be 'NED' or 'ENU'. Defaulting to NED.")
            self.global_frame_id = "NED"
        
        self.ros_global_frame_id = f"{self.frame_id}_{self.global_frame_id.lower()}"
        self.get_logger().info(f"Sensor frame: '{self.frame_id}', Global frame: '{self.ros_global_frame_id}' ({self.global_frame_id})")
        
        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # --- Publishers ---
        self.pubs = {}
        if self.publish_flags.get('imu', False):
            self.pubs['imu'] = self.create_publisher(Imu, 'imu/data', qos_profile)
        if self.publish_flags.get('mag', False):
            self.pubs['mag'] = self.create_publisher(MagneticField, 'imu/mag', qos_profile)
        if self.publish_flags.get('temp', False):
            self.pubs['temp'] = self.create_publisher(Temperature, 'imu/temp', qos_profile)
        if self.publish_flags.get('pres', False):
            self.pubs['pres'] = self.create_publisher(FluidPressure, 'imu/pres', qos_profile)
        if self.publish_flags.get('gps_fix', False):
            self.pubs['gps_fix'] = self.create_publisher(NavSatFix, 'gps/fix', qos_profile)
        if self.publish_flags.get('gps_vel', False):
            self.pubs['gps_vel'] = self.create_publisher(TwistStamped, 'gps/vel', qos_profile)
        if self.publish_flags.get('odometry', False):
            self.pubs['odometry'] = self.create_publisher(Odometry, 'odom', qos_profile)
        if self.publish_flags.get('attitude_quat', False):
            self.pubs['attitude_quat'] = self.create_publisher(QuaternionStamped, 'attitude/quat', qos_profile)
        if self.publish_flags.get('attitude_ypr', False):
            self.pubs['attitude_ypr'] = self.create_publisher(Vector3Stamped, 'attitude/ypr', qos_profile)
        if self.publish_flags.get('gnss_compass', False):
            self.pubs['gnss_compass'] = self.create_publisher(Float64, 'gnss/compass_heading', qos_profile)
        
        # --- Sensor Connection ---
        self.connect_to_sensor()

        # # --- Sensor Configuration ---
        # self.configure_registers()
        # self.configure_binary_output()

        # # --- Save register settings if enabled ---
        # if self.save_settings:
        #     self.save_register_settings()


        # # --- Timer for Data Polling ---
        # timer_period = 1.0 / (self.output_rate * 2.0)
        # self.timer = self.create_timer(timer_period, self.poll_data)

    def baud_rate_table(self, baudrate: int) -> Sensor.BaudRate:
        """
        Converts an integer baud rate to its corresponding Sensor.BaudRate enum member.

        This function assumes Sensor.BaudRate is an Enum imported from a library.

        Args:
            baudrate: The integer value of the baud rate (e.g., 9600).

        Returns:
            The matching Sensor.BaudRate enum member.
        
        Raises:
            ValueError: If the provided baud rate is not supported by the Enum.
        """
        try:
            # The most Pythonic way: The Enum itself is a lookup table.
            # Call the Enum with the integer value to get the corresponding member.
            return Sensor.BaudRate(baudrate)
        except ValueError:
            # This block runs if the integer baudrate does not match any enum member's value.
            # We construct a helpful error message listing all supported rates.
            supported_rates = ", ".join(str(b.value) for b in Sensor.BaudRate)
            raise ValueError(
                f"Unsupported baud rate: {baudrate}. "
                f"Supported rates are: {supported_rates}"
            )  

    def connect_to_sensor(self):
        """ Connects to the VectorNav sensor. """
        self.vs = Sensor()
        self.get_logger().info(f"Connecting to sensor at {self.port} with baudrate {self.baudrate}...")
        try:
            self.vs.connect(self.port, self.baud_rate_table(self.baudrate))
            if not self.vs.verifySensorConnectivity():
                self.get_logger().error("Sensor connectivity check failed.")
                raise ConnectionError("Sensor connectivity failed")
            self.connected_port = self.vs.connectedPortName()
            self.connected_baud = self.vs.connectedBaudRate()
            self.get_logger().info(f"Successfully connected to the VectorNav sensor at port: {self.connected_port} with baud rate: {self.connected_baud}.")
            
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to the sensor: {e}")
            self.destroy_node()
            rclpy.shutdown()

    # def _set_register(self):
    #     # send a write setting command
    #     try:
    #         self.get_logger().info(f"writing settings to register ...")
    #         self.vs.writesettings()
    #     except VnError as e:
    #         self.get_logger().error(f"Failed to write to register 26 for coordiante transform: {e}")
    #     # send a reset command
    #     try:
    #         self.get_logger().info(f"resetting device ...")
    #         self.vs.reset()
    #     except VnError as e:
    #         self.get_logger().error(f"Failed to reset for coordiante transform: {e}")
    
    # def configure_registers(self):
    #     self.set_gnss_baseline()
    
    # # --- Configuration Register 57 & 93 ---
    # def set_gnss_baseline(self):
    #     gnss_antenna_offset = Registers.GNSS.GnssAOffset()
    #     gnss_antenna_offset.positionX = self.gnss_antenna_offset[0]
    #     gnss_antenna_offset.positionY = self.gnss_antenna_offset[1]
    #     gnss_antenna_offset.positionZ = self.gnss_antenna_offset[2]

    #     gnss_compass_baseline = Registers.GNSSCompass.GnssCompassBaseline()
    #     gnss_compass_baseline.positionX = self.gnss_compass_baseline[0]
    #     gnss_compass_baseline.positionY = self.gnss_compass_baseline[1]
    #     gnss_compass_baseline.positionZ = self.gnss_compass_baseline[2]
    #     gnss_compass_baseline.uncertaintyX = self.gnss_compass_baseline[3]
    #     gnss_compass_baseline.uncertaintyY = self.gnss_compass_baseline[4]
    #     gnss_compass_baseline.uncertaintyZ = self.gnss_compass_baseline[5]
    #     self.vs.writeRegister(gnss_compass_baseline)


        