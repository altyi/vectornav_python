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
        self.declare_parameter('gnss_baseline', (1.0, 0.0, 0.0, 0.0254, 0.0254, 0.0254))
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
        self.gnss_baseline = self.get_parameter('gnss_baseline').value()
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
        self.vs = Sensor()
        self.connect_to_sensor()

        # --- Sensor Configuration ---
        self.configure_registers()
        self.configure_binary_output()

        # --- Save register settings if enabled ---
        if self.save_settings:
            self.save_register_settings()


        # --- Timer for Data Polling ---
        timer_period = 1.0 / (self.output_rate * 2.0)
        self.timer = self.create_timer(timer_period, self.poll_data)
    
    def configure_registers(self):
        # --- Coordiante Transform ---
        if self.global_frame_id == 'ENU':
            self.set_ned_to_enu_coordinate_tramsform()
        
        self.set_gnss_baseline()

        # send a write setting command
        try:
            self.vs.writesettings()
        except VnError as e:
            self.get_logger().error(f"Failed to write to register 26 for coordiante transform: {e}")
        # send a reset command
        try:
            self.vs.reset()
        except VnError as e:
            self.get_logger().error(f"Failed to reset for coordiante transform: {e}")

    def set_ned_to_enu_coordinate_tramsform(self):
        ned2enu_rot = Registers.IMU.RefFrameRot()
        ned2enu_rot.rfr00 = 0.0
        ned2enu_rot.rfr01 = 1.0
        ned2enu_rot.rfr02 = 0.0
        ned2enu_rot.rfr10 = 1.0
        ned2enu_rot.rfr11 = 0.0
        ned2enu_rot.rfr12 = 0.0
        ned2enu_rot.rfr20 = 0.0
        ned2enu_rot.rfr21 = 0.0
        ned2enu_rot.rfr22 = -1.0
        # write to register id 26
        self.vs.writeRegister(ned2enu_rot)
    
    def set_gnss_baseline(self):
        gnss_baseline = Registers.GNSSCompass.GnssCompassBaseline()
        gnss_baseline.positionX = self.gnss_baseline[0]
        gnss_baseline.positionY = self.gnss_baseline[1]
        gnss_baseline.positionZ = self.gnss_baseline[2]
        gnss_baseline.uncertaintyX = self.gnss_baseline[3]
        gnss_baseline.uncertaintyY = self.gnss_baseline[4]
        gnss_baseline.uncertaintyZ = self.gnss_baseline[5]
        self.vs.writeRegister(gnss_baseline)


        