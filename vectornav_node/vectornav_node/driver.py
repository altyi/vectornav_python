import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, QuaternionStamped, Vector3Stamped, TwistStamped
from std_msgs.msg import Float64
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

import math
import yaml
import inspect
from datetime import datetime

from vectornav import Sensor, Registers, VnTypes, Plugins

class VectorNavNode(Node):
    """ ROS 2 Node for VectorNav sensors. """

    def __init__(self):
        super().__init__('vectornav_node')

        # --- Parameters ---
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'vectornav')
        self.declare_parameter('global_frame_id', 'ENU') # 'NED' or 'ENU'
        self.declare_parameter('output_rate', 20)
        self.declare_parameter('use_binary_output', True)
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
        self.use_binary_output = self.get_parameter('use_binary_output').value
        self.save_settings = self.get_parameter('save_settings_on_startup').value
        self.settings_filename = self.get_parameter('settings_filename').value
        self.publish_flags = self.get_parameter('publish_data').value

        # --- Frame ID validation and setup ---
        if self.global_frame_id not in ["NED", "ENU"]:
            self.get_logger().error(f"Invalid global_frame_id '{self.global_frame_id}'. Must be 'NED' or 'ENU'. Defaulting to ENU.")
            self.global_frame_id = "ENU"
        
        self.ros_global_frame_id = f"{self.frame_id}_{self.global_frame_id.lower()}"
        self.get_logger().info(f"Sensor frame: '{self.frame_id}', Global frame: '{self.ros_global_frame_id}' ({self.global_frame_id})")
        
        # --- QoS Profile ---
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
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
        if self.publish_flags.get('diagnostics', False):
            self.pubs['diagnostics'] = self.create_publisher(DiagnosticArray, '/diagnostics', qos_profile)

        # --- Sensor Connection ---
        self.vs = Sensor()
        self.connect_to_sensor()

        # --- Save register settings if enabled ---
        if self.save_settings:
            self.save_register_settings()

        # --- Sensor Configuration ---
        if self.use_binary_output:
            self.configure_binary_output()

        # --- Timer for Data Polling ---
        timer_period = 1.0 / (self.output_rate * 2.0)
        self.timer = self.create_timer(timer_period, self.poll_data)

    def connect_to_sensor(self):
        """ Connects to the VectorNav sensor. """
        self.get_logger().info(f"Connecting to sensor at {self.port} with baudrate {self.baudrate}...")
        try:
            self.vs.connect(self.port, self.baudrate)
            if not self.vs.verifySensorConnectivity():
                self.get_logger().error("Sensor connectivity check failed.")
                raise ConnectionError("Sensor connectivity failed")
            
            self.get_logger().info("Successfully connected to the VectorNav sensor.")
            
        except Exception as e:
            self.get_logger().fatal(f"Could not connect to the sensor: {e}")
            self.destroy_node()
            rclpy.shutdown()

    def _register_to_dict(self, reg_obj):
        """Recursively converts a register object to a dictionary."""
        d = {}
        for attr_name in dir(reg_obj):
            if not attr_name.startswith('_') and not callable(getattr(reg_obj, attr_name)):
                attr_val = getattr(reg_obj, attr_name)
                # Check for nested VectorNav objects (excluding the top-level class itself)
                if 'vectornav.VnTypes' in str(type(attr_val)) or 'vectornav.Registers' in str(type(attr_val)):
                    d[attr_name] = self._register_to_dict(attr_val)
                # Check for enums (pybind11 enums are tricky to check directly)
                elif hasattr(attr_val, 'name'):
                    d[attr_name] = attr_val.name
                else:
                    d[attr_name] = attr_val
        return d

    def save_register_settings(self):
        """Reads all configuration registers and saves them to a YAML file."""
        self.get_logger().info(f"Saving register settings to '{self.settings_filename}'...")
        # A map from register ID to a tuple of (Name, Class)
        reg_map = {
            0: ("UserTag", Registers.System.UserTag),
            5: ("BaudRate", Registers.System.BaudRate),
            6: ("AsyncOutputType", Registers.System.AsyncOutputType),
            7: ("AsyncOutputFreq", Registers.System.AsyncOutputFreq),
            21: ("MagGravRefVec", Registers.Attitude.MagGravRefVec),
            23: ("MagCal", Registers.IMU.MagCal),
            25: ("AccelCal", Registers.IMU.AccelCal),
            26: ("RefFrameRot", Registers.IMU.RefFrameRot),
            30: ("ProtocolControl", Registers.System.ProtocolControl),
            32: ("SyncControl", Registers.System.SyncControl),
            35: ("VpeBasicControl", Registers.Attitude.VpeBasicControl),
            36: ("VpeMagBasicTuning", Registers.Attitude.VpeMagBasicTuning),
            38: ("VpeAccelBasicTuning", Registers.Attitude.VpeAccelBasicTuning),
            44: ("RealTimeHsiControl", Registers.HardSoftIronEstimator.RealTimeHsiControl),
            50: ("VelAidingMeas", Registers.VelocityAiding.VelAidingMeas),
            51: ("VelAidingControl", Registers.VelocityAiding.VelAidingControl),
            55: ("GnssBasicConfig", Registers.GNSS.GnssBasicConfig),
            57: ("GnssAOffset", Registers.GNSS.GnssAOffset),
            67: ("InsBasicConfig", Registers.INS.InsBasicConfig),
            74: ("FilterStartupBias", Registers.INS.FilterStartupBias),
            75: ("BinaryOutput1", Registers.System.BinaryOutput1),
            76: ("BinaryOutput2", Registers.System.BinaryOutput2),
            77: ("BinaryOutput3", Registers.System.BinaryOutput3),
            82: ("DeltaThetaVelConfig", Registers.IMU.DeltaThetaVelConfig),
            83: ("RefModelConfig", Registers.WorldMagGravityModel.RefModelConfig),
            84: ("GyroCal", Registers.IMU.GyroCal),
            85: ("ImuFilterControl", Registers.IMU.ImuFilterControl),
            93: ("GnssCompassBaseline", Registers.GNSSCompass.GnssCompassBaseline),
            99: ("GnssSystemConfig", Registers.GNSS.GnssSystemConfig),
            100: ("GnssSyncConfig", Registers.GNSS.GnssSyncConfig),
            101: ("NmeaOutput1", Registers.System.NmeaOutput1),
            102: ("NmeaOutput2", Registers.System.NmeaOutput2),
            105: ("InsRefOffset", Registers.INS.InsRefOffset),
            116: ("HeaveBasicConfig", Registers.Heave.HeaveBasicConfig),
            144: ("InsGnssSelect", Registers.INS.InsGnssSelect),
            157: ("ExtGnssOffset", Registers.GNSS.ExtGnssOffset),
            206: ("LegacyCompatibilitySettings", Registers.System.LegacyCompatibilitySettings)
        }

        all_settings = {}
        try:
            reg_ids_to_read = Plugins.getDefaultConfigRegisters()
            for reg_id in reg_ids_to_read:
                if reg_id in reg_map:
                    name, reg_class = reg_map[reg_id]
                    reg_instance = reg_class()
                    self.vs.readRegister(reg_instance)
                    all_settings[name] = self._register_to_dict(reg_instance)
            
            with open(self.settings_filename, 'w') as f:
                yaml.dump(all_settings, f, indent=4, default_flow_style=False)
            
            self.get_logger().info("Successfully saved register settings.")
        except Exception as e:
            self.get_logger().error(f"Failed to save register settings: {e}")
            

    def configure_binary_output(self):
        """ Configures the sensor to output necessary data via binary messages. """
        self.get_logger().info("Configuring binary output...")
        try:
            # First, stop existing async output
            self.vs.writeRegister(Registers.AsyncOutputType(ador=VnTypes.AsciiAsync.OFF))
            
            # Configure the binary output register
            bo1 = Registers.BinaryOutput1()
            
            # IMU GROUP: For Imu, MagneticField, Temperature, Pressure, Odometry (angular rate)
            bo1.imu.imuStatus = True
            bo1.imu.uncompMag = True
            bo1.imu.uncompAccel = True
            bo1.imu.uncompGyro = True
            bo1.imu.temperature = True
            bo1.imu.pressure = True
            bo1.imu.mag = True
            bo1.imu.accel = True
            bo1.imu.angularRate = True

            # ATTITUDE GROUP: For Imu (orientation), Quaternion, YPR, Odometry (orientation)
            bo1.attitude.ypr = True
            bo1.attitude.quaternion = True
            bo1.attitude.attU = True # Attitude uncertainty for covariance

            # INS GROUP: For Odometry (linear velocity), TwistStamped
            bo1.ins.insStatus = True
            bo1.ins.velNed = True
            bo1.ins.velU = True # Velocity uncertainty for covariance

            # GNSS GROUP: For NavSatFix
            bo1.gnss.gnss1Status = True
            bo1.gnss.gnss1Fix = True
            bo1.gnss.gnss1PosLla = True
            bo1.gnss.gnss1PosUncertainty = True

            # Assuming an IMU rate of 400Hz, which is common.
            # You might need to query the actual rate from the sensor if it varies.
            imu_rate = 400 
            divisor = max(1, round(imu_rate / self.output_rate))
            bo1.rateDivisor = int(divisor)
            actual_rate = imu_rate / divisor
            self.get_logger().info(f"Requested rate: {self.output_rate}Hz. Using divisor: {divisor}. Actual rate: {actual_rate:.2f}Hz.")
            
            # Set async mode to PORT1
            bo1.asyncMode = VnTypes.AsyncMode.SERIAL_1

            self.vs.writeRegister(bo1)
            self.get_logger().info("Binary output configured.")

        except Exception as e:
            self.get_logger().error(f"Failed to configure binary output: {e}")
            
    def poll_data(self):
        """ Polls for the next available measurement from the sensor. """
        cd = self.vs.getNextMeasurement()
        if cd is None:
            return

        stamp = self.get_clock().now().to_msg()

        # Publish data based on available groups
        if cd.attitude is not None and cd.imu is not None:
            self.publish_imu(cd, stamp)
            self.publish_mag(cd, stamp)
            self.publish_temp(cd, stamp)
            self.publish_pres(cd, stamp)
            self.publish_quat(cd, stamp)
            self.publish_ypr(cd, stamp)
            self.publish_gnss_compass(cd, stamp)
            
        if cd.gnss is not None:
            self.publish_navsat(cd, stamp)

        if cd.ins is not None and cd.imu is not None and cd.attitude is not None:
            self.publish_velocity(cd, stamp)
            self.publish_odometry(cd, stamp)
        
        # Publish diagnostics with any available data
        self.publish_diagnostics(cd, stamp)

    def _get_orientation_quaternion(self, cd):
        """ 
        Returns a geometry_msgs/Quaternion, converting to ENU if configured.
        This method builds the quaternion from YPR to ensure it includes the
        GNSS compass heading, as was the behavior in the original implementation.
        """
        yaw_rad = math.radians(cd.attitude.ypr.yaw)
        pitch_rad = math.radians(cd.attitude.ypr.pitch)
        roll_rad = math.radians(cd.attitude.ypr.roll)

        if self.global_frame_id == 'ENU':
            # Convert NED YPR to ENU YPR for ROS standards (REP-103)
            # yaw_enu = 90 - yaw_ned -> pi/2 - yaw_ned_rad
            # pitch_enu = -pitch_ned
            # roll_enu = roll_ned
            yaw_rad = math.pi/2.0 - yaw_rad
            pitch_rad = -pitch_rad
            # roll_rad is unchanged
            
            # Normalize yaw to [-pi, pi]
            yaw_rad = (yaw_rad + math.pi) % (2 * math.pi) - math.pi

        # Convert from RPY to quaternion.
        cy = math.cos(yaw_rad * 0.5)
        sy = math.sin(yaw_rad * 0.5)
        cp = math.cos(pitch_rad * 0.5)
        sp = math.sin(pitch_rad * 0.5)
        cr = math.cos(roll_rad * 0.5)
        sr = math.sin(roll_rad * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

    def publish_imu(self, cd, stamp):
        if not self.publish_flags.get('imu', False) or \
           cd.attitude.quaternion is None or \
           cd.attitude.ypr is None or \
           cd.imu.angularRate is None or \
           cd.imu.accel is None:
            return
            
        msg = Imu()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        # Get orientation, converting to ENU if necessary
        q = self._get_orientation_quaternion(cd)
        msg.orientation = q
        
        # Angular velocity and linear acceleration are in the body frame, no conversion needed.
        msg.angular_velocity.x = cd.imu.angularRate.x
        msg.angular_velocity.y = cd.imu.angularRate.y
        msg.angular_velocity.z = cd.imu.angularRate.z
        
        msg.linear_acceleration.x = cd.imu.accel.x
        msg.linear_acceleration.y = cd.imu.accel.y
        msg.linear_acceleration.z = cd.imu.accel.z
        
        # Covariances
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0
        if cd.attitude.attU is not None:
            att_unc_rad = math.radians(cd.attitude.attU)
            att_var = att_unc_rad ** 2
            msg.orientation_covariance[0] = att_var
            msg.orientation_covariance[4] = att_var
            msg.orientation_covariance[8] = att_var
        else:
            msg.orientation_covariance[0] = -1.0
        
        self.pubs['imu'].publish(msg)

    def publish_mag(self, cd, stamp):
        if not self.publish_flags.get('mag', False) or cd.imu.mag is None:
            return
            
        msg = MagneticField()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        gauss_to_tesla = 1e-4
        msg.magnetic_field.x = cd.imu.mag.x * gauss_to_tesla
        msg.magnetic_field.y = cd.imu.mag.y * gauss_to_tesla
        msg.magnetic_field.z = cd.imu.mag.z * gauss_to_tesla
        msg.magnetic_field_covariance[0] = -1.0

        self.pubs['mag'].publish(msg)

    def publish_temp(self, cd, stamp):
        if not self.publish_flags.get('temp', False) or cd.imu.temperature is None:
            return
            
        msg = Temperature()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        msg.temperature = cd.imu.temperature
        msg.variance = 0.0

        self.pubs['temp'].publish(msg)

    def publish_pres(self, cd, stamp):
        if not self.publish_flags.get('pres', False) or cd.imu.pressure is None:
            return
        
        msg = FluidPressure()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        kpa_to_pa = 1000.0
        msg.fluid_pressure = cd.imu.pressure * kpa_to_pa
        msg.variance = 0.0
        
        self.pubs['pres'].publish(msg)

    def publish_navsat(self, cd, stamp):
        if not self.publish_flags.get('gps_fix', False) or \
           cd.gnss.gnss1PosLla is None or \
           cd.gnss.gnss1Fix is None or \
           cd.gnss.gnss1PosUncertainty is None:
            return
            
        msg = NavSatFix()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id # LLA is frame-agnostic
        
        msg.latitude = cd.gnss.gnss1PosLla.lat
        msg.longitude = cd.gnss.gnss1PosLla.lon
        msg.altitude = cd.gnss.gnss1PosLla.alt
        
        fix_map = { 0: NavSatStatus.STATUS_NO_FIX, 1: NavSatStatus.STATUS_FIX, 2: NavSatStatus.STATUS_FIX, 
                    3: NavSatStatus.STATUS_FIX, 4: NavSatStatus.STATUS_SBAS_FIX, 5: NavSatStatus.STATUS_GBAS_FIX,
                    6: NavSatStatus.STATUS_GBAS_FIX}
        msg.status.status = fix_map.get(cd.gnss.gnss1Fix, NavSatStatus.STATUS_NO_FIX)
        msg.status.service = NavSatStatus.SERVICE_GPS
        
        unc_n, unc_e, unc_d = cd.gnss.gnss1PosUncertainty.x, cd.gnss.gnss1PosUncertainty.y, cd.gnss.gnss1PosUncertainty.z
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        msg.position_covariance[0] = unc_e**2
        msg.position_covariance[4] = unc_n**2
        msg.position_covariance[8] = unc_d**2
        
        self.pubs['gps_fix'].publish(msg)

    def publish_velocity(self, cd, stamp):
        if not self.publish_flags.get('gps_vel', False) or cd.ins.velNed is None:
            return

        msg = TwistStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.ros_global_frame_id

        if self.global_frame_id == 'ENU':
            # Convert NED velocity to ENU
            msg.twist.linear.x = cd.ins.velNed.y  # East
            msg.twist.linear.y = cd.ins.velNed.x  # North
            msg.twist.linear.z = -cd.ins.velNed.z # Up
        else: # NED
            msg.twist.linear.x = cd.ins.velNed.x  # North
            msg.twist.linear.y = cd.ins.velNed.y  # East
            msg.twist.linear.z = cd.ins.velNed.z  # Down

        msg.twist.angular.x = float('nan')
        msg.twist.angular.y = float('nan')
        msg.twist.angular.z = float('nan')

        self.pubs['gps_vel'].publish(msg)

    def publish_odometry(self, cd, stamp):
        if not self.publish_flags.get('odometry', False) or \
           cd.attitude.quaternion is None or \
           cd.attitude.ypr is None or \
           cd.ins.velNed is None or \
           cd.imu.angularRate is None:
            return

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.ros_global_frame_id
        msg.child_frame_id = self.frame_id

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0

        # Get orientation, converting to ENU if necessary
        q = self._get_orientation_quaternion(cd)
        msg.pose.pose.orientation = q

        # Convert NED velocity to ENU if necessary
        if self.global_frame_id == 'ENU':
            msg.twist.twist.linear.x = cd.ins.velNed.y  # East
            msg.twist.twist.linear.y = cd.ins.velNed.x  # North
            msg.twist.twist.linear.z = -cd.ins.velNed.z # Up
        else: # NED
            msg.twist.twist.linear.x = cd.ins.velNed.x
            msg.twist.twist.linear.y = cd.ins.velNed.y
            msg.twist.twist.linear.z = cd.ins.velNed.z

        # Angular rate is in body frame, no conversion needed
        msg.twist.twist.angular.x = cd.imu.angularRate.x
        msg.twist.twist.angular.y = cd.imu.angularRate.y
        msg.twist.twist.angular.z = cd.imu.angularRate.z

        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36

        if cd.attitude.attU is not None:
            att_var = math.radians(cd.attitude.attU) ** 2
            msg.pose.covariance[21] = att_var # roll
            msg.pose.covariance[28] = att_var # pitch
            msg.pose.covariance[35] = att_var # yaw
        
        if cd.ins.velU is not None:
            vel_var = cd.ins.velU ** 2
            msg.twist.covariance[0] = vel_var
            msg.twist.covariance[7] = vel_var
            msg.twist.covariance[14] = vel_var
    
        self.pubs['odometry'].publish(msg)

    def publish_quat(self, cd, stamp):
        if not self.publish_flags.get('attitude_quat', False) or \
           cd.attitude.quaternion is None or \
           cd.attitude.ypr is None:
            return
            
        msg = QuaternionStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        
        q = self._get_orientation_quaternion(cd)
        msg.quaternion = q
        
        self.pubs['attitude_quat'].publish(msg)
        
    def publish_ypr(self, cd, stamp):
        if not self.publish_flags.get('attitude_ypr', False) or cd.attitude.ypr is None:
            return
            
        msg = Vector3Stamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id

        if self.global_frame_id == 'ENU':
            yaw_ned_rad = math.radians(cd.attitude.ypr.yaw)
            pitch_ned_rad = math.radians(cd.attitude.ypr.pitch)
            roll_ned_rad = math.radians(cd.attitude.ypr.roll)

            # Convert to ENU frame in radians, maintaining original x,y,z mapping
            yaw_enu_rad = math.pi/2.0 - yaw_ned_rad
            # Normalize yaw to [-pi, pi]
            yaw_enu_rad = (yaw_enu_rad + math.pi) % (2 * math.pi) - math.pi

            msg.vector.x = yaw_enu_rad                   # Yaw
            msg.vector.y = -pitch_ned_rad                # Pitch
            msg.vector.z = roll_ned_rad                  # Roll
        else: # NED
            msg.vector.x = math.radians(cd.attitude.ypr.yaw)
            msg.vector.y = math.radians(cd.attitude.ypr.pitch)
            msg.vector.z = math.radians(cd.attitude.ypr.roll)
        
        self.pubs['attitude_ypr'].publish(msg)

    def publish_gnss_compass(self, cd, stamp):
        if not self.publish_flags.get('gnss_compass', False) or cd.attitude.ypr is None:
            return
        
        msg = Float64()
        yaw_ned_rad = math.radians(cd.attitude.ypr.yaw)
        
        if self.global_frame_id == 'ENU':
            yaw_enu_rad = math.pi/2.0 - yaw_ned_rad
            # Normalize yaw to [-pi, pi]
            yaw_enu_rad = (yaw_enu_rad + math.pi) % (2 * math.pi) - math.pi
            msg.data = yaw_enu_rad
        else: # NED
            msg.data = yaw_ned_rad

        self.pubs['gnss_compass'].publish(msg)

    def publish_diagnostics(self, cd, stamp):
        if not self.publish_flags.get('diagnostics', False):
            return

        diag_array = DiagnosticArray()
        diag_array.header.stamp = stamp

        # Helper function to add key-value if the value is not None
        def add_if_present(kv_list, key, value):
            if value is not None:
                kv_list.append(KeyValue(key=key, value=str(value)))

        # Helper to format vector types
        def format_vec(v):
            if v is None: return "N/A"
            if hasattr(v, 'w'): # Quaternion
                return f"w:{v.w:.4f}, x:{v.x:.4f}, y:{v.y:.4f}, z:{v.z:.4f}"
            if hasattr(v, 'yaw'): # YPR
                 return f"Y:{v.yaw:.2f}, P:{v.pitch:.2f}, R:{v.roll:.2f}"
            if hasattr(v, 'lat'): # LLA
                 return f"Lat:{v.lat:.7f}, Lon:{v.lon:.7f}, Alt:{v.alt:.2f}"
            if hasattr(v, 'x'): # Vec3f/d
                return f"x:{v.x:.4f}, y:{v.y:.4f}, z:{v.z:.4f}"
            return "Unknown vector format"
        
        # Helper to format UTC time
        def format_utc(t):
            if t is None: return "N/A"
            return f"{2000+t.year:04d}-{t.month:02d}-{t.day:02d} {t.hour:02d}:{t.min:02d}:{t.sec:02d}.{t.ms:03d}Z"
        
        # --- TimeGroup ---
        if cd.time is not None:
            stat = DiagnosticStatus(name="VectorNav Time", hardware_id=self.frame_id)
            stat.level = DiagnosticStatus.OK
            stat.message = "Time Data"
            
            add_if_present(stat.values, "Time Startup (ns)", cd.time.timeStartup)
            add_if_present(stat.values, "Time GPS (ns)", cd.time.timeGps)
            add_if_present(stat.values, "Time GPS TOW (ns)", cd.time.timeGpsTow)
            add_if_present(stat.values, "GPS Week", cd.time.timeGpsWeek)
            add_if_present(stat.values, "Time SyncIn (ns)", cd.time.timeSyncIn)
            add_if_present(stat.values, "Time GPS PPS (ns)", cd.time.timeGpsPps)
            add_if_present(stat.values, "Time UTC", format_utc(cd.time.timeUtc))
            add_if_present(stat.values, "SyncIn Count", cd.time.syncInCnt)
            add_if_present(stat.values, "SyncOut Count", cd.time.syncOutCnt)
            if cd.time.timeStatus is not None:
                ts = cd.time.timeStatus
                stat.values.append(KeyValue(key="Time Status", value=f"{ts.name} ({ts.value})"))
                if not ts.timeOk:
                    stat.level = DiagnosticStatus.WARN
                    stat.message = "Time status not OK"
            diag_array.status.append(stat)

        # --- ImuGroup ---
        if cd.imu is not None:
            stat = DiagnosticStatus(name="VectorNav IMU", hardware_id=self.frame_id)
            is_ok = True
            message = "OK"
            if cd.imu.imuStatus is not None:
                status = cd.imu.imuStatus
                is_ok = not (status.gyroSaturated or status.accelSaturated or status.magSaturated or status.pressureSaturated)
                message = "OK" if is_ok else "Sensor saturation detected"
                add_if_present(stat.values, "IMU Status", f"{status.name} ({status.value})")
                add_if_present(stat.values, "Gyro Saturated", status.gyroSaturated)
                add_if_present(stat.values, "Accel Saturated", status.accelSaturated)
                add_if_present(stat.values, "Mag Saturated", status.magSaturated)
                add_if_present(stat.values, "Pressure Saturated", status.pressureSaturated)
            
            stat.level = DiagnosticStatus.OK if is_ok else DiagnosticStatus.WARN
            stat.message = message

            add_if_present(stat.values, "Uncomp Mag (Gauss)", format_vec(cd.imu.uncompMag))
            add_if_present(stat.values, "Uncomp Accel (m/s^2)", format_vec(cd.imu.uncompAccel))
            add_if_present(stat.values, "Uncomp Gyro (rad/s)", format_vec(cd.imu.uncompGyro))
            add_if_present(stat.values, "Temperature (C)", cd.imu.temperature)
            add_if_present(stat.values, "Pressure (kPa)", cd.imu.pressure)
            if cd.imu.deltaTheta is not None:
                 stat.values.append(KeyValue(key="Delta Theta (rad)", value=f"dt:{cd.imu.deltaTheta.dt:.4f}, {format_vec(cd.imu.deltaTheta)}"))
            add_if_present(stat.values, "Delta Vel (m/s)", format_vec(cd.imu.deltaVel))
            add_if_present(stat.values, "Mag (Gauss)", format_vec(cd.imu.mag))
            add_if_present(stat.values, "Accel (m/s^2)", format_vec(cd.imu.accel))
            add_if_present(stat.values, "Angular Rate (rad/s)", format_vec(cd.imu.angularRate))
            add_if_present(stat.values, "Sensor Saturation", cd.imu.sensSat)
            diag_array.status.append(stat)

        # --- AttitudeGroup ---
        if cd.attitude is not None:
            stat = DiagnosticStatus(name="VectorNav Attitude", hardware_id=self.frame_id)
            stat.level = DiagnosticStatus.OK
            stat.message = "Attitude Data"
            add_if_present(stat.values, "YPR (deg)", format_vec(cd.attitude.ypr))
            add_if_present(stat.values, "Quaternion", format_vec(cd.attitude.quaternion))
            if cd.attitude.dcm is not None:
                dcm = cd.attitude.dcm
                dcm_str = f"[{dcm.m11:.4f} {dcm.m12:.4f} {dcm.m13:.4f}; {dcm.m21:.4f} {dcm.m22:.4f} {dcm.m23:.4f}; {dcm.m31:.4f} {dcm.m32:.4f} {dcm.m33:.4f}]"
                add_if_present(stat.values, "DCM", dcm_str)
            add_if_present(stat.values, "Mag NED (Gauss)", format_vec(cd.attitude.magNed))
            add_if_present(stat.values, "Accel NED (m/s^2)", format_vec(cd.attitude.accelNed))
            add_if_present(stat.values, "Linear Accel Body (m/s^2)", format_vec(cd.attitude.linBodyAcc))
            add_if_present(stat.values, "Linear Accel NED (m/s^2)", format_vec(cd.attitude.linAccelNed))
            add_if_present(stat.values, "YPR Uncertainty (deg)", format_vec(cd.attitude.yprU))
            add_if_present(stat.values, "Heave", format_vec(cd.attitude.heave))
            add_if_present(stat.values, "Attitude Uncertainty (deg)", cd.attitude.attU)
            diag_array.status.append(stat)

        # --- InsGroup ---
        if cd.ins is not None:
            stat = DiagnosticStatus(name="VectorNav INS", hardware_id=self.frame_id)
            if cd.ins.insStatus is not None:
                mode = cd.ins.insStatus.mode
                if mode == VnTypes.InsMode.Off:
                    stat.level = DiagnosticStatus.WARN
                    stat.message = "INS Off"
                elif mode == VnTypes.InsMode.Ahrs:
                    stat.level = DiagnosticStatus.OK
                    stat.message = "INS in AHRS mode"
                else:
                    stat.level = DiagnosticStatus.OK
                    stat.message = f"INS Mode: {mode.name}"
                
                if cd.ins.insStatus.imuErr or cd.ins.insStatus.magPresErr or cd.ins.insStatus.gnssErr:
                    stat.level = DiagnosticStatus.ERROR
                    stat.message += " (Errors Present)"

                add_if_present(stat.values, "INS Status", f"{cd.ins.insStatus.name} ({cd.ins.insStatus.value})")
                add_if_present(stat.values, "INS Mode", str(mode.name))
                add_if_present(stat.values, "GNSS Fix", str(cd.ins.insStatus.gnssFix))
                add_if_present(stat.values, "IMU Error", str(cd.ins.insStatus.imuErr))
                add_if_present(stat.values, "Mag/Pres Error", str(cd.ins.insStatus.magPresErr))
                add_if_present(stat.values, "GNSS Error", str(cd.ins.insStatus.gnssErr))
            else:
                stat.level = DiagnosticStatus.WARN
                stat.message = "INS Status not available"

            add_if_present(stat.values, "Pos LLA", format_vec(cd.ins.posLla))
            add_if_present(stat.values, "Pos ECEF (m)", format_vec(cd.ins.posEcef))
            add_if_present(stat.values, "Vel Body (m/s)", format_vec(cd.ins.velBody))
            add_if_present(stat.values, "Vel NED (m/s)", format_vec(cd.ins.velNed))
            add_if_present(stat.values, "Vel ECEF (m/s)", format_vec(cd.ins.velEcef))
            add_if_present(stat.values, "Mag ECEF (Gauss)", format_vec(cd.ins.magEcef))
            add_if_present(stat.values, "Accel ECEF (m/s^2)", format_vec(cd.ins.accelEcef))
            add_if_present(stat.values, "Linear Accel ECEF (m/s^2)", format_vec(cd.ins.linAccelEcef))
            add_if_present(stat.values, "Pos Uncertainty (m)", cd.ins.posU)
            add_if_present(stat.values, "Vel Uncertainty (m/s)", cd.ins.velU)
            diag_array.status.append(stat)
            
        # --- GNSS Groups (1, 2, 3) ---
        def process_gnss(gnss_group, name_prefix, num_prefix):
            if gnss_group is None:
                return None
            
            stat = DiagnosticStatus(name=f"VectorNav {name_prefix}", hardware_id=self.frame_id)
            stat.level = DiagnosticStatus.OK
            stat.message = "GNSS OK"
            
            fix_type_map = {
                0: "No Fix", 1: "Time Only", 2: "2D Fix", 3: "3D Fix",
                4: "SBAS Aided", 5: "RTK Float", 6: "RTK Fixed"
            }
            
            fix = getattr(gnss_group, f'gnss{num_prefix}Fix', None)
            if fix is not None:
                fix_string = fix_type_map.get(fix, f"Unknown ({fix})")
                if fix == 0:
                    stat.level = DiagnosticStatus.WARN
                    stat.message = "No GNSS Fix"
                else:
                    stat.message = f"GNSS OK ({fix_string})"
                add_if_present(stat.values, "Fix Type", fix_string)
            else:
                stat.level = DiagnosticStatus.WARN
                stat.message = "GNSS Fix status not available"

            add_if_present(stat.values, "UTC Time", format_utc(getattr(gnss_group, f'gnss{num_prefix}TimeUtc', None)))
            add_if_present(stat.values, "TOW (ns)", getattr(gnss_group, f'gps{num_prefix}Tow', None))
            add_if_present(stat.values, "Week", getattr(gnss_group, f'gps{num_prefix}Week', None))
            add_if_present(stat.values, "Num Sats", getattr(gnss_group, f'gnss{num_prefix}NumSats', None))
            add_if_present(stat.values, "Pos LLA", format_vec(getattr(gnss_group, f'gnss{num_prefix}PosLla', None)))
            add_if_present(stat.values, "Pos ECEF (m)", format_vec(getattr(gnss_group, f'gnss{num_prefix}PosEcef', None)))
            add_if_present(stat.values, "Vel NED (m/s)", format_vec(getattr(gnss_group, f'gnss{num_prefix}VelNed', None)))
            add_if_present(stat.values, "Vel ECEF (m/s)", format_vec(getattr(gnss_group, f'gnss{num_prefix}VelEcef', None)))
            add_if_present(stat.values, "Pos Uncertainty NED (m)", format_vec(getattr(gnss_group, f'gnss{num_prefix}PosUncertainty', None)))
            add_if_present(stat.values, "Vel Uncertainty (m/s)", getattr(gnss_group, f'gnss{num_prefix}VelUncertainty', None))
            add_if_present(stat.values, "Time Uncertainty (ns)", getattr(gnss_group, f'gnss{num_prefix}TimeUncertainty', None))
            
            time_info = getattr(gnss_group, f'gnss{num_prefix}TimeInfo', None)
            if time_info is not None: add_if_present(stat.values, "Time Info", f"{time_info.name} ({time_info.value})")

            dop = getattr(gnss_group, f'gnss{num_prefix}Dop', None)
            if dop is not None:
                dop_str = f"G:{dop.g:.2f}, P:{dop.p:.2f}, T:{dop.t:.2f}, V:{dop.v:.2f}, H:{dop.h:.2f}, N:{dop.n:.2f}"
                add_if_present(stat.values, "DOP", dop_str)

            status = getattr(gnss_group, f'gnss{num_prefix}Status', None)
            if status is not None: add_if_present(stat.values, "Status", f"{status.name} ({status.value})")

            add_if_present(stat.values, "Alt MSL (m)", getattr(gnss_group, f'gnss{num_prefix}AltMsl', None))
            return stat

        gnss1_stat = process_gnss(cd.gnss, "GNSS1", "1")
        if gnss1_stat: diag_array.status.append(gnss1_stat)
        
        gnss2_stat = process_gnss(cd.gnss2, "GNSS2", "2")
        if gnss2_stat: diag_array.status.append(gnss2_stat)

        gnss3_stat = process_gnss(cd.gnss3, "GNSS3", "3")
        if gnss3_stat: diag_array.status.append(gnss3_stat)

        if len(diag_array.status) > 0:
            self.pubs['diagnostics'].publish(diag_array)


    def destroy_node(self):
        """ Cleans up the node and disconnects from the sensor. """
        self.get_logger().info("Shutting down node and disconnecting from sensor.")
        if self.vs.connectedPortName() is not None:
            self.vs.disconnect()
        super().destroy_node()