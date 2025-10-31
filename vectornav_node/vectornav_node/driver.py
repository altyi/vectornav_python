import queue
import time
from threading import Thread
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
import numpy as np
import math

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

from vectornav import Sensor, Registers, Plugins

# ==============================================================================
# Helper functions for Coordinate Transformations
# ==============================================================================
# WGS84 ellipsoid constants
WGS84_A = 6378137.0  # Semi-major axis
WGS84_E2 = 0.00669437999014  # Eccentricity squared

def lla_to_ecef(lat, lon, alt):
    """Converts LLA (deg, deg, m) to ECEF (m, m, m) coordinates."""
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    
    N = WGS84_A / math.sqrt(1 - WGS84_E2 * sin_lat**2)
    
    x = (N + alt) * cos_lat * math.cos(lon_rad)
    y = (N + alt) * cos_lat * math.sin(lon_rad)
    z = ((1 - WGS84_E2) * N + alt) * sin_lat
    
    return np.array([x, y, z])

def ecef_to_enu_matrix(lat, lon):
    """Creates a rotation matrix to convert ECEF to ENU coordinates."""
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    sin_lon = math.sin(lon_rad)
    cos_lon = math.cos(lon_rad)
    
    return np.array([
        [-sin_lon, cos_lon, 0],
        [-sin_lat * cos_lon, -sin_lat * sin_lon, cos_lat],
        [cos_lat * cos_lon, cos_lat * sin_lon, sin_lat]
    ])

def quaternion_multiply(q1, q0):
    """Multiplies two quaternions (q1 * q0)."""
    w0, x0, y0, z0 = q0
    w1, x1, y1, z1 = q1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                     x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                     x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

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
    map_frame_id: str
    publish_odometry: bool
    save_settings_on_startup: bool
    settings_filename: str

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
        self.logger().info(f"Connecting to sensor at {self.config.port} with baudrate {self.config.baudrate}...")
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
        gnss_antenna_offset = Registers.GnssAOffset()
        gnss_antenna_offset.positionX = self.config.gnss_antenna_offset[0]
        gnss_antenna_offset.positionY = self.config.gnss_antenna_offset[1]
        gnss_antenna_offset.positionZ = self.config.gnss_antenna_offset[2]
        self.vs.writeRegister(gnss_antenna_offset)

        gnss_compass_baseline = Registers.GnssCompassBaseline()
        gnss_compass_baseline.positionX = self.config.gnss_compass_baseline[0]
        gnss_compass_baseline.positionY = self.config.gnss_compass_baseline[1]
        gnss_compass_baseline.positionZ = self.config.gnss_compass_baseline[2]
        gnss_compass_baseline.uncertaintyX = self.config.gnss_compass_baseline[3]
        gnss_compass_baseline.uncertaintyY = self.config.gnss_compass_baseline[4]
        gnss_compass_baseline.uncertaintyZ = self.config.gnss_compass_baseline[5]
        self.vs.writeRegister(gnss_compass_baseline)

        self.vs.reset()

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
        self.binaryOutput1Register.attitude.yprU = 1             # [float] The estimated uncertainty (1 Sigma) in the current attitude estimate.
        # --- INS ---
        self.binaryOutput1Register.ins.insStatus = 1             # [vnpy.InsStatus]
        self.binaryOutput1Register.ins.posLla = 1                # [vnpy.Lla] The estimated position
        self.binaryOutput1Register.ins.posEcef = 1               # [vnpy.Vec3d] Position in ECEF frame
        self.binaryOutput1Register.ins.velBody = 1               # [vnpy.Vec3f] The estimated velocity in the body-frame
        self.binaryOutput1Register.ins.velNed = 1                # [vnpy.Vec3f] The estimated velocity in the NED-frame
        self.binaryOutput1Register.ins.posU = 1                  # [float] The estimated uncertainty (1 Sigma) in the current position estimate
        self.binaryOutput1Register.ins.velU = 1                  # [float] The estimated uncertainty (1 Sigma) in the current velocity estimate

        # Write and reset
        self.vs.writeRegister(self.binaryOutput1Register)
        self.logger().info("Configured Binary Outputs")

    def save_sensor_settings(self):
        """Saves the sensor's non-default settings to a file if configured to do so."""
        if self.config.save_settings_on_startup:
            self.logger().info(f"Saving non-default sensor settings to {self.config.settings_filename}...")
            try:
                # Based on the filename extension, determine the writer type.
                # For this implementation, we'll primarily support XML as per the params file.
                if self.config.settings_filename.endswith(".xml"):
                    configWriter = Plugins.XmlConfigWriter(self.vs, self.config.settings_filename)
                else:
                    # Default to ASCII if not XML.
                    configWriter = Plugins.AsciiConfigWriter(self.config.settings_filename)
                
                Plugins.saveNonDefaultConfiguration(self.vs, configWriter)
                self.logger().info(f"Successfully saved sensor settings to {self.config.settings_filename}.")
            except Exception as e:
                self.logger().error(f"Failed to save sensor settings: {e}")

# ==============================================================================
# The ROS 2 Node Class
# ==============================================================================
class VectorNavNode(Node):
    def __init__(self):
        super().__init__('vectornav_node')
        # --- Parameters ---
        self.declare_parameters(
            namespace='',
            parameters=[
                ('port', '/dev/ttyUSB0'),
                ('baudrate', 115200),
                ('output_rate', 20),
                ('gnss_antenna_offset', [1.0, 0.0, 0.0]),
                ('gnss_compass_baseline', [1.0, 0.0, 0.0, 0.0256, 0.0256, 0.0256]),
                ('frame_id', 'vn300'),
                ('map_frame_id', 'map'),
                ('publish_odometry', True),
                ('save_settings_on_startup', True),
                ('settings_filename', 'vectornav_settings.xml')
            ]
        )
        self.vn_config = VectorNavConfig(
            port=self.get_parameter('port').value,
            baudrate=self.get_parameter('baudrate').value,
            output_rate=self.get_parameter('output_rate').value,
            gnss_antenna_offset=self.get_parameter('gnss_antenna_offset').value,
            gnss_compass_baseline=self.get_parameter('gnss_compass_baseline').value,
            frame_id=self.get_parameter('frame_id').value,
            map_frame_id=self.get_parameter('map_frame_id').value,
            publish_odometry=self.get_parameter('publish_odometry').value,
            save_settings_on_startup=self.get_parameter('save_settings_on_startup').value,
            settings_filename=self.get_parameter('settings_filename').value
        )
        self.ned_fram_id = 'vectornav_ned'

        # --- Sensor Class ---
        self.vn_sensor = VectorNavSensor(config=self.vn_config, logger=self.get_logger)
        self.vn_sensor.connect_to_sensor()
        self.vn_sensor.configure_sensor()
        self.vn_sensor.save_sensor_settings()

        # --- Odometry State ---
        self.datum_is_set = False
        self.origin_ecef = None
        self.ecef_to_enu_matrix = None
        self.origin_lat_lon_alt = None

        # --- Publisher ---
        self.setup_publishers()

        # Set up a node to handle packet from driver
        self._thread_running = False
        self.packet_thread = Thread(target=self.packet_handling)
        self.get_logger().info("VectorNav Node initiated")

        # Start
        self.start()

    def start(self):
        self._thread_running = True
        self.packet_thread.start()
        self.get_logger().info("VectorNav Node packet handler started")

    def stop(self):
        self._thread_running = False
        if self.packet_thread.is_alive():
            self.packet_thread.join(timeout=2)

    def setup_publishers(self):
        self.startup_time_publisher = self.create_publisher(Time, "vectornav/startup_time", 10)
        self.imu_publisher = self.create_publisher(Imu, "vectornav/imu_raw", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "vectornav/mag", 10)
        self.temperature_publisher = self.create_publisher(Temperature, "vectornav/temperature", 10)
        self.pressure_publisher = self.create_publisher(FluidPressure, "vectornav/pressure", 10)
        self.gps_fix_publisher = self.create_publisher(NavSatFix, 'vectornav/gps/fix', 10)
        self.ins_vel_body_publisher = self.create_publisher(TwistWithCovarianceStamped, 'vectornav/ins/vel_body', 10)
        self.ins_vel_ned_publisher = self.create_publisher(TwistWithCovarianceStamped, 'vectornav/ins/vel_ned', 10)
        self.orientation_ned_publisher = self.create_publisher(PoseWithCovarianceStamped, 'vectornav/attitude/orientation_ned', 10)
        
        if self.vn_config.publish_odometry:
            self.odometry_publisher = self.create_publisher(Odometry, 'vectornav/odometry', 10)
            
            # Create a latched publisher for the datum
            qos_profile_transient_local = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL
            )
            self.datum_publisher = self.create_publisher(
                NavSatFix,
                'vectornav/odometry_datum',
                qos_profile=qos_profile_transient_local
            )


    def packet_handling(self):
        while self._thread_running and rclpy.ok():
            compositeData = self.vn_sensor.vs.getNextMeasurement(True)
            if compositeData is None:
                continue
            if compositeData.matchesMessage(self.vn_sensor.binaryOutput1Register):
                system_timestamp = self.get_clock().now()
                ros_timestamp = system_timestamp.to_msg()

                # -- Time --
                startup_time = Time()
                vectornav_time_startup = float(compositeData.time.timeStartup.nanoseconds())
                startup_time.sec = int(vectornav_time_startup / 1e9)
                startup_time.nanosec = int(vectornav_time_startup % 1e9)
                self.startup_time_publisher.publish(startup_time)

                # -- Temp & Pressure & MagneticField --
                temp_msg = Temperature()
                temp_msg.header.stamp = ros_timestamp
                temp_msg.header.frame_id = self.vn_config.frame_id
                temp_msg.temperature = compositeData.imu.temperature
                self.temperature_publisher.publish(temp_msg)

                pressure_msg = FluidPressure()
                pressure_msg.header.stamp = ros_timestamp
                pressure_msg.header.frame_id = self.vn_config.frame_id
                pressure_msg.fluid_pressure = compositeData.imu.pressure
                self.pressure_publisher.publish(pressure_msg)

                mag_msg = MagneticField()
                mag_msg.header.stamp = ros_timestamp
                mag_msg.header.frame_id = self.vn_config.frame_id
                mag_msg.magnetic_field.x = compositeData.imu.mag[0]
                mag_msg.magnetic_field.y = compositeData.imu.mag[1]
                mag_msg.magnetic_field.z = compositeData.imu.mag[2]
                self.mag_publisher.publish(mag_msg)

                # -- IMU --
                imu_msg = Imu()
                imu_msg.header.stamp = ros_timestamp
                imu_msg.header.frame_id = self.vn_config.frame_id
                imu_msg.angular_velocity.x = compositeData.imu.angularRate[0]
                imu_msg.angular_velocity.y = compositeData.imu.angularRate[1]
                imu_msg.angular_velocity.z = compositeData.imu.angularRate[2]
                imu_msg.linear_acceleration.x = compositeData.imu.accel[0]
                imu_msg.linear_acceleration.y = compositeData.imu.accel[1]
                imu_msg.linear_acceleration.z = compositeData.imu.accel[2]
                imu_msg.orientation.x = compositeData.attitude.quaternion.vector[0]
                imu_msg.orientation.y = compositeData.attitude.quaternion.vector[1]
                imu_msg.orientation.z = compositeData.attitude.quaternion.vector[2]
                imu_msg.orientation.w = compositeData.attitude.quaternion.scalar
                
                y_unc_rad_sq = compositeData.attitude.yprU[0]**2
                p_unc_rad_sq = compositeData.attitude.yprU[1]**2
                r_unc_rad_sq = compositeData.attitude.yprU[2]**2
                
                imu_msg.orientation_covariance[0] = r_unc_rad_sq
                imu_msg.orientation_covariance[4] = p_unc_rad_sq
                imu_msg.orientation_covariance[8] = y_unc_rad_sq
                self.imu_publisher.publish(imu_msg)

                # -- GNSS --
                gnss_msg = NavSatFix()
                gnss_msg.header.stamp = ros_timestamp
                gnss_msg.header.frame_id = self.vn_config.frame_id # LLA is frame-agnostic
                
                gnss_msg.latitude = compositeData.gnss.gnss1PosLla.lat
                gnss_msg.longitude = compositeData.gnss.gnss1PosLla.lon
                gnss_msg.altitude = compositeData.gnss.gnss1PosLla.alt
                
                fix_map = { 0: NavSatStatus.STATUS_NO_FIX, 1: NavSatStatus.STATUS_FIX, 2: NavSatStatus.STATUS_FIX, 
                            3: NavSatStatus.STATUS_FIX, 4: NavSatStatus.STATUS_SBAS_FIX, 5: NavSatStatus.STATUS_GBAS_FIX,
                            6: NavSatStatus.STATUS_GBAS_FIX}
                gnss_status = fix_map.get(compositeData.gnss.gnss1Fix, NavSatStatus.STATUS_NO_FIX)
                gnss_msg.status.status = gnss_status
                gnss_msg.status.service = NavSatStatus.SERVICE_GPS
                
                unc_n = compositeData.gnss.gnss1PosUncertainty[0]
                unc_e = compositeData.gnss.gnss1PosUncertainty[1]
                unc_d = compositeData.gnss.gnss1PosUncertainty[2]

                gnss_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
                gnss_msg.position_covariance[0] = unc_e**2
                gnss_msg.position_covariance[4] = unc_n**2
                gnss_msg.position_covariance[8] = unc_d**2
                self.gps_fix_publisher.publish(gnss_msg)

                # -- Velocity --
                ins_vel_body_msg = TwistWithCovarianceStamped()
                ins_vel_body_msg.header.stamp = ros_timestamp
                ins_vel_body_msg.header.frame_id = self.vn_config.frame_id
                ins_vel_body_msg.twist.twist.linear.x = compositeData.ins.velBody[0]
                ins_vel_body_msg.twist.twist.linear.y = compositeData.ins.velBody[1]
                ins_vel_body_msg.twist.twist.linear.z = compositeData.ins.velBody[2]
                ins_vel_unc_sq = compositeData.ins.velU**2
                ins_vel_body_msg.twist.covariance[0] = ins_vel_unc_sq
                ins_vel_body_msg.twist.covariance[7] = ins_vel_unc_sq
                ins_vel_body_msg.twist.covariance[14] = ins_vel_unc_sq
                self.ins_vel_body_publisher.publish(ins_vel_body_msg)

                ins_vel_ned_msg = TwistWithCovarianceStamped()
                ins_vel_ned_msg.header.stamp = ros_timestamp
                ins_vel_ned_msg.header.frame_id = self.ned_fram_id
                ins_vel_ned_msg.twist.twist.linear.x = compositeData.ins.velNed[0]
                ins_vel_ned_msg.twist.twist.linear.y = compositeData.ins.velNed[1]
                ins_vel_ned_msg.twist.twist.linear.z = compositeData.ins.velNed[2]
                ins_vel_ned_msg.twist.covariance[0] = ins_vel_unc_sq
                ins_vel_ned_msg.twist.covariance[7] = ins_vel_unc_sq
                ins_vel_ned_msg.twist.covariance[14] = ins_vel_unc_sq
                self.ins_vel_ned_publisher.publish(ins_vel_ned_msg)

                # -- Pose (orientation) --
                orientation_msg = PoseWithCovarianceStamped()
                orientation_msg.header.stamp = ros_timestamp
                orientation_msg.header.frame_id = self.ned_fram_id
                orientation_msg.pose.pose.orientation.x = compositeData.attitude.quaternion.vector[0]
                orientation_msg.pose.pose.orientation.y = compositeData.attitude.quaternion.vector[1]
                orientation_msg.pose.pose.orientation.z = compositeData.attitude.quaternion.vector[2]
                orientation_msg.pose.pose.orientation.w = compositeData.attitude.quaternion.scalar
                # Populate orientation covariance in the 6x6 pose covariance matrix
                orientation_msg.pose.covariance[21] = r_unc_rad_sq # Roll variance
                orientation_msg.pose.covariance[28] = p_unc_rad_sq # Pitch variance
                orientation_msg.pose.covariance[35] = y_unc_rad_sq # Yaw variance
                self.orientation_ned_publisher.publish(orientation_msg)

                # -- Odometry --
                if self.vn_config.publish_odometry:
                    self.publish_odometry(compositeData, ros_timestamp, gnss_status, gnss_msg)

    def publish_odometry(self, compositeData, ros_timestamp, gnss_status, gnss_msg):
        """Calculates and publishes the combined odometry message."""
        if not self.datum_is_set:
            if gnss_status > NavSatStatus.STATUS_NO_FIX:
                self.origin_lat_lon_alt = (
                    compositeData.ins.posLla.lat,
                    compositeData.ins.posLla.lon,
                    compositeData.ins.posLla.alt
                )
                self.origin_ecef = np.array([compositeData.ins.posEcef[0], compositeData.ins.posEcef[1], compositeData.ins.posEcef[2]])
                self.ecef_to_enu_matrix = ecef_to_enu_matrix(self.origin_lat_lon_alt[0], self.origin_lat_lon_alt[1])
                self.datum_is_set = True
                self.get_logger().info(f"Odometry datum set at LLA: {self.origin_lat_lon_alt}")
                
                # Publish the datum message
                self.datum_publisher.publish(gnss_msg)

            else:
                self.get_logger().warn("Waiting for valid GPS fix to set odometry datum...", throttle_duration_sec=5)
                return
        
        # --- Create Odometry Message ---
        odom_msg = Odometry()
        odom_msg.header.stamp = ros_timestamp
        odom_msg.header.frame_id = self.vn_config.map_frame_id
        odom_msg.child_frame_id = self.vn_config.frame_id

        # --- Pose ---
        # Position (convert current ECEF to ENU relative to datum)
        current_ecef = np.array([compositeData.ins.posEcef[0], compositeData.ins.posEcef[1], compositeData.ins.posEcef[2]])
        delta_ecef = current_ecef - self.origin_ecef
        pos_enu = self.ecef_to_enu_matrix @ delta_ecef
        odom_msg.pose.pose.position.x = pos_enu[0] # East
        odom_msg.pose.pose.position.y = pos_enu[1] # North
        odom_msg.pose.pose.position.z = pos_enu[2] # Up

        # Orientation (convert body-to-NED quaternion to body-to-ENU)
        q_body_ned = np.array([
            compositeData.attitude.quaternion.scalar,
            compositeData.attitude.quaternion.vector[0],
            compositeData.attitude.quaternion.vector[1],
            compositeData.attitude.quaternion.vector[2]
        ])
        q_ned_to_enu = np.array([0.0, 1/math.sqrt(2), 1/math.sqrt(2), 0.0]) # w,x,y,z
        q_body_enu = quaternion_multiply(q_ned_to_enu, q_body_ned)
        odom_msg.pose.pose.orientation.w = q_body_enu[0]
        odom_msg.pose.pose.orientation.x = q_body_enu[1]
        odom_msg.pose.pose.orientation.y = q_body_enu[2]
        odom_msg.pose.pose.orientation.z = q_body_enu[3]

        # Pose Covariance
        pos_unc_sq = compositeData.ins.posU**2
        odom_msg.pose.covariance[0] = pos_unc_sq  # East
        odom_msg.pose.covariance[7] = pos_unc_sq  # North
        odom_msg.pose.covariance[14] = pos_unc_sq # Up
        y_unc_rad_sq = compositeData.attitude.yprU[0]**2
        p_unc_rad_sq = compositeData.attitude.yprU[1]**2
        r_unc_rad_sq = compositeData.attitude.yprU[2]**2
        odom_msg.pose.covariance[21] = r_unc_rad_sq # Roll
        odom_msg.pose.covariance[28] = p_unc_rad_sq # Pitch
        odom_msg.pose.covariance[35] = y_unc_rad_sq # Yaw

        # --- Twist --- (in child_frame_id)
        # Linear velocity is from velBody (already in the correct frame)
        odom_msg.twist.twist.linear.x = compositeData.ins.velBody[0]
        odom_msg.twist.twist.linear.y = compositeData.ins.velBody[1]
        odom_msg.twist.twist.linear.z = compositeData.ins.velBody[2]
        
        # Angular velocity is from angularRate (already in the correct frame)
        odom_msg.twist.twist.angular.x = compositeData.imu.angularRate[0]
        odom_msg.twist.twist.angular.y = compositeData.imu.angularRate[1]
        odom_msg.twist.twist.angular.z = compositeData.imu.angularRate[2]

        # Twist Covariance
        vel_unc_sq = compositeData.ins.velU**2
        odom_msg.twist.covariance[0] = vel_unc_sq
        odom_msg.twist.covariance[7] = vel_unc_sq
        odom_msg.twist.covariance[14] = vel_unc_sq
        # Angular velocity uncertainty is not provided, so it is left as zero

        self.odometry_publisher.publish(odom_msg)

def main():
    rclpy.init()
    vectornav_node = VectorNavNode()
    try:
        rclpy.spin(vectornav_node)
    except KeyboardInterrupt:
        print('Keyboard Interrupt (SIGINT)')
    except Exception as e:
        vectornav_node.get_logger().error(f'An unexpected error occurred: {e}')
    finally:
        vectornav_node.stop()
        vectornav_node.destroy_node()
        rclpy.shutdown()
