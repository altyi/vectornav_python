import queue
import time
from threading import Thread
from dataclasses import dataclass
from typing import List
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField, Temperature, FluidPressure, NavSatFix, NavSatStatus
# from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped, PoseWithCovarianceStamped
from builtin_interfaces.msg import Time

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
            save_settings_on_startup=self.get_parameter('save_settings_on_startup').value,
            settings_filename=self.get_parameter('settings_filename').value
        )
        self.ned_fram_id = 'vectornav_ned'

        # --- Sensor Class ---
        self.vn_sensor = VectorNavSensor(config=self.vn_config, logger=self.get_logger)
        self.vn_sensor.connect_to_sensor()
        self.vn_sensor.configure_sensor()
        self.vn_sensor.save_sensor_settings()

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

    def packet_handling(self):
        while self._thread_running and rclpy.ok():
            compositeData = self.vn_sensor.vs.getNextMeasurement(block=True)
            if compositeData is None:
                continue
            if compositeData.matchesMessage(self.vn_sensor.binaryOutput1Register):
                system_timestamp = self.get_clock().now()
                ros_timestamp = system_timestamp.to_msg()

                # -- Time --
                startup_time = Time()
                startup_time.sec = int(compositeData.time.timeStartup / 1e9)
                startup_time.nanosec = int(compositeData.time.timeStartup % 1e9)
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
                imu_msg.orientation.x = compositeData.attitude.quaternion[0]
                imu_msg.orientation.y = compositeData.attitude.quaternion[1]
                imu_msg.orientation.z = compositeData.attitude.quaternion[2]
                imu_msg.orientation.w = compositeData.attitude.quaternion[3]
                
                att_unc_rad_sq = compositeData.attitude.attU**2
                imu_msg.orientation_covariance[0] = att_unc_rad_sq
                imu_msg.orientation_covariance[4] = att_unc_rad_sq
                imu_msg.orientation_covariance[8] = att_unc_rad_sq
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
                gnss_msg.status.status = fix_map.get(compositeData.gnss.gnss1Fix, NavSatStatus.STATUS_NO_FIX)
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
                orientation_msg.pose.pose.orientation.x = compositeData.attitude.quaternion[0]
                orientation_msg.pose.pose.orientation.y = compositeData.attitude.quaternion[1]
                orientation_msg.pose.pose.orientation.z = compositeData.attitude.quaternion[2]
                orientation_msg.pose.pose.orientation.w = compositeData.attitude.quaternion[3]
                # Populate orientation covariance in the 6x6 pose covariance matrix
                orientation_msg.pose.covariance[21] = att_unc_rad_sq # Roll variance
                orientation_msg.pose.covariance[28] = att_unc_rad_sq # Pitch variance
                orientation_msg.pose.covariance[35] = att_unc_rad_sq # Yaw variance
                self.orientation_ned_publisher.publish(orientation_msg)

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
