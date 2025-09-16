# VectorNav SDK Registers Documentation

This document details all the registers available in the VectorNav Python SDK, organized by category. These registers are used to read measurement data from the sensor and to configure its various settings.

Below is a quick reference table that links to the detailed description of each register. The registers are divided into two types: Measurement registers, which are read-only and provide sensor data, and Configuration registers, which are read/write and control the sensor's behavior.

### Measurement Registers
[Detailed Measurment Registers Section](#measurement-registers-1)

| Register Type | Register Name | Register ID |
| :--- | :--- | :--- |
| [**Attitude**](#attitude) | [YawPitchRoll](#yawpitchroll) | 8 |
| | [Quaternion](#quaternion) | 9 |
| | [QuatMagAccelRate](#quatmagaccelrate) | 15 |
| | [YprMagAccelAngularRates](#yprmagaccelangularrates) | 27 |
| | [YprLinearBodyAccelAngularRates](#yprlinearbodyaccelangularrates) | 239 |
| | [YprLinearInertialAccelAngularRates](#yprlinearinertialaccelangularrates) | 240 |
| [**GNSS**](#gnss) | [GnssSolLla](#gnsssollla) | 58 |
| | [GnssSolEcef](#gnssolecef) | 59 |
| | [Gnss2SolLla](#gnss2sollla) | 103 |
| | [Gnss2SolEcef](#gnss2solecef) | 104 |
| [**GNSSCompass**](#gnsscompass) | [GnssCompassSignalHealthStatus](#gnsscompasssignalhealthstatus) | 86 |
| | [GnssCompassEstBaseline](#gnsscompassestbaseline) | 97 |
| | [GnssCompassStartupStatus](#gnsscompassstartupstatus) | 98 |
| [**HardSoftIronEstimator**](#hardsoftironestimator) | [EstMagCal](#estmagcal) | 47 |
| [**Heave**](#heave) | [HeaveOutputs](#heaveoutputs) | 115 |
| [**IMU**](#imu) | [Mag](#mag) | 17 |
| | [Accel](#accel) | 18 |
| | [Gyro](#gyro) | 19 |
| | [MagAccelGyro](#magaccelgyro) | 20 |
| | [ImuMeas](#imumeas) | 54 |
| | [DeltaThetaVelocity](#deltathetavelocity) | 80 |
| [**INS**](#ins) | [InsSolLla](#inssollla) | 63 |
| | [InsSolEcef](#inssolecef) | 64 |
| | [InsStateLla](#insstatella) | 72 |
| | [InsStateEcef](#insstateecef) | 73 |
| [**System**](#system) | [Model](#model) | 1 |
| | [HwVer](#hwver) | 2 |
| | [Serial](#serial) | 3 |
| | [FwVer](#fwver) | 4 |
| | [SyncStatus](#syncstatus) | 33 |

### Configuration Registers
[Detailed Configuration Registers Section](#configuration-registers-1)

| Register Type | Register Name | Register ID |
| :--- | :--- | :--- |
| [**Attitude**](#attitude-1) | [MagGravRefVec](#maggravrefvec) | 21 |
| | [VpeBasicControl](#vpebasiccontrol) | 35 |
| | [VpeMagBasicTuning](#vpemagbasictuning) | 36 |
| | [VpeAccelBasicTuning](#vpeaccelbasictuning) | 38 |
| [**GNSS**](#gnss-1) | [GnssBasicConfig](#gnssbasicconfig) | 55 |
| | [GnssAOffset](#gnssaoffset) | 57 |
| | [GnssSystemConfig](#gnsssystemconfig) | 99 |
| | [GnssSyncConfig](#gnsssyncconfig) | 100 |
| | [ExtGnssOffset](#extgnssoffset) | 157 |
| [**GNSSCompass**](#gnsscompass-1) | [GnssCompassBaseline](#gnsscompassbaseline) | 93 |
| [**HardSoftIronEstimator**](#hardsoftironestimator-1) | [RealTimeHsiControl](#realtimehsicontrol) | 44 |
| [**Heave**](#heave-1) | [HeaveBasicConfig](#heavebasicconfig) | 116 |
| [**IMU**](#imu-1) | [MagCal](#magcal) | 23 |
| | [AccelCal](#accelcal) | 25 |
| | [RefFrameRot](#refframerot) | 26 |
| | [DeltaThetaVelConfig](#deltathetavelconfig) | 82 |
| | [GyroCal](#gyrocal) | 84 |
| | [ImuFilterControl](#imufiltercontrol) | 85 |
| [**INS**](#ins-1) | [InsBasicConfig](#insbasicconfig) | 67 |
| | [FilterStartupBias](#filterstartupbias) | 74 |
| | [InsRefOffset](#insrefoffset) | 105 |
| | [InsGnssSelect](#insgnssselect) | 144 |
| [**System**](#system-1) | [UserTag](#usertag) | 0 |
| | [BaudRate](#baudrate) | 5 |
| | [AsyncOutputType](#asyncoutputtype) | 6 |
| | [AsyncOutputFreq](#asyncoutputfreq) | 7 |
| | [ProtocolControl](#protocolcontrol) | 30 |
| | [SyncControl](#synccontrol) | 32 |
| | [BinaryOutput1](#binaryoutput1) | 75 |
| | [BinaryOutput2](#binaryoutput2) | 76 |
| | [BinaryOutput3](#binaryoutput3) | 77 |
| | [NmeaOutput1](#nmeaoutput1) | 101 |
| | [NmeaOutput2](#nmeaoutput2) | 102 |
| | [LegacyCompatibilitySettings](#legacycompatibilitysettings) | 206 |
| [**VelocityAiding**](#velocityaiding) | [VelAidingMeas](#velaidingmeas) | 50 |
| | [VelAidingControl](#velaidingcontrol) | 51 |
| [**WorldMagGravityModel**](#worldmaggravitymodel) | [RefModelConfig](#refmodelconfig) | 83 |

---

## Measurement Registers

> This section provides detailed information about each read-only measurement register, which are used to retrieve data from the sensor.

### Attitude

> ### YawPitchRoll
> > Attitude solution as yaw, pitch, and roll in degrees. The yaw, pitch, and roll is given as a 3,2,1 Euler angle rotation sequence describing the orientation of the sensor with respect to the inertial North East Down (NED) frame.
>
> **ID:** 8 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `yaw`     | `float` | Yaw angle   |
> | `pitch`   | `float` | Pitch angle |
> | `roll`    | `float` | Roll angle  |
>
> ### Quaternion
> > Attitude solution as a quaternion.
>
> **ID:** 9 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                       |
> |:----------|:--------|:----------------------------------|
> | `quatX`   | `float` | First vector component of quaternion |
> | `quatY`   | `float` | Second vector component of quaternion|
> | `quatZ`   | `float` | Third vector component of quaternion |
> | `quatS`   | `float` | Scalar component of quaternion     |
>
> ### QuatMagAccelRate
> > Quaternion attitude solution, and compensated (Magnetic, Acceleration, Angular Rate) values.
>
> **ID:** 15 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `quatX`   | `float` | First vector component of quaternion                      |
> | `quatY`   | `float` | Second vector component of quaternion                     |
> | `quatZ`   | `float` | Third vector component of quaternion                      |
> | `quatS`   | `float` | Scalar component of quaternion                              |
> | `magX`    | `float` | Compensated magnetometer measurement in the body-frame x-axis |
> | `magY`    | `float` | Compensated magnetometer measurement in the body-frame y-axis |
> | `magZ`    | `float` | Compensated magnetometer measurement in the body-frame z-axis |
> | `accelX`  | `float` | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float` | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float` | Compensated accelerometer measurement in the body-frame z-axis|
> | `gyroX`   | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float` | Compensated angular rate measurement in the body-frame z-axis |
>
> ### YprMagAccelAngularRates
> > Yaw, Pitch, Roll, Accel, and Angular Rates
>
> **ID:** 27 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `yaw`     | `float` | Yaw angle                                                   |
> | `pitch`   | `float` | Pitch angle                                                 |
> | `roll`    | `float` | Roll angle                                                  |
> | `magX`    | `float` | Compensated magnetometer measurement in the body-frame x-axis |
> | `magY`    | `float` | Compensated magnetometer measurement in the body-frame y-axis |
> | `magZ`    | `float` | Compensated magnetometer measurement in the body-frame z-axis |
> | `accelX`  | `float` | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float` | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float` | Compensated accelerometer measurement in the body-frame z-axis|
> | `gyroX`   | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float` | Compensated angular rate measurement in the body-frame z-axis |
>
> ### YprLinearBodyAccelAngularRates
> > Yaw, Pitch, Roll, Linear Body Accel, and Angular Rates.
>
> **ID:** 239 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `yaw`       | `float` | Yaw angle                                                   |
> | `pitch`     | `float` | Pitch angle                                                 |
> | `roll`      | `float` | Roll angle                                                  |
> | `linAccelX` | `float` | Linear acceleration in body-frame x-axis                  |
> | `linAccelY` | `float` | Linear acceleration in body-frame y-axis                  |
> | `linAccelZ` | `float` | Linear acceleration in body-frame z-axis                  |
> | `gyroX`     | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`     | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`     | `float` | Compensated angular rate measurement in the body-frame z-axis |
>
> ### YprLinearInertialAccelAngularRates
> > Yaw, Pitch, Roll, Linear Inertial Accel, and Angular Rates.
>
> **ID:** 240 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `yaw`       | `float` | Yaw angle                                                   |
> | `pitch`     | `float` | Pitch angle                                                 |
> | `roll`      | `float` | Roll angle                                                  |
> | `linAccelN` | `float` | Linear acceleration in North direction                    |
> | `linAccelE` | `float` | Linear acceleration in East direction                     |
> | `linAccelD` | `float` | Linear acceleration in Down direction                     |
> | `gyroX`     | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`     | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`     | `float` | Compensated angular rate measurement in the body-frame z-axis |

---
### GNSS

> ### GnssSolLla
> > Primary GNSS receiver measurement with lat/lon/alt position and velocity in NED frame.
>
> **ID:** 58 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute             | Type                  | Description                                   |
> |:----------------------|:----------------------|:----------------------------------------------|
> | `gps1Tow`             | `double`              | GPS time of week                              |
> | `gps1Week`            | `uint16_t`            | The current GPS week                          |
> | `gnss1Fix`            | `GnssSolLla.Gnss1Fix` | Type of GNSS fix                              |
> | `gnss1NumSats`        | `uint8_t`             | Number of satellites tracked by GNSS receiver |
> | `lat`                 | `double`              | GNSS geodetic latitude                        |
> | `lon`                 | `double`              | GNSS longitude                                |
> | `alt`                 | `double`              | GNSS altitude above WGS84 ellipsoid           |
> | `velN`                | `float`               | GNSS velocity in North direction              |
> | `velE`                | `float`               | GNSS velocity in East direction               |
> | `velD`                | `float`               | GNSS velocity in Down direction               |
> | `posUncertaintyN`     | `float`               | GNSS position uncertainty, North direction    |
> | `posUncertaintyE`     | `float`               | GNSS position uncertainty, East direction     |
> | `posUncertaintyD`     | `float`               | GNSS position uncertainty, Down direction     |
> | `gnss1VelUncertainty` | `float`               | GNSS velocity uncertainty (scalar)            |
> | `gnss1TimeUncertainty`| `float`               | GNSS time uncertainty                         |
>
> **Nested Enums**
>
> **`Gnss1Fix`**
>
> | Value      |
> |:-----------|
> | `NoFix`    |
> | `TimeFix`  |
> | `Fix2D`    |
> | `Fix3D`    |
> | `SBAS`     |
> | `RtkFloat` |
> | `RtkFix`   |
>
> ### GnssSolEcef
> > Primary GNSS receiver measurement in ECEF frame.
>
> **ID:** 59 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute             | Type                   | Description                                   |
> |:----------------------|:-----------------------|:----------------------------------------------|
> | `gps1Tow`             | `double`               | GPS time of week                              |
> | `gps1Week`            | `uint16_t`             | The current GPS week                          |
> | `gnss1Fix`            | `GnssSolEcef.Gnss1Fix` | Type of GNSS fix                              |
> | `gnss1NumSats`        | `uint8_t`              | Number of satellites tracked by GNSS receiver |
> | `posX`                | `double`               | GNSS position in ECEF-frame x-axis            |
> | `posY`                | `double`               | GNSS position in ECEF-frame y-axis            |
> | `posZ`                | `double`               | GNSS position in ECEF-frame z-axis            |
> | `velX`                | `float`                | GNSS velocity in ECEF-frame x-axis            |
> | `velY`                | `float`                | GNSS velocity in ECEF-frame y-axis            |
> | `velZ`                | `float`                | GNSS velocity in ECEF-frame z-axis            |
> | `posUncertaintyX`     | `float`                | GNSS position uncertainty ECEF X              |
> | `posUncertaintyY`     | `float`                | GNSS position uncertainty ECEF Y              |
> | `posUncertaintyZ`     | `float`                | GNSS position uncertainty ECEF Z              |
> | `gnss1VelUncertainty` | `float`                | GNSS velocity uncertainty (scalar)            |
> | `gnss1TimeUncertainty`| `float`                | GNSS time uncertainty                         |
>
> **Nested Enums**
>
> **`Gnss1Fix`**
>
> | Value      |
> |:-----------|
> | `NoFix`    |
> | `TimeFix`  |
> | `Fix2D`    |
> | `Fix3D`    |
> | `SBAS`     |
> | `RtkFloat` |
> | `RtkFix`   |
>
> ### Gnss2SolLla
> > Estimated GNSS 2 Solution with lat/lon/alt position. This register is deprecated and will be removed in future firmware versions.
>
> **ID:** 103 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute             | Type                   | Description                                   |
> |:----------------------|:-----------------------|:----------------------------------------------|
> | `gps2Tow`             | `double`               | GPS time of week                              |
> | `gps2Week`            | `uint16_t`             | The current GPS week                          |
> | `gnss2Fix`            | `Gnss2SolLla.Gnss2Fix` | Type of GNSS fix                              |
> | `gnss2NumSats`        | `uint8_t`              | Number of satellites tracked by GNSS receiver |
> | `lat`                 | `double`               | GNSS geodetic latitude                        |
> | `lon`                 | `double`               | GNSS longitude                                |
> | `alt`                 | `double`               | GNSS altitude above WGS84 ellipsoid           |
> | `velN`                | `float`                | GNSS velocity in North direction              |
> | `velE`                | `float`                | GNSS velocity in East direction               |
> | `velD`                | `float`                | GNSS velocity in Down direction               |
> | `posUncertaintyN`     | `float`                | GNSS position uncertainty, North direction    |
> | `posUncertaintyE`     | `float`                | GNSS position uncertainty, East direction     |
> | `posUncertaintyD`     | `float`                | GNSS position uncertainty, Down direction     |
> | `gnss2VelUncertainty` | `float`                | GNSS velocity uncertainty (scalar)            |
> | `gnss2TimeUncertainty`| `float`                | GNSS time uncertainty                         |
>
> **Nested Enums**
>
> **`Gnss2Fix`**
>
> | Value      |
> |:-----------|
> | `NoFix`    |
> | `TimeFix`  |
> | `Fix2D`    |
> | `Fix3D`    |
> | `SBAS`     |
> | `RtkFloat` |
> | `RtkFix`   |
>
> ### Gnss2SolEcef
> > Estimated GNSS 2 Solution with ECEF position. This register is deprecated and will be removed in future firmware versions.
>
> **ID:** 104 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute             | Type                    | Description                                   |
> |:----------------------|:------------------------|:----------------------------------------------|
> | `gps2Tow`             | `double`                | GPS time of week                              |
> | `gps2Week`            | `uint16_t`              | The current GPS week                          |
> | `gnss2Fix`            | `Gnss2SolEcef.Gnss2Fix` | Type of GNSS fix                              |
> | `gnss2NumSats`        | `uint8_t`               | Number of satellites tracked by GNSS receiver |
> | `posX`                | `double`                | GNSS position in ECEF-frame x-axis            |
> | `posY`                | `double`                | GNSS position in ECEF-frame y-axis            |
> | `posZ`                | `double`                | GNSS position in ECEF-frame z-axis            |
> | `velX`                | `float`                 | GNSS velocity in ECEF-frame x-axis            |
> | `velY`                | `float`                 | GNSS velocity in ECEF-frame y-axis            |
> | `velZ`                | `float`                 | GNSS velocity in ECEF-frame z-axis            |
> | `posUncertaintyX`     | `float`                 | GNSS position uncertainty ECEF X              |
> | `posUncertaintyY`     | `float`                 | GNSS position uncertainty ECEF Y              |
> | `posUncertaintyZ`     | `float`                 | GNSS position uncertainty ECEF Z              |
> | `gnss2VelUncertainty` | `float`                 | GNSS velocity uncertainty (scalar)            |
> | `gnss2TimeUncertainty`| `float`                 | GNSS time uncertainty                         |
>
> **Nested Enums**
>
> **`Gnss2Fix`**
>
> | Value      |
> |:-----------|
> | `NoFix`    |
> | `TimeFix`  |
> | `Fix2D`    |
> | `Fix3D`    |
> | `SBAS`     |
> | `RtkFloat` |
> | `RtkFix`   |

---
### GNSSCompass

> ### GnssCompassSignalHealthStatus
> > Provides several indicators that serve as an overall health status of the GNSS compass subsystem.
>
> **ID:** 86 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute       | Type    | Description                                                                                         |
> |:----------------|:--------|:----------------------------------------------------------------------------------------------------|
> | `numSatsPvtA`   | `float` | Number of satellites available for PVT solution for receiver A                                      |
> | `numSatsRtkA`   | `float` | Number of satellites available for RTK solution for receiver A                                      |
> | `highestCn0A`   | `float` | Highest CN0 reported on receiver A                                                                  |
> | `numSatsPvtB`   | `float` | Number of satellites available for the PVT solution of receiver B                                   |
> | `numSatsRtkB`   | `float` | Number of satellites available for the RTK solution of receiver B                                   |
> | `highestCn0B`   | `float` | Highest CN0 reported on receiver B                                                                  |
> | `numComSatsPvt` | `float` | The number of common satellites that are used in the PVT solutions of both receiver A and receiver B  |
> | `numComSatsRtk` | `float` | The number of common satellites that are used in the RTK solutions of both receiver A and receiver B  |
>
> ### GnssCompassEstBaseline
> > Provides the estimated GNSS compass baseline measurement. The estimated position offset and measurement uncertainty is for antenna B relative to antenna A in the body reference frame.
>
> **ID:** 97 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute           | Type       | Description                                                                        |
> |:--------------------|:-----------|:-----------------------------------------------------------------------------------|
> | `estBaselineComplete` | `uint8_t`  | Set to 1 when baseline estimation has complete                                     |
> | `resv`                | `uint8_t`  | Reserved                                                                           |
> | `numMeas`             | `uint16_t` | Number of measurements used by the estimated solution                              |
> | `positionX`           | `float`    | Position of GNSS antenna B with respect to antenna A in the body-frame x-axis    |
> | `positionY`           | `float`    | Position of GNSS antenna B with respect to antenna A in the body-frame y-axis    |
> | `positionZ`           | `float`    | Position of GNSS antenna B with respect to antenna A in the body-frame z-axis    |
> | `uncertaintyX`        | `float`    | Uncertainty in the x-axis position measurement                                     |
> | `uncertaintyY`        | `float`    | Uncertainty in the y-axis position measurement                                     |
> | `uncertaintyZ`        | `float`    | Uncertainty in the z-axis position measurement                                     |
>
> ### GnssCompassStartupStatus
> > Provides status information on the GNSS compass startup process.
>
> **ID:** 98 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute       | Type      | Description                                     |
> |:----------------|:----------|:------------------------------------------------|
> | `percentComplete` | `uint8_t` | The estimated startup process completion percent|
> | `currentHeading`  | `float`   | The current GNSS compass heading estimate       |

---
### HardSoftIronEstimator

> ### EstMagCal
> > Magnetometer calibration values calculated by the real-time HSI calibration filter.
>
> **ID:** 47 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                             |
> |:----------|:--------|:--------------------------------------------------------|
> | `magGain00` | `float` | Magnetometer calibration gain matrix, row 0, colum 0  |
> | `magGain01` | `float` | Magnetometer calibration gain matrix, row 0, colum 1  |
> | `magGain02` | `float` | Magnetometer calibration gain matrix, row 0, colum 2  |
> | `magGain10` | `float` | Magnetometer calibration gain matrix, row 1, colum 0  |
> | `magGain11` | `float` | Magnetometer calibration gain matrix, row 1, colum 1  |
> | `magGain12` | `float` | Magnetometer calibration gain matrix, row 1, colum 2  |
> | `magGain20` | `float` | Magnetometer calibration gain matrix, row 2, colum 0  |
> | `magGain21` | `float` | Magnetometer calibration gain matrix, row 2, colum 1  |
> | `magGain22` | `float` | Magnetometer calibration gain matrix, row 2, colum 2  |
> | `magBiasX`  | `float` | Magnetometer bias calibration, sensor-frame x-axis    |
> | `magBiasY`  | `float` | Magnetometer bias calibration, sensor-frame y-axis    |
> | `magBiasZ`  | `float` | Magnetometer bias calibration, sensor-frame z-axis    |

---
### Heave

> ### HeaveOutputs
> > Real-time heave and heave-rate estimates, plus a delayed-heave estimate.
>
> **ID:** 115 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute      | Type    | Description                                                               |
> |:---------------|:--------|:--------------------------------------------------------------------------|
> | `heave`        | `float` | Real-time heave estimate                                                  |
> | `heaveRate`    | `float` | Real-time heave rate estimate                                             |
> | `delayedHeave` | `float` | Delayed heave. Higher accuracy than real-time heave, but 50 seconds old |

---
### IMU

> ### Mag
> > Compensated magnetometer measurements.
>
> **ID:** 17 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `magX`    | `float` | Compensated magnetometer measurement in the body-frame x-axis |
> | `magY`    | `float` | Compensated magnetometer measurement in the body-frame y-axis |
> | `magZ`    | `float` | Compensated magnetometer measurement in the body-frame z-axis |
>
> ### Accel
> > Compensated acceleration measurements
>
> **ID:** 18 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `accelX`  | `float` | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float` | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float` | Compensated accelerometer measurement in the body-frame z-axis|
>
> ### Gyro
> > Compensated angular rate measurements.
>
> **ID:** 19 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `gyroX`   | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float` | Compensated angular rate measurement in the body-frame z-axis |
>
> ### MagAccelGyro
> > Compensated magnetic, acceleration, and angular rate measurements.
>
> **ID:** 20 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type    | Description                                                 |
> |:----------|:--------|:------------------------------------------------------------|
> | `magX`    | `float` | Compensated magnetometer measurement in the body-frame x-axis |
> | `magY`    | `float` | Compensated magnetometer measurement in the body-frame y-axis |
> | `magZ`    | `float` | Compensated magnetometer measurement in the body-frame z-axis |
> | `accelX`  | `float` | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float` | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float` | Compensated accelerometer measurement in the body-frame z-axis|
> | `gyroX`   | `float` | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float` | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float` | Compensated angular rate measurement in the body-frame z-axis |
>
> ### ImuMeas
> > Provides the calibrated IMU measurements including barometric pressure.
>
> **ID:** 54 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute   | Type    | Description                     |
> |:------------|:--------|:--------------------------------|
> | `uncompMagX`  | `float` | Magnetometer body-frame x-axis  |
> | `uncompMagY`  | `float` | Magnetometer body-frame y-axis  |
> | `uncompMagZ`  | `float` | Magnetometer body-frame z-axis  |
> | `uncompAccX`  | `float` | Accelerometer body-frame x-axis |
> | `uncompAccY`  | `float` | Accelerometer body-frame y-axis |
> | `uncompAccZ`  | `float` | Accelerometer body-frame z-axis |
> | `uncompGyroX` | `float` | Angular rate body-frame x-axis    |
> | `uncompGyroY` | `float` | Angular rate body-frame y-axis    |
> | `uncompGyroZ` | `float` | Angular rate body-frame z-axis    |
> | `temperature` | `float` | Sensor temperature              |
> | `pressure`    | `float` | Barometric pressure             |
>
> ### DeltaThetaVelocity
> > This register contains the output values of the onboard coning and sculling algorithm.
>
> **ID:** 80 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute   | Type    | Description                         |
> |:------------|:--------|:------------------------------------|
> | `deltaTime`   | `float` | Duration of integration interval    |
> | `deltaThetaX` | `float` | Integrated rotation vector x-axis   |
> | `deltaThetaY` | `float` | Integrated rotation vector y-axis   |
> | `deltaThetaZ` | `float` | Integrated rotation vector z-axis   |
> | `deltaVelX`   | `float` | Integrated velocity x-axis          |
> | `deltaVelY`   | `float` | Integrated velocity y-axis          |
> | `deltaVelZ`   | `float` | Integrated velocity z-axis          |

---
### INS

> ### InsSolLla
> > Estimated INS solution with lat/lon/alt position.
>
> **ID:** 63 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute       | Type       | Description                             |
> |:----------------|:-----------|:----------------------------------------|
> | `timeGpsTow`    | `double`   | GPS time of week                        |
> | `timeGpsWeek`   | `uint16_t` | The current GPS week                    |
> | `insStatus`     | `uint16_t` | Ins status bitfield                     |
> | `yaw`           | `float`    | Yaw angle                               |
> | `pitch`         | `float`    | Pitch angle                             |
> | `roll`          | `float`    | Roll angle                              |
> | `posLat`        | `double`   | Geodetic latitude                       |
> | `posLon`        | `double`   | Longitude                               |
> | `posAlt`        | `double`   | Altitude above WGS84 ellipsoid          |
> | `velN`          | `float`    | Velocity in North direction             |
> | `velE`          | `float`    | Velocity in East direction              |
> | `velD`          | `float`    | Velocity in Down direction              |
> | `attUncertainty`| `float`    | Filter estimated attitude uncertainty |
> | `posUncertainty`| `float`    | Filter estimated position uncertainty   |
> | `velUncertainty`| `float`    | Filter estimated velocity uncertainty   |
>
> **Nested Bitfields**
>
> **`InsStatus`**
>
> | Field            | Description                                                                                         |
> |:-----------------|:----------------------------------------------------------------------------------------------------|
> | `mode`           | Two-bit enumeration that indicates the current mode of the INS filter                                 |
> | `gnssFix`        | Indicates whether the GNSS has a valid fix                                                          |
> | `resv1`          | Reserved                                                                                            |
> | `imuErr`         | High if gyro or accelerometer subsystem error is detected                                           |
> | `magPresErr`     | High if magnetometer or pressure subsystem error is detected                                        |
> | `gnssErr`        | High if GNSS communication error is detected or if no valid PPS signal is received                  |
> | `resv2`          | Reserved for internal use. May toggle state during runtime and should be ignored                  |
> | `gnssCompassFix` | Two-bit enumeration that indicates whether the GNSS compass is operational and aiding the INS |
>
> ### InsSolEcef
> > Estimated INS Solution with ECEF position
>
> **ID:** 64 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute       | Type       | Description                             |
> |:----------------|:-----------|:----------------------------------------|
> | `timeGpsTow`    | `double`   | GPS time of week                        |
> | `timeGpsWeek`   | `uint16_t` | The current GPS week                    |
> | `insStatus`     | `uint16_t` | Ins status bitfield                     |
> | `yaw`           | `float`    | Yaw angle                               |
> | `pitch`         | `float`    | Pitch angle                             |
> | `roll`          | `float`    | Roll angle                              |
> | `posEx`         | `double`   | Position in ECEF-frame x-axis           |
> | `posEy`         | `double`   | Position in ECEF-frame y-axis           |
> | `posEz`         | `double`   | Position in ECEF-frame z-axis           |
> | `velEx`         | `float`    | Velocity in ECEF-frame x-axis           |
> | `velEy`         | `float`    | Velocity in ECEF-frame y-axis           |
> | `velEz`         | `float`    | Velocity in ECEF-frame z-axis           |
> | `attUncertainty`| `float`    | Filter estimated attitude uncertainty |
> | `posUncertainty`| `float`    | Filter estimated position uncertainty   |
> | `velUncertainty`| `float`    | Filter estimated velocity uncertainty   |
>
> **Nested Bitfields**
>
> **`InsStatus`**
>
> | Field            | Description                                                                                         |
> |:-----------------|:----------------------------------------------------------------------------------------------------|
> | `mode`           | Two-bit enumeration that indicates the current mode of the INS filter                                 |
> | `gnssFix`        | Indicates whether the GNSS has a valid fix                                                          |
> | `resv1`          | Reserved                                                                                            |
> | `imuErr`         | High if gyro or accelerometer subsystem error is detected                                           |
> | `magPresErr`     | High if magnetometer or pressure subsystem error is detected                                        |
> | `gnssErr`        | High if GNSS communication error is detected or if no valid PPS signal is received                  |
> | `resv2`          | Reserved for internal use. May toggle state during runtime and should be ignored                  |
> | `gnssCompassFix` | Two-bit enumeration that indicates whether the GNSS compass is operational and aiding the INS |
>
> ### InsStateLla
> > Estimated INS state with lat/lon/alt position.
>
> **ID:** 72 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type     | Description                                                 |
> |:----------|:---------|:------------------------------------------------------------|
> | `yaw`     | `float`  | Yaw angle                                                   |
> | `pitch`   | `float`  | Pitch angle                                                 |
> | `roll`    | `float`  | Roll angle                                                  |
> | `posLat`  | `double` | Geodetic latitude                                           |
> | `posLon`  | `double` | Longitude                                                   |
> | `posAlt`  | `double` | Altitude above WGS84 ellipsoid                              |
> | `velN`    | `float`  | Velocity in North direction                                 |
> | `velE`    | `float`  | Velocity in East direction                                  |
> | `velD`    | `float`  | Velocity in Down direction                                  |
> | `accelX`  | `float`  | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float`  | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float`  | Compensated accelerometer measurement in the body-frame z-axis|
> | `gyroX`   | `float`  | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float`  | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float`  | Compensated angular rate measurement in the body-frame z-axis |
>
> ### InsStateEcef
> > Estimated INS state with ECEF position.
>
> **ID:** 73 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type     | Description                                                 |
> |:----------|:---------|:------------------------------------------------------------|
> | `yaw`     | `float`  | Yaw angle                                                   |
> | `pitch`   | `float`  | Pitch angle                                                 |
> | `roll`    | `float`  | Roll angle                                                  |
> | `posEx`   | `double` | Position in ECEF-frame x-axis                               |
> | `posEy`   | `double` | Position in ECEF-frame y-axis                               |
> | `posEz`   | `double` | Position in ECEF-frame z-axis                               |
> | `velEx`   | `float`  | Velocity in ECEF-frame x-axis                               |
> | `velEy`   | `float`  | Velocity in ECEF-frame y-axis                               |
> | `velEz`   | `float`  | Velocity in ECEF-frame z-axis                               |
> | `accelX`  | `float`  | Compensated accelerometer measurement in the body-frame x-axis|
> | `accelY`  | `float`  | Compensated accelerometer measurement in the body-frame y-axis|
> | `accelZ`  | `float`  | Compensated accelerometer measurement in the body-frame z-axis|
> | `gyroX`   | `float`  | Compensated angular rate measurement in the body-frame x-axis |
> | `gyroY`   | `float`  | Compensated angular rate measurement in the body-frame y-axis |
> | `gyroZ`   | `float`  | Compensated angular rate measurement in the body-frame z-axis |

---
### System

> ### Model
> > Product model string.
>
> **ID:** 1 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type   | Description                                   |
> |:----------|:-------|:----------------------------------------------|
> | `model`   | `char` | Product model number, maximum length 24 characters |
>
> ### HwVer
> > Hardware version number.
>
> **ID:** 2 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute  | Type       | Description                  |
> |:-----------|:-----------|:-----------------------------|
> | `hwVer`    | `uint32_t` | Hardware version number      |
> | `hwMinVer` | `uint32_t` | Hardware minor version number|
>
> ### Serial
> > Device serial number.
>
> **ID:** 3 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type       | Description               |
> |:----------|:-----------|:--------------------------|
> | `serialNum` | `uint32_t` | The unit's serial number|
>
> ### FwVer
> > Firmware version number.
>
> **ID:** 4 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute | Type   | Description        |
> |:----------|:-------|:-------------------|
> | `fwVer`   | `char` | Firmware version |
>
> ### SyncStatus
> > Contains counters based on the SyncIn and SyncOut events.
>
> **ID:** 33 | **Type:** Measurement
> ***
> **Attributes**
>
> | Attribute    | Type       | Description                                                 |
> |:-------------|:-----------|:------------------------------------------------------------|
> | `syncInCount`  | `uint32_t` | Amount of SyncIn Events that have occurred                |
> | `syncInTime`   | `uint32_t` | The amount of time that has elapsed since the last SyncIn Event |
> | `syncOutCount` | `uint32_t` | Keeps track of the number of times that a SyncOut pulse has occurred |

---
## Configuration Registers

> This section provides detailed information about each read/write configuration register, which are used to control the sensor's behavior.

### Attitude

> ### MagGravRefVec
> >
>
> **ID:** 21 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `magRefN`   | `float` |             |
> | `magRefE`   | `float` |             |
> | `magRefD`   | `float` |             |
> | `gravRefN`  | `float` |             |
> | `gravRefE`  | `float` |             |
> | `gravRefD`  | `float` |             |
>
> ### VpeBasicControl
> >
>
> **ID:** 35 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute     | Type                          | Description |
> |:--------------|:------------------------------|:------------|
> | `resv`          | `uint8_t`                     |             |
> | `headingMode`   | `VpeBasicControl.HeadingMode` |             |
> | `filteringMode` | `VpeBasicControl.FilteringMode` |             |
> | `tuningMode`    | `VpeBasicControl.TuningMode`  |             |
>
> **Nested Enums**
>
> **`HeadingMode`**
>
> | Value      |
> |:-----------|
> | `Absolute` |
> | `Relative` |
> | `Indoor`   |
>
> **`FilteringMode`**
>
> | Value                |
> |:---------------------|
> | `Unfiltered`         |
> | `AdaptivelyFiltered` |
>
> **`TuningMode`**
>
> | Value      |
> |:-----------|
> | `Static`   |
> | `Adaptive` |
>
> ### VpeMagBasicTuning
> >
>
> **ID:** 36 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute          | Type    | Description |
> |:-------------------|:--------|:------------|
> | `baseTuningX`      | `float` |             |
> | `baseTuningY`      | `float` |             |
> | `baseTuningZ`      | `float` |             |
> | `adaptiveTuningX`  | `float` |             |
> | `adaptiveTuningY`  | `float` |             |
> | `adaptiveTuningZ`  | `float` |             |
> | `adaptiveFilteringX` | `float` |             |
> | `adaptiveFilteringY` | `float` |             |
> | `adaptiveFilteringZ` | `float` |             |
>
> ### VpeAccelBasicTuning
> >
>
> **ID:** 38 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute          | Type    | Description |
> |:-------------------|:--------|:------------|
> | `baseTuningX`      | `float` |             |
> | `baseTuningY`      | `float` |             |
> | `baseTuningZ`      | `float` |             |
> | `adaptiveTuningX`  | `float` |             |
> | `adaptiveTuningY`  | `float` |             |
> | `adaptiveTuningZ`  | `float` |             |
> | `adaptiveFilteringX` | `float` |             |
> | `adaptiveFilteringY` | `float` |             |
> | `adaptiveFilteringZ` | `float` |             |

---
### GNSS

> ### GnssBasicConfig
> >
>
> **ID:** 55 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute      | Type                           | Description |
> |:---------------|:-------------------------------|:------------|
> | `receiverEnable` | `GnssBasicConfig.ReceiverEnable` |             |
> | `ppsSource`      | `GnssBasicConfig.PpsSource`      |             |
> | `rate`           | `GnssBasicConfig.Rate`           |             |
> | `resv4`          | `uint8_t`                      |             |
> | `antPower`       | `GnssBasicConfig.AntPower`       |             |
>
> **Nested Enums**
>
> **`ReceiverEnable`**
>
> | Value              |
> |:-------------------|
> | `Internal`         |
> | `VnWrite`          |
> | `VnAdor`           |
> | `GnssA`            |
> | `VnWriteAndGnssA`  |
> | `VnAdorAndGnssA`   |
> | `VnWriteAndGnssAB` |
> | `VnAdorAndGnssAB`  |
>
> **`PpsSource`**
>
> | Value           |
> |:----------------|
> | `PpsPinRising`  |
> | `PpsPinFalling` |
> | `SyncInRising`  |
> | `SyncInFalling` |
> | `None`          |
>
> **`Rate`**
>
> | Value     |
> |:----------|
> | `Rate1Hz` |
> | `Rate5Hz` |
>
> **`AntPower`**
>
> | Value      |
> |:-----------|
> | `Off`      |
> | `Internal` |
> | `External` |
>
> ### GnssAOffset
> >
>
> **ID:** 57 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `positionX` | `float` |             |
> | `positionY` | `float` |             |
> | `positionZ` | `float` |             |
>
> ### GnssSystemConfig
> >
>
> **ID:** 99 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute      | Type                              | Description |
> |:---------------|:----------------------------------|:------------|
> | `systems`        | `uint16_t`                        |             |
> | `minCno`         | `uint8_t`                         |             |
> | `minElev`        | `uint8_t`                         |             |
> | `maxSats`        | `uint8_t`                         |             |
> | `sbasMode`       | `uint8_t`                         |             |
> | `sbasSelect1`    | `uint16_t`                        |             |
> | `sbasSelect2`    | `uint16_t`                        |             |
> | `sbasSelect3`    | `uint16_t`                        |             |
> | `receiverSelect` | `GnssSystemConfig.ReceiverSelect` |             |
>
> **Nested Enums**
>
> **`ReceiverSelect`**
>
> | Value    |
> |:---------|
> | `GnssAB` |
> | `GnssA`  |
> | `GnssB`  |
>
> **`Systems`**
>
> | Value          |
> |:---------------|
> | `gps`          |
> | `sbas`         |
> | `glonass`      |
> | `beidou`       |
> | `galileo`      |
> | `imes`         |
> | `qzssL1Ca`     |
> | `qzssL1Saif`   |
>
> **`SbasMode`**
>
> | Value        |
> |:-------------|
> | `ranging`    |
> | `diffCorr`   |
> | `integrity`  |
> | `testMode`   |
>
> **`SbasSelect1`**
>
> | Value     |
> |:----------|
> | `sbas120` |
> | `sbas121` |
> | ...       |
> | `sbas135` |
>
> **`SbasSelect2`**
>
> | Value     |
> |:----------|
> | `sbas136` |
> | `sbas137` |
> | ...       |
> | `sbas151` |
>
> **`SbasSelect3`**
>
> | Value     |
> |:----------|
> | `sbas152` |
> | `sbas153` |
> | ...       |
> | `sbas158` |
>
> ### GnssSyncConfig
> >
>
> **ID:** 100 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute      | Type                           | Description |
> |:---------------|:-------------------------------|:------------|
> | `gnssSyncEnable` | `GnssSyncConfig.GnssSyncEnable`|             |
> | `polarity`       | `GnssSyncConfig.Polarity`      |             |
> | `specType`       | `GnssSyncConfig.SpecType`      |             |
> | `resv`           | `uint8_t`                      |             |
> | `period`         | `uint32_t`                     |             |
> | `pulseWidth`     | `uint32_t`                     |             |
> | `offset`         | `int32_t`                      |             |
>
> **Nested Enums**
>
> **`GnssSyncEnable`**
>
> | Value          |
> |:---------------|
> | `Off`          |
> | `AlwaysOn`     |
> | `OnWhenLocked` |
>
> **`Polarity`**
>
> | Value         |
> |:--------------|
> | `FallingEdge` |
> | `RisingEdge`  |
>
> **`SpecType`**
>
> | Value             |
> |:------------------|
> | `PeriodPulseWidth`|
> | `FreqDutyCycle`   |
>
> ### ExtGnssOffset
> >
>
> **ID:** 157 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `positionX` | `float` |             |
> | `positionY` | `float` |             |
> | `positionZ` | `float` |             |

---
### GNSSCompass

> ### GnssCompassBaseline
> >
>
> **ID:** 93 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute    | Type    | Description |
> |:-------------|:--------|:------------|
> | `positionX`    | `float` |             |
> | `positionY`    | `float` |             |
> | `positionZ`    | `float` |             |
> | `uncertaintyX` | `float` |             |
> | `uncertaintyY` | `float` |             |
> | `uncertaintyZ` | `float` |             |

---
### HardSoftIronEstimator

> ### RealTimeHsiControl
> >
>
> **ID:** 44 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute          | Type                                | Description |
> |:-------------------|:------------------------------------|:------------|
> | `mode`               | `RealTimeHsiControl.Mode`           |             |
> | `applyCompensation`  | `RealTimeHsiControl.ApplyCompensation` |             |
> | `convergeRate`       | `uint8_t`                           |             |
>
> **Nested Enums**
>
> **`Mode`**
>
> | Value   |
> |:--------|
> | `Off`   |
> | `Run`   |
> | `Reset` |
>
> **`ApplyCompensation`**
>
> | Value     |
> |:----------|
> | `Disable` |
> | `Enable`  |

---
### Heave

> ### HeaveBasicConfig
> >
>
> **ID:** 116 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute               | Type    | Description |
> |:------------------------|:--------|:------------|
> | `initialWavePeriod`     | `float` |             |
> | `initialWaveAmplitude`  | `float` |             |
> | `maxWavePeriod`         | `float` |             |
> | `minWaveAmplitude`      | `float` |             |
> | `delayedHeaveCutoffFreq`| `float` |             |
> | `heaveCutoffFreq`       | `float` |             |
> | `heaveRateCutoffFreq`   | `float` |             |

---
### IMU

> ### MagCal
> >
>
> **ID:** 23 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `magGain00` | `float` |             |
> | `magGain01` | `float` |             |
> | `magGain02` | `float` |             |
> | `magGain10` | `float` |             |
> | `magGain11` | `float` |             |
> | `magGain12` | `float` |             |
> | `magGain20` | `float` |             |
> | `magGain21` | `float` |             |
> | `magGain22` | `float` |             |
> | `magBiasX`  | `float` |             |
> | `magBiasY`  | `float` |             |
> | `magBiasZ`  | `float` |             |
>
> ### AccelCal
> >
>
> **ID:** 25 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute   | Type    | Description |
> |:------------|:--------|:------------|
> | `accelGain00` | `float` |             |
> | `accelGain01` | `float` |             |
> | `accelGain02` | `float` |             |
> | `accelGain10` | `float` |             |
> | `accelGain11` | `float` |             |
> | `accelGain12` | `float` |             |
> | `accelGain20` | `float` |             |
> | `accelGain21` | `float` |             |
> | `accelGain22` | `float` |             |
> | `accelBiasX`  | `float` |             |
> | `accelBiasY`  | `float` |             |
> | `accelBiasZ`  | `float` |             |
>
> ### RefFrameRot
> >
>
> **ID:** 26 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `rfr00`   | `float` |             |
> | `rfr01`   | `float` |             |
> | `rfr02`   | `float` |             |
> | `rfr10`   | `float` |             |
> | `rfr11`   | `float` |             |
> | `rfr12`   | `float` |             |
> | `rfr20`   | `float` |             |
> | `rfr21`   | `float` |             |
> | `rfr22`   | `float` |             |
>
> ### DeltaThetaVelConfig
> >
>
> **ID:** 82 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute             | Type                                        | Description |
> |:----------------------|:--------------------------------------------|:------------|
> | `integrationFrame`      | `DeltaThetaVelConfig.IntegrationFrame`      |             |
> | `gyroCompensation`      | `DeltaThetaVelConfig.GyroCompensation`      |             |
> | `accelCompensation`     | `DeltaThetaVelConfig.AccelCompensation`     |             |
> | `earthRateCompensation` | `DeltaThetaVelConfig.EarthRateCompensation` |             |
> | `resv`                  | `uint16_t`                                  |             |
>
> **Nested Enums**
>
> **`IntegrationFrame`**
>
> | Value  |
> |:-------|
> | `Body` |
> | `NED`  |
>
> **`GyroCompensation`**
>
> | Value  |
> |:-------|
> | `None` |
> | `Bias` |
>
> **`AccelCompensation`**
>
> | Value          |
> |:---------------|
> | `None`         |
> | `Gravity`      |
> | `Bias`         |
> | `BiasAndGravity` |
>
> **`EarthRateCompensation`**
>
> | Value           |
> |:----------------|
> | `None`          |
> | `GyroRate`      |
> | `CoriolisAccel` |
> | `RateAndCoriolis` |
>
> ### GyroCal
> >
>
> **ID:** 84 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute  | Type    | Description |
> |:-----------|:--------|:------------|
> | `gyroGain00` | `float` |             |
> | `gyroGain01` | `float` |             |
> | `gyroGain02` | `float` |             |
> | `gyroGain10` | `float` |             |
> | `gyroGain11` | `float` |             |
> | `gyroGain12` | `float` |             |
> | `gyroGain20` | `float` |             |
> | `gyroGain21` | `float` |             |
> | `gyroGain22` | `float` |             |
> | `gyroBiasX`  | `float` |             |
> | `gyroBiasY`  | `float` |             |
> | `gyroBiasZ`  | `float` |             |
>
> ### ImuFilterControl
> >
>
> **ID:** 85 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute       | Type      | Description |
> |:----------------|:----------|:------------|
> | `magWindowSize`   | `uint16_t`|             |
> | `accelWindowSize` | `uint16_t`|             |
> | `gyroWindowSize`  | `uint16_t`|             |
> | `tempWindowSize`  | `uint16_t`|             |
> | `presWindowSize`  | `uint16_t`|             |
> | `magFilterMode`   | `uint8_t` |             |
> | `accelFilterMode` | `uint8_t` |             |
> | `gyroFilterMode`  | `uint8_t` |             |
> | `tempFilterMode`  | `uint8_t` |             |
> | `presFilterMode`  | `uint8_t` |             |
>
> **Nested Enums**
>
> **`MagFilterMode`**
>
> | Value    |
> |:---------|
> | `uncomp` |
> | `comp`   |
>
> **`AccelFilterMode`**
>
> | Value    |
> |:---------|
> | `uncomp` |
> | `comp`   |
>
> **`GyroFilterMode`**
>
> | Value    |
> |:---------|
> | `uncomp` |
> | `comp`   |
>
> **`TempFilterMode`**
>
> | Value    |
> |:---------|
> | `uncomp` |
> | `comp`   |
>
> **`PresFilterMode`**
>
> | Value    |
> |:---------|
> | `uncomp` |
> | `comp`   |

---
### INS

> ### InsBasicConfig
> >
>
> **ID:** 67 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute   | Type                         | Description |
> |:------------|:-----------------------------|:------------|
> | `scenario`    | `InsBasicConfig.Scenario`    |             |
> | `ahrsAiding`  | `InsBasicConfig.AhrsAiding`  |             |
> | `estBaseline` | `InsBasicConfig.EstBaseline` |             |
> | `resv`        | `uint8_t`                    |             |
>
> **Nested Enums**
>
> **`Scenario`**
>
> | Value                  |
> |:-----------------------|
> | `Ahrs`                 |
> | `GnssInsWithPressure`  |
> | `GnssInsNoPressure`    |
> | `DualGnssNoPressure`   |
> | `DualGnssWithPressure` |
>
> **`AhrsAiding`**
>
> | Value     |
> |:----------|
> | `Disable` |
> | `Enable`  |
>
> **`EstBaseline`**
>
> | Value     |
> |:----------|
> | `Disable` |
> | `Enable`  |
>
> ### FilterStartupBias
> >
>
> **ID:** 74 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute   | Type    | Description |
> |:------------|:--------|:------------|
> | `gyroBiasX`   | `float` |             |
> | `gyroBiasY`   | `float` |             |
> | `gyroBiasZ`   | `float` |             |
> | `accelBiasX`  | `float` |             |
> | `accelBiasY`  | `float` |             |
> | `accelBiasZ`  | `float` |             |
> | `presBias`    | `float` |             |
>
> ### InsRefOffset
> >
>
> **ID:** 105 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute  | Type    | Description |
> |:-----------|:--------|:------------|
> | `refOffsetX` | `float` |             |
> | `refOffsetY` | `float` |             |
> | `refOffsetZ` | `float` |             |
> | `refUncertX` | `float` |             |
> | `refUncertY` | `float` |             |
> | `refUncertZ` | `float` |             |
>
> ### InsGnssSelect
> >
>
> **ID:** 144 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute            | Type                               | Description |
> |:---------------------|:-----------------------------------|:------------|
> | `activeReceiverSelect` | `InsGnssSelect.ActiveReceiverSelect` |             |
> | `usedForNavTime`       | `uint8_t`                          |             |
> | `hysteresisTime`       | `uint8_t`                          |             |
> | `useGnssCompass`       | `InsGnssSelect.UseGnssCompass`     |             |
> | `resv1`                | `uint8_t`                          |             |
> | `resv2`                | `uint8_t`                          |             |
>
> **Nested Enums**
>
> **`ActiveReceiverSelect`**
>
> | Value               |
> |:--------------------|
> | `None`              |
> | `PrimaryReceiver`   |
> | `SecondaryReceiver` |
> | `TertiaryReceiver`  |
> | `FallbackOnFailure` |
>
> **`UseGnssCompass`**
>
> | Value |
> |:------|
> | `Off` |
> | `On`  |

---
### System

> ### UserTag
> >
>
> **ID:** 0 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type   | Description |
> |:----------|:-------|:------------|
> | `tag`     | `char` |             |
>
> ### BaudRate
> >
>
> **ID:** 5 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute  | Type                  | Description |
> |:-----------|:----------------------|:------------|
> | `baudRate`   | `BaudRate.BaudRate`   |             |
> | `serialPort` | `BaudRate.SerialPort` |             |
>
> **Nested Enums**
>
> **`BaudRate`**
>
> | Value        |
> |:-------------|
> | `Baud9600`   |
> | `Baud19200`  |
> | `Baud38400`  |
> | `Baud57600`  |
> | `Baud115200` |
> | `Baud128000` |
> | `Baud230400` |
> | `Baud460800` |
> | `Baud921600` |
>
> **`SerialPort`**
>
> | Value     |
> |:----------|
> | `Serial1` |
> | `Serial2` |
>
> ### AsyncOutputType
> >
>
> **ID:** 6 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute  | Type                           | Description |
> |:-----------|:-------------------------------|:------------|
> | `ador`       | `AsyncOutputType.Ador`       |             |
> | `serialPort` | `AsyncOutputType.SerialPort` |             |
>
> **Nested Enums**
>
> **`Ador`**
>
> | Value |
> |:------|
> | `OFF` |
> | `YPR` |
> | `QTN` |
> | `QMR` |
> | `MAG` |
> | `ACC` |
> | `GYR` |
> | `MAR` |
> | `YMR` |
> | `YBA` |
> | `YIA` |
> | `IMU` |
> | `GPS` |
> | `GPE` |
> | `INS` |
> | `INE` |
> | `ISL` |
> | `ISE` |
> | `DTV` |
> | `G2S` |
> | `G2E` |
> | `HVE` |
>
> **`SerialPort`**
>
> | Value     |
> |:----------|
> | `Serial1` |
> | `Serial2` |
>
> ### AsyncOutputFreq
> >
>
> **ID:** 7 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute  | Type                           | Description |
> |:-----------|:-------------------------------|:------------|
> | `adof`       | `AsyncOutputFreq.Adof`       |             |
> | `serialPort` | `AsyncOutputFreq.SerialPort` |             |
>
> **Nested Enums**
>
> **`Adof`**
>
> | Value      |
> |:-----------|
> | `Rate0Hz`  |
> | `Rate1Hz`  |
> | `Rate2Hz`  |
> | `Rate4Hz`  |
> | `Rate5Hz`  |
> | `Rate10Hz` |
> | `Rate20Hz` |
> | `Rate25Hz` |
> | `Rate40Hz` |
> | `Rate50Hz` |
> | `Rate100Hz`|
> | `Rate200Hz`|
>
> **`SerialPort`**
>
> | Value     |
> |:----------|
> | `Serial1` |
> | `Serial2` |
>
> ### ProtocolControl
> >
>
> **ID:** 30 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute        | Type                                  | Description |
> |:-----------------|:--------------------------------------|:------------|
> | `asciiAppendCount` | `ProtocolControl.AsciiAppendCount`  |             |
> | `asciiAppendStatus`| `ProtocolControl.AsciiAppendStatus` |             |
> | `spiAppendCount`   | `ProtocolControl.SpiAppendCount`    |             |
> | `spiAppendStatus`  | `ProtocolControl.SpiAppendStatus`   |             |
> | `asciiChecksum`    | `ProtocolControl.AsciiChecksum`     |             |
> | `spiChecksum`      | `ProtocolControl.SpiChecksum`       |             |
> | `errorMode`        | `ProtocolControl.ErrorMode`         |             |
>
> **Nested Enums**
>
> **`AsciiAppendCount`**
>
> | Value         |
> |:--------------|
> | `None`        |
> | `SyncInCount` |
> | `SyncInTime`  |
> | `SyncOutCount`|
> | `GpsPps`      |
> | `GpsTow`      |
>
> **`AsciiAppendStatus`**
>
> | Value   |
> |:--------|
> | `None`  |
> | `Ahrs`  |
> | `Ins`   |
> | `Imu`   |
> | `Gnss1` |
> | `Gnss2` |
> | `Gnss3` |
>
> **`SpiAppendCount`**
>
> | Value         |
> |:--------------|
> | `None`        |
> | `SyncInCount` |
> | `SyncInTime`  |
> | `SyncOutCount`|
> | `GpsPps`      |
> | `GpsTow`      |
>
> **`SpiAppendStatus`**
>
> | Value   |
> |:--------|
> | `None`  |
> | `Ahrs`  |
> | `Ins`   |
> | `Imu`   |
> | `Gnss1` |
> | `Gnss2` |
> | `Gnss3` |
>
> **`AsciiChecksum`**
>
> | Value        |
> |:-------------|
> | `Checksum8bit` |
> | `Crc16bit`   |
>
> **`SpiChecksum`**
>
> | Value        |
> |:-------------|
> | `Off`        |
> | `Checksum8bit` |
> | `Crc16bit`   |
>
> **`ErrorMode`**
>
> | Value       |
> |:------------|
> | `Ignore`    |
> | `SendError` |
> | `AdorOff`   |
>
> ### SyncControl
> >
>
> **ID:** 32 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute         | Type                          | Description |
> |:------------------|:------------------------------|:------------|
> | `syncInMode`        | `SyncControl.SyncInMode`      |             |
> | `syncInEdge`        | `SyncControl.SyncInEdge`      |             |
> | `syncInSkipFactor`  | `uint16_t`                    |             |
> | `resv1`             | `uint32_t`                    |             |
> | `syncOutMode`       | `SyncControl.SyncOutMode`     |             |
> | `syncOutPolarity`   | `SyncControl.SyncOutPolarity` |             |
> | `syncOutSkipFactor` | `uint16_t`                    |             |
> | `syncOutPulseWidth` | `uint32_t`                    |             |
> | `resv2`             | `uint32_t`                    |             |
>
> **Nested Enums**
>
> **`SyncInMode`**
>
> | Value      |
> |:-----------|
> | `Disable`  |
> | `Count`    |
> | `ImuSample`|
> | `AsyncAll` |
> | `Async0`   |
>
> **`SyncInEdge`**
>
> | Value         |
> |:--------------|
> | `RisingEdge`  |
> | `FallingEdge` |
>
> **`SyncOutMode`**
>
> | Value      |
> |:-----------|
> | `None`     |
> | `ImuStart` |
> | `ImuReady` |
> | `NavReady` |
> | `GpsPps`   |
>
> **`SyncOutPolarity`**
>
> | Value           |
> |:----------------|
> | `NegativePulse` |
> | `PositivePulse` |
>
> ### BinaryOutput
> > Base for Binary Output Config registers, IDs 75-77
>
> **ID:** 75-77 | **Type:** Configuration (Base Class)
> ***
> **Attributes**
>
> | Attribute    | Type                 | Description |
> |:-------------|:---------------------|:------------|
> | `asyncMode`    | `uint16_t`           |             |
> | `rateDivisor`  | `uint16_t`           |             |
> | `common`       | `BinaryOutput.Common`|             |
> | `time`         | `BinaryOutput.Time`  |             |
> | `imu`          | `BinaryOutput.Imu`   |             |
> | `gnss`         | `BinaryOutput.Gnss`  |             |
> | `attitude`     | `BinaryOutput.Attitude`|             |
> | `ins`          | `BinaryOutput.Ins`   |             |
> | `gnss2`        | `BinaryOutput.Gnss2` |             |
> | `gnss3`        | `BinaryOutput.Gnss3` |             |
> | `outputGroups` | `uint8_t`            |             |
>
> **Nested Bitfields**
>
> **`BinaryOutput.Common` (Binary output config Common group)**
>
> | Field       | Type   |
> |:------------|:-------|
> | `timeStartup` | `bool` |
> | `timeGps`     | `bool` |
> | `timeSyncIn`  | `bool` |
> | `ypr`         | `bool` |
> | `quaternion`  | `bool` |
> | `angularRate` | `bool` |
> | `posLla`      | `bool` |
> | `velNed`      | `bool` |
> | `accel`       | `bool` |
> | `imu`         | `bool` |
> | `magPres`     | `bool` |
> | `deltas`      | `bool` |
> | `insStatus`   | `bool` |
> | `syncInCnt`   | `bool` |
> | `timeGpsPps`  | `bool` |
>
> **`BinaryOutput.Time` (Binary output config Time group)**
>
> | Field       | Type   |
> |:------------|:-------|
> | `timeStartup` | `bool` |
> | `timeGps`     | `bool` |
> | `timeGpsTow`  | `bool` |
> | `timeGpsWeek` | `bool` |
> | `timeSyncIn`  | `bool` |
> | `timeGpsPps`  | `bool` |
> | `timeUtc`     | `bool` |
> | `syncInCnt`   | `bool` |
> | `syncOutCnt`  | `bool` |
> | `timeStatus`  | `bool` |
>
> **`BinaryOutput.Imu` (Binary output config Imu group)**
>
> | Field       | Type   |
> |:------------|:-------|
> | `imuStatus`   | `bool` |
> | `uncompMag`   | `bool` |
> | `uncompAccel` | `bool` |
> | `uncompGyro`  | `bool` |
> | `temperature` | `bool` |
> | `pressure`    | `bool` |
> | `deltaTheta`  | `bool` |
> | `deltaVel`    | `bool` |
> | `mag`         | `bool` |
> | `accel`       | `bool` |
> | `angularRate` | `bool` |
> | `sensSat`     | `bool` |
>
> **`BinaryOutput.Gnss` (Binary output config Gnss group)**
>
> | Field               | Type   |
> |:--------------------|:-------|
> | `gnss1TimeUtc`        | `bool` |
> | `gps1Tow`             | `bool` |
> | `gps1Week`            | `bool` |
> | `gnss1NumSats`        | `bool` |
> | `gnss1Fix`            | `bool` |
> | `gnss1PosLla`         | `bool` |
> | `gnss1PosEcef`        | `bool` |
> | `gnss1VelNed`         | `bool` |
> | `gnss1VelEcef`        | `bool` |
> | `gnss1PosUncertainty` | `bool` |
> | `gnss1VelUncertainty` | `bool` |
> | `gnss1TimeUncertainty`| `bool` |
> | `gnss1TimeInfo`       | `bool` |
> | `gnss1Dop`            | `bool` |
> | `gnss1SatInfo`        | `bool` |
> | `gnss1RawMeas`        | `bool` |
> | `gnss1Status`         | `bool` |
> | `gnss1AltMsl`         | `bool` |
>
> **`BinaryOutput.Attitude` (Binary output config Attitude group)**
>
> | Field        | Type   |
> |:-------------|:-------|
> | `ypr`          | `bool` |
> | `quaternion`   | `bool` |
> | `dcm`          | `bool` |
> | `magNed`       | `bool` |
> | `accelNed`     | `bool` |
> | `linBodyAcc`   | `bool` |
> | `linAccelNed`  | `bool` |
> | `yprU`         | `bool` |
> | `heave`        | `bool` |
> | `attU`         | `bool` |
>
> **`BinaryOutput.Ins` (Binary output config Ins group)**
>
> | Field        | Type   |
> |:-------------|:-------|
> | `insStatus`    | `bool` |
> | `posLla`       | `bool` |
> | `posEcef`      | `bool` |
> | `velBody`      | `bool` |
> | `velNed`       | `bool` |
> | `velEcef`      | `bool` |
> | `magEcef`      | `bool` |
> | `accelEcef`    | `bool` |
> | `linAccelEcef` | `bool` |
> | `posU`         | `bool` |
> | `velU`         | `bool` |
>
> **`BinaryOutput.Gnss2` (Binary output config Gnss2 group)**
>
> | Field               | Type   |
> |:--------------------|:-------|
> | `gnss2TimeUtc`        | `bool` |
> | `gps2Tow`             | `bool` |
> | `gps2Week`            | `bool` |
> | `gnss2NumSats`        | `bool` |
> | `gnss2Fix`            | `bool` |
> | `gnss2PosLla`         | `bool` |
> | `gnss2PosEcef`        | `bool` |
> | `gnss2VelNed`         | `bool` |
> | `gnss2VelEcef`        | `bool` |
> | `gnss2PosUncertainty` | `bool` |
> | `gnss2VelUncertainty` | `bool` |
> | `gnss2TimeUncertainty`| `bool` |
> | `gnss2TimeInfo`       | `bool` |
> | `gnss2Dop`            | `bool` |
> | `gnss2SatInfo`        | `bool` |
> | `gnss2RawMeas`        | `bool` |
> | `gnss2Status`         | `bool` |
> | `gnss2AltMsl`         | `bool` |
>
> **`BinaryOutput.Gnss3` (Binary output config Gnss3 group)**
>
> | Field               | Type   |
> |:--------------------|:-------|
> | `gnss3TimeUtc`        | `bool` |
> | `gps3Tow`             | `bool` |
> | `gps3Week`            | `bool` |
> | `gnss3NumSats`        | `bool` |
> | `gnss3Fix`            | `bool` |
> | `gnss3PosLla`         | `bool` |
> | `gnss3PosEcef`        | `bool` |
> | `gnss3VelNed`         | `bool` |
> | `gnss3VelEcef`        | `bool` |
> | `gnss3PosUncertainty` | `bool` |
> | `gnss3VelUncertainty` | `bool` |
> | `gnss3TimeUncertainty`| `bool` |
> | `gnss3TimeInfo`       | `bool` |
> | `gnss3Dop`            | `bool` |
> | `gnss3SatInfo`        | `bool` |
> | `gnss3RawMeas`        | `bool` |
> | `gnss3Status`         | `bool` |
> | `gnss3AltMsl`         | `bool` |
>
> ### BinaryOutput1
> > Derived from BinaryOutput.
>
> **ID:** 75 | **Type:** Configuration
> ***
> This register's attributes are derived from the `BinaryOutput` base class.
>
> ### BinaryOutput2
> > Derived from BinaryOutput.
>
> **ID:** 76 | **Type:** Configuration
> ***
> This register's attributes are derived from the `BinaryOutput` base class.
>
> ### BinaryOutput3
> > Derived from BinaryOutput.
>
> **ID:** 77 | **Type:** Configuration
> ***
> This register's attributes are derived from the `BinaryOutput` base class.
>
> ### NmeaOutput1
> >
>
> **ID:** 101 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute    | Type                     | Description |
> |:-------------|:-------------------------|:------------|
> | `port`         | `NmeaOutput1.Port`       |             |
> | `rate`         | `NmeaOutput1.Rate`       |             |
> | `mode`         | `NmeaOutput1.Mode`       |             |
> | `gnssSelect`   | `NmeaOutput1.GnssSelect` |             |
> | `msgSelection` | `uint32_t`               |             |
>
> **Nested Enums**
>
> **`Port`**
>
> | Value         |
> |:--------------|
> | `None`        |
> | `Serial1`     |
> | `Serial2`     |
> | `Serial1And2` |
>
> **`Rate`**
>
> | Value      |
> |:-----------|
> | `Rate0Hz`  |
> | `Rate1Hz`  |
> | `Rate5Hz`  |
> | `Rate10Hz` |
> | `Rate20Hz` |
>
> **`Mode`**
>
> | Value      |
> |:-----------|
> | `V41GPID`  |
> | `V23GPID`  |
> | `V41SYSID` |
>
> **`GnssSelect`**
>
> | Value            |
> |:-----------------|
> | `GnssA`          |
> | `ActiveReceiver` |
>
> **`MsgSelection`**
>
> | Value      |
> |:-----------|
> | `rmcGnss`  |
> | `rmcIns`   |
> | `ggaGnss`  |
> | `ggaIns`   |
> | `gllGnss`  |
> | `gllIns`   |
> | `gsaGnss`  |
> | `gsvGnss`  |
> | `hdgIns`   |
> | `hdtIns`   |
> | `thsIns`   |
> | `vtgGnss`  |
> | `vtgIns`   |
> | `zdaGnss`  |
> | `zdaIns`   |
> | `pashrIns` |
> | `tss1Ins`  |
> | `indyn`    |
>
> ### NmeaOutput2
> >
>
> **ID:** 102 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute    | Type                     | Description |
> |:-------------|:-------------------------|:------------|
> | `port`         | `NmeaOutput2.Port`       |             |
> | `rate`         | `NmeaOutput2.Rate`       |             |
> | `mode`         | `NmeaOutput2.Mode`       |             |
> | `gnssSelect`   | `NmeaOutput2.GnssSelect` |             |
> | `msgSelection` | `uint32_t`               |             |
>
> **Nested Enums**
>
> **`Port`**
>
> | Value         |
> |:--------------|
> | `None`        |
> | `Serial1`     |
> | `Serial2`     |
> | `Serial1And2` |
>
> **`Rate`**
>
> | Value      |
> |:-----------|
> | `Rate0Hz`  |
> | `Rate1Hz`  |
> | `Rate5Hz`  |
> | `Rate10Hz` |
> | `Rate20Hz` |
>
> **`Mode`**
>
> | Value      |
> |:-----------|
> | `V41GPID`  |
> | `V23GPID`  |
> | `V41SYSID` |
>
> **`GnssSelect`**
>
> | Value            |
> |:-----------------|
> | `GnssA`          |
> | `ActiveReceiver` |
>
> **`MsgSelection`**
>
> | Value      |
> |:-----------|
> | `rmcGnss`  |
> | `rmcIns`   |
> | `ggaGnss`  |
> | `ggaIns`   |
> | `gllGnss`  |
> | `gllIns`   |
> | `gsaGnss`  |
> | `gsvGnss`  |
> | `hdgIns`   |
> | `hdtIns`   |
> | `thsIns`   |
> | `vtgGnss`  |
> | `vtgIns`   |
> | `zdaGnss`  |
> | `zdaIns`   |
> | `pashrIns` |
> | `tss1Ins`  |
> | `indyn`    |
>
> ### LegacyCompatibilitySettings
> >
>
> **ID:** 206 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute   | Type                                      | Description |
> |:------------|:------------------------------------------|:------------|
> | `insLegacy`   | `LegacyCompatibilitySettings.InsLegacy`   |             |
> | `gnssLegacy`  | `uint8_t`                                 |             |
> | `imuLegacy`   | `LegacyCompatibilitySettings.ImuLegacy`   |             |
> | `hwLegacy`    | `LegacyCompatibilitySettings.HwLegacy`    |             |
>
> **Nested Enums**
>
> **`InsLegacy`**
>
> | Value     |
> |:----------|
> | `Current` |
> | `Legacy`  |
>
> **`ImuLegacy`**
>
> | Value     |
> |:----------|
> | `Current` |
> | `Legacy`  |
>
> **`HwLegacy`**
>
> | Value     |
> |:----------|
> | `Current` |
> | `Legacy`  |
>
> **Nested Bitfields**
>
> **`GnssLegacy`**
>
> | Field                 |
> |:----------------------|
> | `legacyGnssFix`       |
> | `requireReg55Reset`   |
> | `alwaysPpsPulse`      |

---
### VelocityAiding

> ### VelAidingMeas
> >
>
> **ID:** 50 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute | Type    | Description |
> |:----------|:--------|:------------|
> | `velocityX` | `float` |             |
> | `velocityY` | `float` |             |
> | `velocityZ` | `float` |             |
>
> ### VelAidingControl
> >
>
> **ID:** 51 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute       | Type                               | Description |
> |:----------------|:-----------------------------------|:------------|
> | `velAidEnable`    | `VelAidingControl.VelAidEnable`    |             |
> | `velUncertTuning` | `float`                            |             |
> | `resv`            | `float`                            |             |
>
> **Nested Enums**
>
> **`VelAidEnable`**
>
> | Value     |
> |:----------|
> | `Disable` |
> | `Enable`  |

---
### WorldMagGravityModel

> ### RefModelConfig
> >
>
> **ID:** 83 | **Type:** Configuration
> ***
> **Attributes**
>
> | Attribute          | Type                               | Description |
> |:-------------------|:-----------------------------------|:------------|
> | `enableMagModel`     | `RefModelConfig.EnableMagModel`    |             |
> | `enableGravityModel` | `RefModelConfig.EnableGravityModel`|             |
> | `resv1`              | `uint8_t`                          |             |
> | `resv2`              | `uint8_t`                          |             |
> | `recalcThreshold`    | `uint32_t`                         |             |
> | `year`               | `float`                            |             |
> | `latitude`           | `double`                           |             |
> | `longitude`          | `double`                           |             |
> | `altitude`           | `double`                           |             |
>
> **Nested Enums**
>
> **`EnableMagModel`**
>
> | Value      |
> |:-----------|
> | `Disabled` |
> | `Enabled`  |
>
> **`EnableGravityModel`**
>
> | Value      |
> |:-----------|
> | `Disabled` |
> | `Enabled`  |