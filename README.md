# AUAV_V3_TestMavLink
Simulink model for AUAV3 pilot (Developed by ASL@UCSC)
The purpose of this branch is to reconstruct the sensor data block in the SLUGS_sensor model
 * added mlHighresImu to MavlinkBus.m
 * Most of the sensor data block in the SLUGS_sensor is replaced by a mavlink_highres_imu_ mavlink data structure which is instantiated as mlHighresImu bus elemnt. All data that was preiviosly read from the sensor data block is now read from mlHighresImu except for GPS data which is actually read as before without the get function (integer GPS data).
HIL data should eventually be read from a HIL_SENSOR mavlink data structure not implemented at this time.
 * changed mpu6500 scale range of accelerometers to 4g
 * the scaling factors from RawIMU data to HighResIMU data are set at the model preLoadFcn callback.
MagScale, AcclScale, GyroScle.
 * TODO: The temperature reading fead into mlHighresImu should be filtered and fuzed temperature reading from the MPU6050, BMP180 and other sources, currently it is just the barometer temperature.
 * TODO: change Servo In Out blok to pilot console output 
