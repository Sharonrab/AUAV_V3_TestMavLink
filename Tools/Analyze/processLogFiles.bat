cd C:\Users\sharon\Documents\GitHub\SLUGS2-plus\Tools\Analyze


%for files from SLogger sd card
 python27 extractData.py 000F.log 
 python27 -m pymavlink.tools.mavloss --no-timestamps LOG00008.TXT --robust
 %for files from OPENLOG sd card
 python27 -m pymavlink.tools.mavlogdump LOG00008.TXT --format=csv --no-timestamps --dialect=slugs --csv_sep="," --types=ATTITUDE,LOCAL_POSITION_NED,GPS_RAW_INT,GPS_DATE_TIME,SLUGS_NAVIGATION,RAW_IMU,SCALED_PRESSURE,CPU_LOAD,HEARTBEAT,SYS_STATUS,RC_CHANNELS_RAW,SERVO_OUTPUT_RAW,SCALED_IMU,RAW_PRESSURE,MID_LVL_CMDS,SENSOR_DIAG,VOLT_SENSOR,STATUS_GPS,NOVATEL_DIAG,PTZ_STATUS > my_csv_log_file.csv
%for files from QGC
python27 -m pymavlink.tools.mavlogdump tryNewQGC.mavlink --format=csv --no-timestamps --dialect=slugs --csv_sep="," --types=ATTITUDE,LOCAL_POSITION_NED,GPS_RAW_INT,GPS_DATE_TIME,SLUGS_NAVIGATION,RAW_IMU,SCALED_PRESSURE,CPU_LOAD,HEARTBEAT,SYS_STATUS,RC_CHANNELS_RAW,SERVO_OUTPUT_RAW,SCALED_IMU,RAW_PRESSURE,MID_LVL_CMDS,SENSOR_DIAG,VOLT_SENSOR,STATUS_GPS,NOVATEL_DIAG,PTZ_STATUS > tryNewQGC.csv
python27 -m pymavlink.tools.mavlogdump tryNewQGC.mavlink --format=csv --dialect=slugs --csv_sep="," --types=ATTITUDE,LOCAL_POSITION_NED,GPS_RAW_INT,GPS_DATE_TIME,SLUGS_NAVIGATION,RAW_IMU,SCALED_PRESSURE,CPU_LOAD,HEARTBEAT,SYS_STATUS,RC_CHANNELS_RAW,SERVO_OUTPUT_RAW,SCALED_IMU,RAW_PRESSURE,MID_LVL_CMDS,SENSOR_DIAG,VOLT_SENSOR,STATUS_GPS,NOVATEL_DIAG,PTZ_STATUS > tryNewQGC.csv
																												

python27 extractData.py tryNewQGC.mavlink 