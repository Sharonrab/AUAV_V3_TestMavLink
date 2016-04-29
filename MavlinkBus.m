% % Bus object: mlPassthrough
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'bitfieldPt';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint16';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'target';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'uint8';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% mavlink_ctrl_srfc_pt_t = Simulink.Bus;
% mavlink_ctrl_srfc_pt_t.Description = sprintf('mavlink_ctrl_srfc_pt');
% mavlink_ctrl_srfc_pt_t.DataScope = 'Imported';
% mavlink_ctrl_srfc_pt_t.HeaderFile = 'mavlink.h'; 
% mavlink_ctrl_srfc_pt_t.Alignment = -1;
% mavlink_ctrl_srfc_pt_t.Elements = elems;
% assignin('base', 'mavlink_ctrl_srfc_pt_t', mavlink_ctrl_srfc_pt_t)
% 
% % Bus object: mlVISensor
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'voltage';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint16';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'reading2';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'uint16';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'r2Type';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'uint8';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% mavlink_volt_sensor_t = Simulink.Bus;
% mavlink_volt_sensor_t.Description = sprintf('mavlink_volt_sensor');
% mavlink_volt_sensor_t.DataScope = 'Imported';
% mavlink_volt_sensor_t.HeaderFile = 'mavlinkControlMcu.h'; 
% mavlink_volt_sensor_t.Alignment = -1;
% mavlink_volt_sensor_t.Elements = elems;
% assignin('base', 'mavlink_volt_sensor_t', mavlink_volt_sensor_t)
% 
% % Bus object: mlPwmCommands
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'time_usec';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint32';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'servo1_raw';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'uint16';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'servo2_raw';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'uint16';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'servo3_raw';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'uint16';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'servo4_raw';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'uint16';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'servo5_raw';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'uint16';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% elems(7) = Simulink.BusElement;
% elems(7).Name = 'servo6_raw';
% elems(7).Dimensions = 1;
% elems(7).DimensionsMode = 'Fixed';
% elems(7).DataType = 'uint16';
% elems(7).SampleTime = -1;
% elems(7).Complexity = 'real';
% elems(7).SamplingMode = 'Sample based';
% elems(7).Min = [];
% elems(7).Max = [];
% 
% elems(8) = Simulink.BusElement;
% elems(8).Name = 'servo7_raw';
% elems(8).Dimensions = 1;
% elems(8).DimensionsMode = 'Fixed';
% elems(8).DataType = 'uint16';
% elems(8).SampleTime = -1;
% elems(8).Complexity = 'real';
% elems(8).SamplingMode = 'Sample based';
% elems(8).Min = [];
% elems(8).Max = [];
% 
% elems(9) = Simulink.BusElement;
% elems(9).Name = 'servo8_raw';
% elems(9).Dimensions = 1;
% elems(9).DimensionsMode = 'Fixed';
% elems(9).DataType = 'uint16';
% elems(9).SampleTime = -1;
% elems(9).Complexity = 'real';
% elems(9).SamplingMode = 'Sample based';
% elems(9).Min = [];
% elems(9).Max = [];
% 
% elems(10) = Simulink.BusElement;
% elems(10).Name = 'port';
% elems(10).Dimensions = 1;
% elems(10).DimensionsMode = 'Fixed';
% elems(10).DataType = 'uint8';
% elems(10).SampleTime = -1;
% elems(10).Complexity = 'real';
% elems(10).SamplingMode = 'Sample based';
% elems(10).Min = [];
% elems(10).Max = [];
% 
% mavlink_servo_output_raw_t = Simulink.Bus;
% mavlink_servo_output_raw_t.Description = sprintf('mavlink_rtb');
% mavlink_servo_output_raw_t.DataScope = 'Imported';
% mavlink_servo_output_raw_t.HeaderFile = 'mavlinkControlMcu.h'; 
% mavlink_servo_output_raw_t.Alignment = -1;
% mavlink_servo_output_raw_t.Elements = elems;
% assignin('base', 'mavlink_servo_output_raw_t', mavlink_servo_output_raw_t)
% 
% % Bus object: mlRTB
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'rtb';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint8';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'track_mobile';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'uint8';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% mavlink_rtb_t = Simulink.Bus;
% mavlink_rtb_t.Description = sprintf('mavlink_rtb');
% mavlink_rtb_t.DataScope = 'Imported';
% mavlink_rtb_t.HeaderFile = 'mavlinkControlMcu.h'; 
% mavlink_rtb_t.Alignment = -1;
% mavlink_rtb_t.Elements = elems;
% assignin('base', 'mavlink_rtb_t', mavlink_rtb_t)
% 
% Bus object: mlRawIMU
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'time_boot_ms';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'xacc';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int16';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'yacc';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int16';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'zacc';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'xgyro';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int16';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'ygyro';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int16';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'zgyro';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'int16';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'xmag';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'int16';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'ymag';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'int16';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'zmag';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'int16';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

mavlink_raw_imu_t = Simulink.Bus;
mavlink_raw_imu_t.Description = sprintf('mavlink_raw_imu');
mavlink_raw_imu_t.DataScope = 'Imported';
mavlink_raw_imu_t.HeaderFile = 'mavlink.h'; 
mavlink_raw_imu_t.Alignment = -1;
mavlink_raw_imu_t.Elements = elems;
assignin('base', 'mavlink_raw_imu_t', mavlink_raw_imu_t)


% Bus object: mlHighresImu
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'time_boot_ms';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'xacc';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'yacc';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'zacc';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'single';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'xgyro';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'single';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'ygyro';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'single';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'zgyro';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'single';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'xmag';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'single';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'ymag';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'single';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'zmag';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'single';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

elems(11) = Simulink.BusElement;
elems(11).Name = 'abs_pressure';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'single';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];

elems(12) = Simulink.BusElement;
elems(12).Name = 'diff_pressure';
elems(12).Dimensions = 1;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'single';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];

elems(13) = Simulink.BusElement;
elems(13).Name = 'pressure_alt';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'single';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];

elems(14) = Simulink.BusElement;
elems(14).Name = 'temperature';
elems(14).Dimensions = 1;
elems(14).DimensionsMode = 'Fixed';
elems(14).DataType = 'single';
elems(14).SampleTime = -1;
elems(14).Complexity = 'real';
elems(14).SamplingMode = 'Sample based';
elems(14).Min = [];
elems(14).Max = [];

mavlink_highres_imu_t = Simulink.Bus;
mavlink_highres_imu_t.Description = sprintf('mavlink_highres_imu');
mavlink_highres_imu_t.DataScope = 'Imported';
mavlink_highres_imu_t.HeaderFile = 'mavlink.h'; 
mavlink_highres_imu_t.Alignment = -1;
mavlink_highres_imu_t.Elements = elems;
assignin('base', 'mavlink_highres_imu_t', mavlink_highres_imu_t)

% Bus object: mlFilteredData
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'time_boot_ms';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint32';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'xacc';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'yacc';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'zacc';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'single';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'xgyro';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'single';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'ygyro';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'single';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% elems(7) = Simulink.BusElement;
% elems(7).Name = 'zgyro';
% elems(7).Dimensions = 1;
% elems(7).DimensionsMode = 'Fixed';
% elems(7).DataType = 'single';
% elems(7).SampleTime = -1;
% elems(7).Complexity = 'real';
% elems(7).SamplingMode = 'Sample based';
% elems(7).Min = [];
% elems(7).Max = [];
% 
% elems(8) = Simulink.BusElement;
% elems(8).Name = 'xmag';
% elems(8).Dimensions = 1;
% elems(8).DimensionsMode = 'Fixed';
% elems(8).DataType = 'single';
% elems(8).SampleTime = -1;
% elems(8).Complexity = 'real';
% elems(8).SamplingMode = 'Sample based';
% elems(8).Min = [];
% elems(8).Max = [];
% 
% elems(9) = Simulink.BusElement;
% elems(9).Name = 'ymag';
% elems(9).Dimensions = 1;
% elems(9).DimensionsMode = 'Fixed';
% elems(9).DataType = 'single';
% elems(9).SampleTime = -1;
% elems(9).Complexity = 'real';
% elems(9).SamplingMode = 'Sample based';
% elems(9).Min = [];
% elems(9).Max = [];
% 
% elems(10) = Simulink.BusElement;
% elems(10).Name = 'zmag';
% elems(10).Dimensions = 1;
% elems(10).DimensionsMode = 'Fixed';
% elems(10).DataType = 'single';
% elems(10).SampleTime = -1;
% elems(10).Complexity = 'real';
% elems(10).SamplingMode = 'Sample based';
% elems(10).Min = [];
% elems(10).Max = [];
% 
% mavlink_scaled_imu_t = Simulink.Bus;
% mavlink_scaled_imu_t.Description = sprintf('mavlink_scaled_imu');
% mavlink_scaled_imu_t.DataScope = 'Imported';
% mavlink_scaled_imu_t.HeaderFile = 'mavlink.h'; 
% mavlink_scaled_imu_t.Alignment = -1;
% mavlink_scaled_imu_t.Elements = elems;
% assignin('base', 'mavlink_scaled_imu_t', mavlink_scaled_imu_t)
% 
% % Bus object: mlSensorBiasData
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'axBias';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'ayBias';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'azBias';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'gxBias';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'single';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'gyBias';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'single';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'gzBias';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'single';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% mavlink_sensor_bias_t = Simulink.Bus;
% mavlink_sensor_bias_t.Description = sprintf('mavlink_sensor_bias');
% mavlink_sensor_bias_t.DataScope = 'Imported';
% mavlink_sensor_bias_t.HeaderFile = 'mavlink.h'; 
% mavlink_sensor_bias_t.Alignment = -1;
% mavlink_sensor_bias_t.Elements = elems;
% assignin('base', 'mavlink_sensor_bias_t', mavlink_sensor_bias_t)
% 
% % Bus object: mlAttitudeData
% % Bus object: mlAttitudeRotated
% 
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'time_boot_ms';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint32';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'roll';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'pitch';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'yaw';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'single';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'rollspeed';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'single';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'pitchspeed';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'single';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% elems(7) = Simulink.BusElement;
% elems(7).Name = 'yawspeed';
% elems(7).Dimensions = 1;
% elems(7).DimensionsMode = 'Fixed';
% elems(7).DataType = 'single';
% elems(7).SampleTime = -1;
% elems(7).Complexity = 'real';
% elems(7).SamplingMode = 'Sample based';
% elems(7).Min = [];
% elems(7).Max = [];
% 
% mavlink_attitude_t = Simulink.Bus;
% mavlink_attitude_t.Description = sprintf('mavlink_attitude');
% mavlink_attitude_t.DataScope = 'Imported';
% mavlink_attitude_t.HeaderFile = 'mavlink.h'; 
% mavlink_attitude_t.Alignment = -1;
% mavlink_attitude_t.Elements = elems;
% assignin('base', 'mavlink_attitude_t', mavlink_attitude_t)
% 
% 
% % Bus object: mlParamInterface
% clear elems;
% SLUGS_PARAM_NAME_LENGTH	 = 16;
% PAR_PARAM_COUNT = 31;
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'param';
% elems(1).Dimensions = PAR_PARAM_COUNT;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'param_name';
% elems(2).Dimensions = [PAR_PARAM_COUNT SLUGS_PARAM_NAME_LENGTH];
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% pi_struct = Simulink.Bus;
% pi_struct.Description = sprintf('Parameter interface');
% pi_struct.DataScope = 'Imported';
% pi_struct.HeaderFile = 'mavlinkControlMcu.h'; 
% pi_struct.Alignment = -1;
% pi_struct.Elements = elems;
% assignin('base', 'pi_struct', pi_struct)
% clear PAR_PARAM_COUNT;
% clear SLUGS_PARAM_NAME_LENGTH; 
% % Bus object: mlISR
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'latitude';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'longitude';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'height';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'option1';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'uint8';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'option2';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'uint8';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'option3';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'uint8';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% mavlink_isr_location_t = Simulink.Bus;
% mavlink_isr_location_t.Description = sprintf('mavlink_isr_location');
% mavlink_isr_location_t.DataScope = 'Imported';
% mavlink_isr_location_t.HeaderFile = 'mavlink.h'; 
% mavlink_isr_location_t.Alignment = -1;
% mavlink_isr_location_t.Elements = elems;
% assignin('base', 'mavlink_isr_location_t', mavlink_isr_location_t)
% 
% 
% % Bus object: mlNavigation
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'u_m';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'phi_c';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'theta_c';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'psiDot_c';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'single';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'ay_body';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'single';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'totalDist';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'single';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% elems(7) = Simulink.BusElement;
% elems(7).Name = 'dist2Go';
% elems(7).Dimensions = 1;
% elems(7).DimensionsMode = 'Fixed';
% elems(7).DataType = 'single';
% elems(7).SampleTime = -1;
% elems(7).Complexity = 'real';
% elems(7).SamplingMode = 'Sample based';
% elems(7).Min = [];
% elems(7).Max = [];
% 
% elems(8) = Simulink.BusElement;
% elems(8).Name = 'h_c';
% elems(8).Dimensions = 1;
% elems(8).DimensionsMode = 'Fixed';
% elems(8).DataType = 'uint16';
% elems(8).SampleTime = -1;
% elems(8).Complexity = 'real';
% elems(8).SamplingMode = 'Sample based';
% elems(8).Min = [];
% elems(8).Max = [];
% 
% elems(9) = Simulink.BusElement;
% elems(9).Name = 'fromWP';
% elems(9).Dimensions = 1;
% elems(9).DimensionsMode = 'Fixed';
% elems(9).DataType = 'uint8';
% elems(9).SampleTime = -1;
% elems(9).Complexity = 'real';
% elems(9).SamplingMode = 'Sample based';
% elems(9).Min = [];
% elems(9).Max = [];
% 
% elems(10) = Simulink.BusElement;
% elems(10).Name = 'toWP';
% elems(10).Dimensions = 1;
% elems(10).DimensionsMode = 'Fixed';
% elems(10).DataType = 'uint8';
% elems(10).SampleTime = -1;
% elems(10).Complexity = 'real';
% elems(10).SamplingMode = 'Sample based';
% elems(10).Min = [];
% elems(10).Max = [];
% 
% mavlink_slugs_navigation_t = Simulink.Bus;
% mavlink_slugs_navigation_t.Description = sprintf('mavlink_slugs_navigation ');
% mavlink_slugs_navigation_t.DataScope = 'Imported';
% mavlink_slugs_navigation_t.HeaderFile = 'mavlink.h'; 
% mavlink_slugs_navigation_t.Alignment = -1;
% mavlink_slugs_navigation_t.Elements = elems;
% assignin('base', 'mavlink_slugs_navigation_t', mavlink_slugs_navigation_t)
% 
% 
% % Bus object: mlMobileLocation
% clear elems;
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'latitude';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'longitude';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'target';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'uint8';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% mavlink_slugs_mobile_location_t = Simulink.Bus;
% mavlink_slugs_mobile_location_t.Description = sprintf('mavlink_slugs_mobile_location');
% mavlink_slugs_mobile_location_t.DataScope = 'Imported';
% mavlink_slugs_mobile_location_t.HeaderFile = 'mavlink.h'; 
% mavlink_slugs_mobile_location_t.Alignment = -1;
% mavlink_slugs_mobile_location_t.Elements = elems;
% assignin('base', 'mavlink_slugs_mobile_location_t', mavlink_slugs_mobile_location_t)
% 
% 
% % Bus object: mlWpValues
% clear elems;
% MAX_NUM_WPS	= 17;
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'lat';
% elems(1).Dimensions= MAX_NUM_WPS;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'lon';
% elems(2).Dimensions= MAX_NUM_WPS;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'alt';
% elems(3).Dimensions= MAX_NUM_WPS;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'type';
% elems(4).Dimensions= MAX_NUM_WPS;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'uint8';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'orbit';
% elems(5).Dimensions= MAX_NUM_WPS;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'uint16';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'wpCount';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'uint8';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% mavlink_mission_item_values_t = Simulink.Bus;
% mavlink_mission_item_values_t.Description = sprintf('mavlink_mission_item_values');
% mavlink_mission_item_values_t.DataScope = 'Imported';
% mavlink_mission_item_values_t.HeaderFile = 'mavlinkControlMcu.h'; 
% mavlink_mission_item_values_t.Alignment = -1;
% mavlink_mission_item_values_t.Elements = elems;
% assignin('base', 'mavlink_mission_item_values_t', mavlink_mission_item_values_t)
% 
% clear MAX_NUM_WPS;
% % Bus object: mlMidLevelCommands
% clear elems;
% 
% 
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'hCommand';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'single';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'uCommand';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'rCommand';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'target';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'uint8';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% mavlink_mid_lvl_cmds_t = Simulink.Bus;
% mavlink_mid_lvl_cmds_t.Description = sprintf('mavlink_Mid_Level_Commands');
% mavlink_mid_lvl_cmds_t.DataScope = 'Imported';
% mavlink_mid_lvl_cmds_t.HeaderFile = 'mavlink.h'; 
% mavlink_mid_lvl_cmds_t.Alignment = -1;
% mavlink_mid_lvl_cmds_t.Elements = elems;
% assignin('base', 'mavlink_mid_lvl_cmds_t', mavlink_mid_lvl_cmds_t)
% 
% % Bus object: mlAirData
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'time_boot_ms';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'press_abs';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'single';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'press_diff';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'single';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'temperature';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

mavlink_scaled_pressure_t = Simulink.Bus;
mavlink_scaled_pressure_t.Description = sprintf('mavlink_scaled_pressure');
mavlink_scaled_pressure_t.DataScope = 'Imported';
mavlink_scaled_pressure_t.HeaderFile = 'mavlink.h'; 
mavlink_scaled_pressure_t.Alignment = -1;
mavlink_scaled_pressure_t.Elements = elems;
assignin('base', 'mavlink_scaled_pressure_t', mavlink_scaled_pressure_t)

% % Bus object: mlRawPressureData
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'time_boot_ms';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'press_abs';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int16';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'press_diff1';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int16';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'press_diff2';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'temperature';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'int16';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

mavlink_raw_pressure_t = Simulink.Bus;
mavlink_raw_pressure_t.Description = sprintf('mavlink_raw_pressure');
mavlink_raw_pressure_t.DataScope = 'Imported';
mavlink_raw_pressure_t.HeaderFile = 'mavlink.h'; 
mavlink_raw_pressure_t.Alignment = -1;
mavlink_raw_pressure_t.Elements = elems;

assignin('base', 'mavlink_raw_pressure_t', mavlink_raw_pressure_t)
% 
% % Bus object: mlLocalPositionData
% clear elems;
%  
% elems(1) = Simulink.BusElement;
% elems(1).Name = 'time_boot_ms';
% elems(1).Dimensions = 1;
% elems(1).DimensionsMode = 'Fixed';
% elems(1).DataType = 'uint32';
% elems(1).SampleTime = -1;
% elems(1).Complexity = 'real';
% elems(1).SamplingMode = 'Sample based';
% elems(1).Min = [];
% elems(1).Max = [];
% 
% elems(2) = Simulink.BusElement;
% elems(2).Name = 'x';
% elems(2).Dimensions = 1;
% elems(2).DimensionsMode = 'Fixed';
% elems(2).DataType = 'single';
% elems(2).SampleTime = -1;
% elems(2).Complexity = 'real';
% elems(2).SamplingMode = 'Sample based';
% elems(2).Min = [];
% elems(2).Max = [];
% 
% elems(3) = Simulink.BusElement;
% elems(3).Name = 'y';
% elems(3).Dimensions = 1;
% elems(3).DimensionsMode = 'Fixed';
% elems(3).DataType = 'single';
% elems(3).SampleTime = -1;
% elems(3).Complexity = 'real';
% elems(3).SamplingMode = 'Sample based';
% elems(3).Min = [];
% elems(3).Max = [];
% 
% elems(4) = Simulink.BusElement;
% elems(4).Name = 'z';
% elems(4).Dimensions = 1;
% elems(4).DimensionsMode = 'Fixed';
% elems(4).DataType = 'single';
% elems(4).SampleTime = -1;
% elems(4).Complexity = 'real';
% elems(4).SamplingMode = 'Sample based';
% elems(4).Min = [];
% elems(4).Max = [];
% 
% elems(5) = Simulink.BusElement;
% elems(5).Name = 'vx';
% elems(5).Dimensions = 1;
% elems(5).DimensionsMode = 'Fixed';
% elems(5).DataType = 'single';
% elems(5).SampleTime = -1;
% elems(5).Complexity = 'real';
% elems(5).SamplingMode = 'Sample based';
% elems(5).Min = [];
% elems(5).Max = [];
% 
% elems(6) = Simulink.BusElement;
% elems(6).Name = 'vy';
% elems(6).Dimensions = 1;
% elems(6).DimensionsMode = 'Fixed';
% elems(6).DataType = 'single';
% elems(6).SampleTime = -1;
% elems(6).Complexity = 'real';
% elems(6).SamplingMode = 'Sample based';
% elems(6).Min = [];
% elems(6).Max = [];
% 
% elems(7) = Simulink.BusElement;
% elems(7).Name = 'vz';
% elems(7).Dimensions = 1;
% elems(7).DimensionsMode = 'Fixed';
% elems(7).DataType = 'single';
% elems(7).SampleTime = -1;
% elems(7).Complexity = 'real';
% elems(7).SamplingMode = 'Sample based';
% elems(7).Min = [];
% elems(7).Max = [];
% 
% mavlink_local_position_ned_t = Simulink.Bus;
% mavlink_local_position_ned_t.Description = sprintf('NED position anc velocity');
% mavlink_local_position_ned_t.DataScope = 'Imported';
% mavlink_local_position_ned_t.HeaderFile = 'mavlink.h';
% mavlink_local_position_ned_t.Alignment = -1;
% mavlink_local_position_ned_t.Elements = elems;
% 
% assignin('base', 'mavlink_local_position_ned_t', mavlink_local_position_ned_t)
% clear elems;


% Bus object: mlRawGPSint
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'time_usec';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'lat';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'int32';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'lon';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'int32';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'alt';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'int32';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'eph';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'uint16';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'epv';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'uint16';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'vel';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint16';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'cog';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint16';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'fix_type';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'uint8';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'satellites_visible';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'uint8';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

mavlink_gps_raw_int_t = Simulink.Bus;
mavlink_gps_raw_int_t.Description = sprintf('mavlink_gps_raw_int');
mavlink_gps_raw_int_t.DataScope = 'Imported';
mavlink_gps_raw_int_t.HeaderFile = 'mavlink.h'; 
mavlink_gps_raw_int_t.Alignment = -1;
mavlink_gps_raw_int_t.Elements = elems;
assignin('base', 'mavlink_gps_raw_int_t', mavlink_gps_raw_int_t)

% Bus object: mlSysStatus
clear elems;

elems(1) = Simulink.BusElement;
elems(1).Name = 'onboard_control_sensors_present';
elems(1).Dimensions = 1;
elems(1).DimensionsMode = 'Fixed';
elems(1).DataType = 'uint32';
elems(1).SampleTime = -1;
elems(1).Complexity = 'real';
elems(1).SamplingMode = 'Sample based';
elems(1).Min = [];
elems(1).Max = [];

elems(2) = Simulink.BusElement;
elems(2).Name = 'onboard_control_sensors_enabled';
elems(2).Dimensions = 1;
elems(2).DimensionsMode = 'Fixed';
elems(2).DataType = 'uint32';
elems(2).SampleTime = -1;
elems(2).Complexity = 'real';
elems(2).SamplingMode = 'Sample based';
elems(2).Min = [];
elems(2).Max = [];

elems(3) = Simulink.BusElement;
elems(3).Name = 'onboard_control_sensors_health';
elems(3).Dimensions = 1;
elems(3).DimensionsMode = 'Fixed';
elems(3).DataType = 'uint32';
elems(3).SampleTime = -1;
elems(3).Complexity = 'real';
elems(3).SamplingMode = 'Sample based';
elems(3).Min = [];
elems(3).Max = [];

elems(4) = Simulink.BusElement;
elems(4).Name = 'load';
elems(4).Dimensions = 1;
elems(4).DimensionsMode = 'Fixed';
elems(4).DataType = 'uint16';
elems(4).SampleTime = -1;
elems(4).Complexity = 'real';
elems(4).SamplingMode = 'Sample based';
elems(4).Min = [];
elems(4).Max = [];

elems(5) = Simulink.BusElement;
elems(5).Name = 'voltage_battery';
elems(5).Dimensions = 1;
elems(5).DimensionsMode = 'Fixed';
elems(5).DataType = 'uint16';
elems(5).SampleTime = -1;
elems(5).Complexity = 'real';
elems(5).SamplingMode = 'Sample based';
elems(5).Min = [];
elems(5).Max = [];

elems(6) = Simulink.BusElement;
elems(6).Name = 'current_battery';
elems(6).Dimensions = 1;
elems(6).DimensionsMode = 'Fixed';
elems(6).DataType = 'int16';
elems(6).SampleTime = -1;
elems(6).Complexity = 'real';
elems(6).SamplingMode = 'Sample based';
elems(6).Min = [];
elems(6).Max = [];

elems(7) = Simulink.BusElement;
elems(7).Name = 'drop_rate_comm';
elems(7).Dimensions = 1;
elems(7).DimensionsMode = 'Fixed';
elems(7).DataType = 'uint16';
elems(7).SampleTime = -1;
elems(7).Complexity = 'real';
elems(7).SamplingMode = 'Sample based';
elems(7).Min = [];
elems(7).Max = [];

elems(8) = Simulink.BusElement;
elems(8).Name = 'errors_comm';
elems(8).Dimensions = 1;
elems(8).DimensionsMode = 'Fixed';
elems(8).DataType = 'uint16';
elems(8).SampleTime = -1;
elems(8).Complexity = 'real';
elems(8).SamplingMode = 'Sample based';
elems(8).Min = [];
elems(8).Max = [];

elems(9) = Simulink.BusElement;
elems(9).Name = 'errors_count1';
elems(9).Dimensions = 1;
elems(9).DimensionsMode = 'Fixed';
elems(9).DataType = 'uint16';
elems(9).SampleTime = -1;
elems(9).Complexity = 'real';
elems(9).SamplingMode = 'Sample based';
elems(9).Min = [];
elems(9).Max = [];

elems(10) = Simulink.BusElement;
elems(10).Name = 'errors_count2';
elems(10).Dimensions = 1;
elems(10).DimensionsMode = 'Fixed';
elems(10).DataType = 'uint16';
elems(10).SampleTime = -1;
elems(10).Complexity = 'real';
elems(10).SamplingMode = 'Sample based';
elems(10).Min = [];
elems(10).Max = [];

elems(11) = Simulink.BusElement;
elems(11).Name = 'errors_count3';
elems(11).Dimensions = 1;
elems(11).DimensionsMode = 'Fixed';
elems(11).DataType = 'uint16';
elems(11).SampleTime = -1;
elems(11).Complexity = 'real';
elems(11).SamplingMode = 'Sample based';
elems(11).Min = [];
elems(11).Max = [];

elems(12) = Simulink.BusElement;
elems(12).Name = 'errors_count4';
elems(12).Dimensions = 1;
elems(12).DimensionsMode = 'Fixed';
elems(12).DataType = 'uint16';
elems(12).SampleTime = -1;
elems(12).Complexity = 'real';
elems(12).SamplingMode = 'Sample based';
elems(12).Min = [];
elems(12).Max = [];

elems(13) = Simulink.BusElement;
elems(13).Name = 'battery_remaining';
elems(13).Dimensions = 1;
elems(13).DimensionsMode = 'Fixed';
elems(13).DataType = 'uint8';
elems(13).SampleTime = -1;
elems(13).Complexity = 'real';
elems(13).SamplingMode = 'Sample based';
elems(13).Min = [];
elems(13).Max = [];

mavlink_sys_status_t = Simulink.Bus;
mavlink_sys_status_t.Description = sprintf('mavlink_sys_status');
mavlink_sys_status_t.DataScope = 'Imported';
mavlink_sys_status_t.HeaderFile = 'mavlink.h'; 
mavlink_sys_status_t.Alignment = -1;
mavlink_sys_status_t.Elements = elems;
assignin('base', 'mavlink_sys_status_t', mavlink_sys_status_t)

clear elems;

% typedef struct __mavlink_gps_date_time_t
% {
%  uint8_t year; ///< Year reported by Gps 
%  uint8_t month; ///< Month reported by Gps 
%  uint8_t day; ///< Day reported by Gps 
%  uint8_t hour; ///< Hour reported by Gps 
%  uint8_t min; ///< Min reported by Gps 
%  uint8_t sec; ///< Sec reported by Gps  
%  uint8_t clockStat; ///< Clock Status. See table 47 page 211 OEMStar Manual  
%  uint8_t visSat; ///< Visible satellites reported by Gps  
%  uint8_t useSat; ///< Used satellites in Solution  
%  uint8_t GppGl; ///< GPS+GLONASS satellites in Solution  
%  uint8_t sigUsedMask; ///< GPS and GLONASS usage mask (bit 0 GPS_used? bit_4 GLONASS_used?)
%  uint8_t percentUsed; ///< Percent used GPS
% } mavlink_gps_date_time_t;