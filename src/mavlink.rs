# [ doc = "The heartbeat message shows that a system is present and responding. The type of the MAV and Autopilot hardware allow the receiving system to treat further messages from this system appropriate (e.g. by laying out the user interface based on the autopilot)." ]
pub struct HEARTBEAT { # [ doc = "Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)" ] typ : u8 , # [ doc = "Autopilot type / class. defined in MAV_AUTOPILOT ENUM" ] autopilot : u8 , # [ doc = "System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h" ] base_mode : u8 , # [ doc = "A bitfield for use for autopilot-specific flags." ] custom_mode : u32 , # [ doc = "System status flag, see MAV_STATE ENUM" ] system_status : u8 , # [ doc = "MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version" ] mavlink_version : u8 , }
# [ doc = "The general system state. If the system is following the MAVLink standard, the system state is mainly defined by three orthogonal states/modes: The system mode, which is either LOCKED (motors shut down and locked), MANUAL (system under RC control), GUIDED (system with autonomous position control, position setpoint controlled manually) or AUTO (system guided by path/waypoint planner). The NAV_MODE defined the current flight state: LIFTOFF (often an open-loop maneuver), LANDING, WAYPOINTS or VECTOR. This represents the internal navigation state machine. The system status shows whether the system is currently active or not and if an emergency occured. During the CRITICAL and EMERGENCY states the MAV is still considered to be active, but should start emergency procedures autonomously. After a failure occured it should first move from active to critical to allow manual intervention and then move to emergency after a certain timeout." ]
pub struct SYS_STATUS { # [ doc = "Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR" ] onboard_control_sensors_present : u32 , # [ doc = "Bitmask showing which onboard controllers and sensors are enabled:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR" ] onboard_control_sensors_enabled : u32 , # [ doc = "Bitmask showing which onboard controllers and sensors are operational or have an error:  Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR" ] onboard_control_sensors_health : u32 , # [ doc = "Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000" ] load : u16 , # [ doc = "Battery voltage, in millivolts (1 = 1 millivolt)" ] voltage_battery : u16 , # [ doc = "Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current" ] current_battery : i16 , # [ doc = "Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery" ] battery_remaining : i8 , # [ doc = "Communication drops in percent, (0%: 0, 100%: 10\'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)" ] drop_rate_comm : u16 , # [ doc = "Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)" ] errors_comm : u16 , # [ doc = "Autopilot-specific errors" ] errors_count1 : u16 , # [ doc = "Autopilot-specific errors" ] errors_count2 : u16 , # [ doc = "Autopilot-specific errors" ] errors_count3 : u16 , # [ doc = "Autopilot-specific errors" ] errors_count4 : u16 , }
# [ doc = "The system time is the time of the master clock, typically the computer clock of the main onboard computer." ]
pub struct SYSTEM_TIME {
    #[doc = "Timestamp of the master clock in microseconds since UNIX epoch."] time_unix_usec: u64,
    #[doc = "Timestamp of the component clock since boot time in milliseconds."] time_boot_ms: u32,
}
# [ doc = "A ping message either requesting or responding to a ping. This allows to measure the system latencies, including serial port, radio modem and UDP connections." ]
pub struct PING { # [ doc = "Unix timestamp in microseconds or since system boot if smaller than MAVLink epoch (1.1.2009)" ] time_usec : u64 , # [ doc = "PING sequence" ] seq : u32 , # [ doc = "0: request ping from all receiving systems, if greater than 0: message is a ping response and number is the system id of the requesting system" ] target_system : u8 , # [ doc = "0: request ping from all receiving components, if greater than 0: message is a ping response and number is the system id of the requesting system" ] target_component : u8 , }
#[doc = "Request to control this MAV"]
pub struct CHANGE_OPERATOR_CONTROL { # [ doc = "System the GCS requests control for" ] target_system : u8 , # [ doc = "0: request control of this MAV, 1: Release control of this MAV" ] control_request : u8 , # [ doc = "0: key as plaintext, 1-255: future, different hashing/encryption variants. The GCS should in general use the safest mode possible initially and then gradually move down the encryption level if it gets a NACK message indicating an encryption mismatch." ] version : u8 , # [ doc = "Password / Key, depending on version plaintext or encrypted. 25 or less characters, NULL terminated. The characters may involve A-Z, a-z, 0-9, and \"!?,.-\"" ] passkey : [ char ; 25usize ] , }
#[doc = "Accept / deny control of this MAV"]
pub struct CHANGE_OPERATOR_CONTROL_ACK { # [ doc = "ID of the GCS this message " ] gcs_system_id : u8 , # [ doc = "0: request control of this MAV, 1: Release control of this MAV" ] control_request : u8 , # [ doc = "0: ACK, 1: NACK: Wrong passkey, 2: NACK: Unsupported passkey encryption method, 3: NACK: Already under control" ] ack : u8 , }
# [ doc = "Emit an encrypted signature / key identifying this system. PLEASE NOTE: This protocol has been kept simple, so transmitting the key requires an encrypted channel for true safety." ]
pub struct AUTH_KEY {
    #[doc = "key"] key: [char; 32usize],
}
# [ doc = "THIS INTERFACE IS DEPRECATED. USE COMMAND_LONG with MAV_CMD_DO_SET_MODE INSTEAD. Set the system mode, as defined by enum MAV_MODE. There is no target component id as the mode is by definition for the overall aircraft, not only for one component." ]
pub struct SET_MODE {
    #[doc = "The system setting the mode"] target_system: u8,
    #[doc = "The new base mode"] base_mode: u8,
    #[doc = "The new autopilot-specific mode. This field can be ignored by an autopilot."]
    custom_mode: u32,
}
# [ doc = "Request to read the onboard parameter with the param_id string id. Onboard parameters are stored as key[const char*] -> value[float]. This allows to send a parameter to any other component (such as the GCS) without the need of previous knowledge of possible parameter names. Thus the same GCS can store different parameters for different autopilots. See also https://mavlink.io/en/protocol/parameter.html for a full documentation of QGroundControl and IMU code." ]
pub struct PARAM_REQUEST_READ { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored)" ] param_index : i16 , }
#[doc = "Request all parameters of this component. After this request, all parameters are emitted."]
pub struct PARAM_REQUEST_LIST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
}
# [ doc = "Emit the value of a onboard parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows him to re-request missing parameters after a loss or timeout." ]
pub struct PARAM_VALUE { # [ doc = "Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Onboard parameter value" ] param_value : f32 , # [ doc = "Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types." ] param_type : u8 , # [ doc = "Total number of onboard parameters" ] param_count : u16 , # [ doc = "Index of this onboard parameter" ] param_index : u16 , }
# [ doc = "Set a parameter value TEMPORARILY to RAM. It will be reset to default on system reboot. Send the ACTION MAV_ACTION_STORAGE_WRITE to PERMANENTLY write the RAM contents to EEPROM. IMPORTANT: The receiving component should acknowledge the new parameter value by sending a param_value message to all communication partners. This will also ensure that multiple GCS all have an up-to-date list of all parameters. If the sending GCS did not receive a PARAM_VALUE message within its timeout time, it should re-send the PARAM_SET message." ]
pub struct PARAM_SET { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Onboard parameter value" ] param_value : f32 , # [ doc = "Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types." ] param_type : u8 , }
# [ doc = "The global position, as returned by the Global Positioning System (GPS). This is\n                NOT the global position estimate of the system, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame)." ]
pub struct GPS_RAW_INT { # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , # [ doc = "See the GPS_FIX_TYPE enum." ] fix_type : u8 , # [ doc = "Latitude (WGS84, EGM96 ellipsoid), in degrees * 1E7" ] lat : i32 , # [ doc = "Longitude (WGS84, EGM96 ellipsoid), in degrees * 1E7" ] lon : i32 , # [ doc = "Altitude (AMSL, NOT WGS84), in meters * 1000 (positive for up). Note that virtually all GPS modules provide the AMSL altitude in addition to the WGS84 altitude." ] alt : i32 , # [ doc = "GPS HDOP horizontal dilution of position (unitless). If unknown, set to: UINT16_MAX" ] eph : u16 , # [ doc = "GPS VDOP vertical dilution of position (unitless). If unknown, set to: UINT16_MAX" ] epv : u16 , # [ doc = "GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX" ] vel : u16 , # [ doc = "Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX" ] cog : u16 , # [ doc = "Number of satellites visible. If unknown, set to 255" ] satellites_visible : u8 , # [ doc = "Altitude (above WGS84, EGM96 ellipsoid), in meters * 1000 (positive for up)." ] alt_ellipsoid : i32 , # [ doc = "Position uncertainty in meters * 1000 (positive for up)." ] h_acc : u32 , # [ doc = "Altitude uncertainty in meters * 1000 (positive for up)." ] v_acc : u32 , # [ doc = "Speed uncertainty in meters * 1000 (positive for up)." ] vel_acc : u32 , # [ doc = "Heading / track uncertainty in degrees * 1e5." ] hdg_acc : u32 , }
# [ doc = "The positioning status, as reported by GPS. This message is intended to display status information about each satellite visible to the receiver. See message GLOBAL_POSITION for the global position estimate. This message can contain information for up to 20 satellites." ]
pub struct GPS_STATUS {
    #[doc = "Number of satellites visible"] satellites_visible: u8,
    #[doc = "Global satellite ID"] satellite_prn: [i8; 20usize],
    #[doc = "0: Satellite not used, 1: used for localization"] satellite_used: [i8; 20usize],
    #[doc = "Elevation (0: right on top of receiver, 90: on the horizon) of satellite"]
    satellite_elevation: [i8; 20usize],
    #[doc = "Direction of satellite, 0: 0 deg, 255: 360 deg."] satellite_azimuth: [i8; 20usize],
    #[doc = "Signal to noise ratio of satellite"] satellite_snr: [i8; 20usize],
}
# [ doc = "The RAW IMU readings for the usual 9DOF sensor setup. This message should contain the scaled values to the described units" ]
pub struct SCALED_IMU {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "X acceleration (mg)"] xacc: i16,
    #[doc = "Y acceleration (mg)"] yacc: i16,
    #[doc = "Z acceleration (mg)"] zacc: i16,
    #[doc = "Angular speed around X axis (millirad /sec)"] xgyro: i16,
    #[doc = "Angular speed around Y axis (millirad /sec)"] ygyro: i16,
    #[doc = "Angular speed around Z axis (millirad /sec)"] zgyro: i16,
    #[doc = "X Magnetic field (milli tesla)"] xmag: i16,
    #[doc = "Y Magnetic field (milli tesla)"] ymag: i16,
    #[doc = "Z Magnetic field (milli tesla)"] zmag: i16,
}
# [ doc = "The RAW IMU readings for the usual 9DOF sensor setup. This message should always contain the true raw values without any scaling to allow data capture and system debugging." ]
pub struct RAW_IMU {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "X acceleration (raw)"] xacc: i16,
    #[doc = "Y acceleration (raw)"] yacc: i16,
    #[doc = "Z acceleration (raw)"] zacc: i16,
    #[doc = "Angular speed around X axis (raw)"] xgyro: i16,
    #[doc = "Angular speed around Y axis (raw)"] ygyro: i16,
    #[doc = "Angular speed around Z axis (raw)"] zgyro: i16,
    #[doc = "X Magnetic field (raw)"] xmag: i16,
    #[doc = "Y Magnetic field (raw)"] ymag: i16,
    #[doc = "Z Magnetic field (raw)"] zmag: i16,
}
# [ doc = "The RAW pressure readings for the typical setup of one absolute pressure and one differential pressure sensor. The sensor values should be the raw, UNSCALED ADC values." ]
pub struct RAW_PRESSURE {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "Absolute pressure (raw)"] press_abs: i16,
    #[doc = "Differential pressure 1 (raw, 0 if nonexistant)"] press_diff1: i16,
    #[doc = "Differential pressure 2 (raw, 0 if nonexistant)"] press_diff2: i16,
    #[doc = "Raw Temperature measurement (raw)"] temperature: i16,
}
# [ doc = "The pressure readings for the typical setup of one absolute and differential pressure sensor. The units are as specified in each field." ]
pub struct SCALED_PRESSURE {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Absolute pressure (hectopascal)"] press_abs: f32,
    #[doc = "Differential pressure 1 (hectopascal)"] press_diff: f32,
    #[doc = "Temperature measurement (0.01 degrees celsius)"] temperature: i16,
}
#[doc = "The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right)."]
pub struct ATTITUDE {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Roll angle (rad, -pi..+pi)"] roll: f32,
    #[doc = "Pitch angle (rad, -pi..+pi)"] pitch: f32,
    #[doc = "Yaw angle (rad, -pi..+pi)"] yaw: f32,
    #[doc = "Roll angular speed (rad/s)"] rollspeed: f32,
    #[doc = "Pitch angular speed (rad/s)"] pitchspeed: f32,
    #[doc = "Yaw angular speed (rad/s)"] yawspeed: f32,
}
# [ doc = "The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)." ]
pub struct ATTITUDE_QUATERNION {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Quaternion component 1, w (1 in null-rotation)"] q1: f32,
    #[doc = "Quaternion component 2, x (0 in null-rotation)"] q2: f32,
    #[doc = "Quaternion component 3, y (0 in null-rotation)"] q3: f32,
    #[doc = "Quaternion component 4, z (0 in null-rotation)"] q4: f32,
    #[doc = "Roll angular speed (rad/s)"] rollspeed: f32,
    #[doc = "Pitch angular speed (rad/s)"] pitchspeed: f32,
    #[doc = "Yaw angular speed (rad/s)"] yawspeed: f32,
}
# [ doc = "The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)" ]
pub struct LOCAL_POSITION_NED {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "X Position"] x: f32,
    #[doc = "Y Position"] y: f32,
    #[doc = "Z Position"] z: f32,
    #[doc = "X Speed"] vx: f32,
    #[doc = "Y Speed"] vy: f32,
    #[doc = "Z Speed"] vz: f32,
}
# [ doc = "The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It\n               is designed as scaled integer message since the resolution of float is not sufficient." ]
pub struct GLOBAL_POSITION_INT { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Latitude, expressed as degrees * 1E7" ] lat : i32 , # [ doc = "Longitude, expressed as degrees * 1E7" ] lon : i32 , # [ doc = "Altitude in meters, expressed as * 1000 (millimeters), AMSL (not WGS84 - note that virtually all GPS modules provide the AMSL as well)" ] alt : i32 , # [ doc = "Altitude above ground in meters, expressed as * 1000 (millimeters)" ] relative_alt : i32 , # [ doc = "Ground X Speed (Latitude, positive north), expressed as m/s * 100" ] vx : i16 , # [ doc = "Ground Y Speed (Longitude, positive east), expressed as m/s * 100" ] vy : i16 , # [ doc = "Ground Z Speed (Altitude, positive down), expressed as m/s * 100" ] vz : i16 , # [ doc = "Vehicle heading (yaw angle) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX" ] hdg : u16 , }
# [ doc = "The scaled values of the RC channels received. (-100%) -10000, (0%) 0, (100%) 10000. Channels that are inactive should be set to UINT16_MAX." ]
pub struct RC_CHANNELS_SCALED { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos." ] port : u8 , # [ doc = "RC channel 1 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan1_scaled : i16 , # [ doc = "RC channel 2 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan2_scaled : i16 , # [ doc = "RC channel 3 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan3_scaled : i16 , # [ doc = "RC channel 4 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan4_scaled : i16 , # [ doc = "RC channel 5 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan5_scaled : i16 , # [ doc = "RC channel 6 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan6_scaled : i16 , # [ doc = "RC channel 7 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan7_scaled : i16 , # [ doc = "RC channel 8 value scaled, (-100%) -10000, (0%) 0, (100%) 10000, (invalid) INT16_MAX." ] chan8_scaled : i16 , # [ doc = "Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown." ] rssi : u8 , }
# [ doc = "The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification." ]
pub struct RC_CHANNELS_RAW { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows for more than 8 servos." ] port : u8 , # [ doc = "RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan1_raw : u16 , # [ doc = "RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan2_raw : u16 , # [ doc = "RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan3_raw : u16 , # [ doc = "RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan4_raw : u16 , # [ doc = "RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan5_raw : u16 , # [ doc = "RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan6_raw : u16 , # [ doc = "RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan7_raw : u16 , # [ doc = "RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan8_raw : u16 , # [ doc = "Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown." ] rssi : u8 , }
# [ doc = "The RAW values of the servo outputs (for RC input from the remote, use the RC_CHANNELS messages). The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%." ]
pub struct SERVO_OUTPUT_RAW { # [ doc = "Timestamp (microseconds since system boot)" ] time_usec : u32 , # [ doc = "Servo output port (set of 8 outputs = 1 port). Most MAVs will just use one, but this allows to encode more than 8 servos." ] port : u8 , # [ doc = "Servo output 1 value, in microseconds" ] servo1_raw : u16 , # [ doc = "Servo output 2 value, in microseconds" ] servo2_raw : u16 , # [ doc = "Servo output 3 value, in microseconds" ] servo3_raw : u16 , # [ doc = "Servo output 4 value, in microseconds" ] servo4_raw : u16 , # [ doc = "Servo output 5 value, in microseconds" ] servo5_raw : u16 , # [ doc = "Servo output 6 value, in microseconds" ] servo6_raw : u16 , # [ doc = "Servo output 7 value, in microseconds" ] servo7_raw : u16 , # [ doc = "Servo output 8 value, in microseconds" ] servo8_raw : u16 , # [ doc = "Servo output 9 value, in microseconds" ] servo9_raw : u16 , # [ doc = "Servo output 10 value, in microseconds" ] servo10_raw : u16 , # [ doc = "Servo output 11 value, in microseconds" ] servo11_raw : u16 , # [ doc = "Servo output 12 value, in microseconds" ] servo12_raw : u16 , # [ doc = "Servo output 13 value, in microseconds" ] servo13_raw : u16 , # [ doc = "Servo output 14 value, in microseconds" ] servo14_raw : u16 , # [ doc = "Servo output 15 value, in microseconds" ] servo15_raw : u16 , # [ doc = "Servo output 16 value, in microseconds" ] servo16_raw : u16 , }
# [ doc = "Request a partial list of mission items from the system/component. https://mavlink.io/en/protocol/mission.html. If start and end index are the same, just send one waypoint." ]
pub struct MISSION_REQUEST_PARTIAL_LIST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Start index, 0 by default"] start_index: i16,
    #[doc = "End index, -1 by default (-1: send list to end). Else a valid index of the list"]
    end_index: i16,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "This message is sent to the MAV to write a partial list. If start index == end index, only one item will be transmitted / updated. If the start index is NOT 0 and above the current list size, this request should be REJECTED!" ]
pub struct MISSION_WRITE_PARTIAL_LIST { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Start index, 0 by default and smaller / equal to the largest index of the current onboard list." ] start_index : i16 , # [ doc = "End index, equal or greater than start index." ] end_index : i16 , # [ doc = "Mission type, see MAV_MISSION_TYPE" ] mission_type : u8 , }
# [ doc = "Message encoding a mission item. This message is emitted to announce\n                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/protocol/mission.html." ]
pub struct MISSION_ITEM {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Sequence"] seq: u16,
    #[doc = "The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h"] frame: u8,
    #[doc = "The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs"]
    command: u16,
    #[doc = "false:0, true:1"] current: u8,
    #[doc = "autocontinue to next wp"] autocontinue: u8,
    #[doc = "PARAM1, see MAV_CMD enum"] param1: f32,
    #[doc = "PARAM2, see MAV_CMD enum"] param2: f32,
    #[doc = "PARAM3, see MAV_CMD enum"] param3: f32,
    #[doc = "PARAM4, see MAV_CMD enum"] param4: f32,
    #[doc = "PARAM5 / local: x position, global: latitude"] x: f32,
    #[doc = "PARAM6 / y position: global: longitude"] y: f32,
    #[doc = "PARAM7 / z position: global: altitude (relative or absolute, depending on frame."]
    z: f32,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM message. https://mavlink.io/en/protocol/mission.html" ]
pub struct MISSION_REQUEST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Sequence"] seq: u16,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "Set the mission item with sequence number seq as current item. This means that the MAV will continue to this mission item on the shortest path (not following the mission items in-between)." ]
pub struct MISSION_SET_CURRENT {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Sequence"] seq: u16,
}
# [ doc = "Message that announces the sequence number of the current active mission item. The MAV will fly towards this mission item." ]
pub struct MISSION_CURRENT {
    #[doc = "Sequence"] seq: u16,
}
#[doc = "Request the overall list of mission items from the system/component."]
pub struct MISSION_REQUEST_LIST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "This message is emitted as response to MISSION_REQUEST_LIST by the MAV and to initiate a write transaction. The GCS can then request the individual mission item based on the knowledge of the total number of waypoints." ]
pub struct MISSION_COUNT {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Number of mission items in the sequence"] count: u16,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
#[doc = "Delete all mission items at once."]
pub struct MISSION_CLEAR_ALL {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "A certain mission item has been reached. The system will either hold this position (or circle on the orbit) or (if the autocontinue on the WP was set) continue to the next waypoint." ]
pub struct MISSION_ITEM_REACHED {
    #[doc = "Sequence"] seq: u16,
}
# [ doc = "Ack message during waypoint handling. The type field states if this message is a positive ack (type=0) or if an error happened (type=non-zero)." ]
pub struct MISSION_ACK {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "See MAV_MISSION_RESULT enum"] typ: u8,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "As local waypoints exist, the global waypoint reference allows to transform between the local coordinate frame and the global (GPS) coordinate frame. This can be necessary when e.g. in- and outdoor settings are connected and the MAV should move from in- to outdoor." ]
pub struct SET_GPS_GLOBAL_ORIGIN {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Latitude (WGS84), in degrees * 1E7"] latitude: i32,
    #[doc = "Longitude (WGS84), in degrees * 1E7"] longitude: i32,
    #[doc = "Altitude (AMSL), in meters * 1000 (positive for up)"] altitude: i32,
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
}
# [ doc = "Once the MAV sets a new GPS-Local correspondence, this message announces the origin (0,0,0) position" ]
pub struct GPS_GLOBAL_ORIGIN {
    #[doc = "Latitude (WGS84), in degrees * 1E7"] latitude: i32,
    #[doc = "Longitude (WGS84), in degrees * 1E7"] longitude: i32,
    #[doc = "Altitude (AMSL), in meters * 1000 (positive for up)"] altitude: i32,
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
}
# [ doc = "Bind a RC channel to a parameter. The parameter should change accoding to the RC channel value." ]
pub struct PARAM_MAP_RC { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter index. Send -1 to use the param ID field as identifier (else the param id will be ignored), send -2 to disable any existing map for this rc_channel_index." ] param_index : i16 , # [ doc = "Index of parameter RC channel. Not equal to the RC channel id. Typically correpsonds to a potentiometer-knob on the RC." ] parameter_rc_channel_index : u8 , # [ doc = "Initial parameter value" ] param_value0 : f32 , # [ doc = "Scale, maps the RC range [-1, 1] to a parameter value" ] scale : f32 , # [ doc = "Minimum param value. The protocol does not define if this overwrites an onboard minimum value. (Depends on implementation)" ] param_value_min : f32 , # [ doc = "Maximum param value. The protocol does not define if this overwrites an onboard maximum value. (Depends on implementation)" ] param_value_max : f32 , }
# [ doc = "Request the information of the mission item with the sequence number seq. The response of the system to this message should be a MISSION_ITEM_INT message. https://mavlink.io/en/protocol/mission.html" ]
pub struct MISSION_REQUEST_INT {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Sequence"] seq: u16,
    #[doc = "Mission type, see MAV_MISSION_TYPE"] mission_type: u8,
}
# [ doc = "Set a safety zone (volume), which is defined by two corners of a cube. This message can be used to tell the MAV which setpoints/waypoints to accept and which to reject. Safety areas are often enforced by national or competition regulations." ]
pub struct SAFETY_SET_ALLOWED_AREA { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down." ] frame : u8 , # [ doc = "x position 1 / Latitude 1" ] p1x : f32 , # [ doc = "y position 1 / Longitude 1" ] p1y : f32 , # [ doc = "z position 1 / Altitude 1" ] p1z : f32 , # [ doc = "x position 2 / Latitude 2" ] p2x : f32 , # [ doc = "y position 2 / Longitude 2" ] p2y : f32 , # [ doc = "z position 2 / Altitude 2" ] p2z : f32 , }
#[doc = "Read out the safety zone the MAV currently assumes."]
pub struct SAFETY_ALLOWED_AREA { # [ doc = "Coordinate frame, as defined by MAV_FRAME enum in mavlink_types.h. Can be either global, GPS, right-handed with Z axis up or local, right handed, Z axis down." ] frame : u8 , # [ doc = "x position 1 / Latitude 1" ] p1x : f32 , # [ doc = "y position 1 / Longitude 1" ] p1y : f32 , # [ doc = "z position 1 / Altitude 1" ] p1z : f32 , # [ doc = "x position 2 / Latitude 2" ] p2x : f32 , # [ doc = "y position 2 / Longitude 2" ] p2y : f32 , # [ doc = "z position 2 / Altitude 2" ] p2z : f32 , }
# [ doc = "The attitude in the aeronautical frame (right-handed, Z-down, X-front, Y-right), expressed as quaternion. Quaternion order is w, x, y, z and a zero rotation would be expressed as (1 0 0 0)." ]
pub struct ATTITUDE_QUATERNION_COV {
    #[doc = "Timestamp (microseconds since system boot or since UNIX epoch)"] time_usec: u64,
    #[doc = "Quaternion components, w, x, y, z (1 0 0 0 is the null-rotation)"] q: [f32; 4usize],
    #[doc = "Roll angular speed (rad/s)"] rollspeed: f32,
    #[doc = "Pitch angular speed (rad/s)"] pitchspeed: f32,
    #[doc = "Yaw angular speed (rad/s)"] yawspeed: f32,
    #[doc = "Attitude covariance"] covariance: [f32; 9usize],
}
#[doc = "The state of the fixed wing navigation and position controller."]
pub struct NAV_CONTROLLER_OUTPUT {
    #[doc = "Current desired roll in degrees"] nav_roll: f32,
    #[doc = "Current desired pitch in degrees"] nav_pitch: f32,
    #[doc = "Current desired heading in degrees"] nav_bearing: i16,
    #[doc = "Bearing to current waypoint/target in degrees"] target_bearing: i16,
    #[doc = "Distance to active waypoint in meters"] wp_dist: u16,
    #[doc = "Current altitude error in meters"] alt_error: f32,
    #[doc = "Current airspeed error in meters/second"] aspd_error: f32,
    #[doc = "Current crosstrack error on x-y plane in meters"] xtrack_error: f32,
}
# [ doc = "The filtered global position (e.g. fused GPS and accelerometers). The position is in GPS-frame (right-handed, Z-up). It  is designed as scaled integer message since the resolution of float is not sufficient. NOTE: This message is intended for onboard networks / companion computers and higher-bandwidth links and optimized for accuracy and completeness. Please use the GLOBAL_POSITION_INT message for a minimal subset." ]
pub struct GLOBAL_POSITION_INT_COV { # [ doc = "Timestamp (microseconds since system boot or since UNIX epoch)" ] time_usec : u64 , # [ doc = "Class id of the estimator this estimate originated from." ] estimator_type : u8 , # [ doc = "Latitude, expressed as degrees * 1E7" ] lat : i32 , # [ doc = "Longitude, expressed as degrees * 1E7" ] lon : i32 , # [ doc = "Altitude in meters, expressed as * 1000 (millimeters), above MSL" ] alt : i32 , # [ doc = "Altitude above ground in meters, expressed as * 1000 (millimeters)" ] relative_alt : i32 , # [ doc = "Ground X Speed (Latitude), expressed as m/s" ] vx : f32 , # [ doc = "Ground Y Speed (Longitude), expressed as m/s" ] vy : f32 , # [ doc = "Ground Z Speed (Altitude), expressed as m/s" ] vz : f32 , # [ doc = "Covariance matrix (first six entries are the first ROW, next six entries are the second row, etc.)" ] covariance : [ f32 ; 36usize ] , }
# [ doc = "The filtered local position (e.g. fused computer vision and accelerometers). Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)" ]
pub struct LOCAL_POSITION_NED_COV { # [ doc = "Timestamp (microseconds since system boot or since UNIX epoch)" ] time_usec : u64 , # [ doc = "Class id of the estimator this estimate originated from." ] estimator_type : u8 , # [ doc = "X Position" ] x : f32 , # [ doc = "Y Position" ] y : f32 , # [ doc = "Z Position" ] z : f32 , # [ doc = "X Speed (m/s)" ] vx : f32 , # [ doc = "Y Speed (m/s)" ] vy : f32 , # [ doc = "Z Speed (m/s)" ] vz : f32 , # [ doc = "X Acceleration (m/s^2)" ] ax : f32 , # [ doc = "Y Acceleration (m/s^2)" ] ay : f32 , # [ doc = "Z Acceleration (m/s^2)" ] az : f32 , # [ doc = "Covariance matrix upper right triangular (first nine entries are the first ROW, next eight entries are the second row, etc.)" ] covariance : [ f32 ; 45usize ] , }
# [ doc = "The PPM values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification." ]
pub struct RC_CHANNELS { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Total number of RC channels being received. This can be larger than 18, indicating that more channels are available but not given in this message. This value should be 0 when no RC channels are available." ] chancount : u8 , # [ doc = "RC channel 1 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan1_raw : u16 , # [ doc = "RC channel 2 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan2_raw : u16 , # [ doc = "RC channel 3 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan3_raw : u16 , # [ doc = "RC channel 4 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan4_raw : u16 , # [ doc = "RC channel 5 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan5_raw : u16 , # [ doc = "RC channel 6 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan6_raw : u16 , # [ doc = "RC channel 7 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan7_raw : u16 , # [ doc = "RC channel 8 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan8_raw : u16 , # [ doc = "RC channel 9 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan9_raw : u16 , # [ doc = "RC channel 10 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan10_raw : u16 , # [ doc = "RC channel 11 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan11_raw : u16 , # [ doc = "RC channel 12 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan12_raw : u16 , # [ doc = "RC channel 13 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan13_raw : u16 , # [ doc = "RC channel 14 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan14_raw : u16 , # [ doc = "RC channel 15 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan15_raw : u16 , # [ doc = "RC channel 16 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan16_raw : u16 , # [ doc = "RC channel 17 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan17_raw : u16 , # [ doc = "RC channel 18 value, in microseconds. A value of UINT16_MAX implies the channel is unused." ] chan18_raw : u16 , # [ doc = "Receive signal strength indicator, 0: 0%, 100: 100%, 255: invalid/unknown." ] rssi : u8 , }
#[doc = "THIS INTERFACE IS DEPRECATED. USE SET_MESSAGE_INTERVAL INSTEAD."]
pub struct REQUEST_DATA_STREAM {
    #[doc = "The target requested to send the message stream."] target_system: u8,
    #[doc = "The target requested to send the message stream."] target_component: u8,
    #[doc = "The ID of the requested data stream"] req_stream_id: u8,
    #[doc = "The requested message rate"] req_message_rate: u16,
    #[doc = "1 to start sending, 0 to stop sending."] start_stop: u8,
}
#[doc = "THIS INTERFACE IS DEPRECATED. USE MESSAGE_INTERVAL INSTEAD."]
pub struct DATA_STREAM {
    #[doc = "The ID of the requested data stream"] stream_id: u8,
    #[doc = "The message rate"] message_rate: u16,
    #[doc = "1 stream is enabled, 0 stream is stopped."] on_off: u8,
}
# [ doc = "This message provides an API for manually controlling the vehicle using standard joystick axes nomenclature, along with a joystick-like input device. Unused axes can be disabled an buttons are also transmit as boolean values of their " ]
pub struct MANUAL_CONTROL { # [ doc = "The system to be controlled." ] target : u8 , # [ doc = "X-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to forward(1000)-backward(-1000) movement on a joystick and the pitch of a vehicle." ] x : i16 , # [ doc = "Y-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to left(-1000)-right(1000) movement on a joystick and the roll of a vehicle." ] y : i16 , # [ doc = "Z-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a separate slider movement with maximum being 1000 and minimum being -1000 on a joystick and the thrust of a vehicle. Positive values are positive thrust, negative values are negative thrust." ] z : i16 , # [ doc = "R-axis, normalized to the range [-1000,1000]. A value of INT16_MAX indicates that this axis is invalid. Generally corresponds to a twisting of the joystick, with counter-clockwise being 1000 and clockwise being -1000, and the yaw of a vehicle." ] r : i16 , # [ doc = "A bitfield corresponding to the joystick buttons\' current state, 1 for pressed, 0 for released. The lowest bit corresponds to Button 1." ] buttons : u16 , }
# [ doc = "The RAW values of the RC channels sent to the MAV to override info received from the RC radio. A value of UINT16_MAX means no change to that channel. A value of 0 means control of that channel should be released back to the RC radio. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification." ]
pub struct RC_CHANNELS_OVERRIDE { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "RC channel 1 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan1_raw : u16 , # [ doc = "RC channel 2 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan2_raw : u16 , # [ doc = "RC channel 3 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan3_raw : u16 , # [ doc = "RC channel 4 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan4_raw : u16 , # [ doc = "RC channel 5 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan5_raw : u16 , # [ doc = "RC channel 6 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan6_raw : u16 , # [ doc = "RC channel 7 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan7_raw : u16 , # [ doc = "RC channel 8 value, in microseconds. A value of UINT16_MAX means to ignore this field." ] chan8_raw : u16 , # [ doc = "RC channel 9 value, in microseconds. A value of 0 means to ignore this field." ] chan9_raw : u16 , # [ doc = "RC channel 10 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan10_raw : u16 , # [ doc = "RC channel 11 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan11_raw : u16 , # [ doc = "RC channel 12 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan12_raw : u16 , # [ doc = "RC channel 13 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan13_raw : u16 , # [ doc = "RC channel 14 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan14_raw : u16 , # [ doc = "RC channel 15 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan15_raw : u16 , # [ doc = "RC channel 16 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan16_raw : u16 , # [ doc = "RC channel 17 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan17_raw : u16 , # [ doc = "RC channel 18 value, in microseconds. A value of 0 or UINT16_MAX means to ignore this field." ] chan18_raw : u16 , }
# [ doc = "Message encoding a mission item. This message is emitted to announce\n                the presence of a mission item and to set a mission item on the system. The mission item can be either in x, y, z meters (type: LOCAL) or x:lat, y:lon, z:altitude. Local frame is Z-down, right handed (NED), global frame is Z-up, right handed (ENU). See also https://mavlink.io/en/protocol/mission.html." ]
pub struct MISSION_ITEM_INT { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Waypoint ID (sequence number). Starts at zero. Increases monotonically for each waypoint, no gaps in the sequence (0,1,2,3,4)." ] seq : u16 , # [ doc = "The coordinate system of the waypoint. see MAV_FRAME in mavlink_types.h" ] frame : u8 , # [ doc = "The scheduled action for the waypoint. see MAV_CMD in common.xml MAVLink specs" ] command : u16 , # [ doc = "false:0, true:1" ] current : u8 , # [ doc = "autocontinue to next wp" ] autocontinue : u8 , # [ doc = "PARAM1, see MAV_CMD enum" ] param1 : f32 , # [ doc = "PARAM2, see MAV_CMD enum" ] param2 : f32 , # [ doc = "PARAM3, see MAV_CMD enum" ] param3 : f32 , # [ doc = "PARAM4, see MAV_CMD enum" ] param4 : f32 , # [ doc = "PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7" ] x : i32 , # [ doc = "PARAM6 / y position: local: x position in meters * 1e4, global: longitude in degrees *10^7" ] y : i32 , # [ doc = "PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame." ] z : f32 , # [ doc = "Mission type, see MAV_MISSION_TYPE" ] mission_type : u8 , }
#[doc = "Metrics typically displayed on a HUD for fixed wing aircraft"]
pub struct VFR_HUD {
    #[doc = "Current airspeed in m/s"] airspeed: f32,
    #[doc = "Current ground speed in m/s"] groundspeed: f32,
    #[doc = "Current heading in degrees, in compass units (0..360, 0=north)"] heading: i16,
    #[doc = "Current throttle setting in integer percent, 0 to 100"] throttle: u16,
    #[doc = "Current altitude (MSL), in meters"] alt: f32,
    #[doc = "Current climb rate in meters/second"] climb: f32,
}
# [ doc = "Message encoding a command with parameters as scaled integers. Scaling depends on the actual command value." ]
pub struct COMMAND_INT { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "The coordinate system of the COMMAND. see MAV_FRAME in mavlink_types.h" ] frame : u8 , # [ doc = "The scheduled action for the mission item. see MAV_CMD in common.xml MAVLink specs" ] command : u16 , # [ doc = "false:0, true:1" ] current : u8 , # [ doc = "autocontinue to next wp" ] autocontinue : u8 , # [ doc = "PARAM1, see MAV_CMD enum" ] param1 : f32 , # [ doc = "PARAM2, see MAV_CMD enum" ] param2 : f32 , # [ doc = "PARAM3, see MAV_CMD enum" ] param3 : f32 , # [ doc = "PARAM4, see MAV_CMD enum" ] param4 : f32 , # [ doc = "PARAM5 / local: x position in meters * 1e4, global: latitude in degrees * 10^7" ] x : i32 , # [ doc = "PARAM6 / local: y position in meters * 1e4, global: longitude in degrees * 10^7" ] y : i32 , # [ doc = "PARAM7 / z position: global: altitude in meters (relative or absolute, depending on frame." ] z : f32 , }
#[doc = "Send a command with up to seven parameters to the MAV"]
pub struct COMMAND_LONG { # [ doc = "System which should execute the command" ] target_system : u8 , # [ doc = "Component which should execute the command, 0 for all components" ] target_component : u8 , # [ doc = "Command ID, as defined by MAV_CMD enum." ] command : u16 , # [ doc = "0: First transmission of this command. 1-255: Confirmation transmissions (e.g. for kill command)" ] confirmation : u8 , # [ doc = "Parameter 1, as defined by MAV_CMD enum." ] param1 : f32 , # [ doc = "Parameter 2, as defined by MAV_CMD enum." ] param2 : f32 , # [ doc = "Parameter 3, as defined by MAV_CMD enum." ] param3 : f32 , # [ doc = "Parameter 4, as defined by MAV_CMD enum." ] param4 : f32 , # [ doc = "Parameter 5, as defined by MAV_CMD enum." ] param5 : f32 , # [ doc = "Parameter 6, as defined by MAV_CMD enum." ] param6 : f32 , # [ doc = "Parameter 7, as defined by MAV_CMD enum." ] param7 : f32 , }
#[doc = "Report status of a command. Includes feedback whether the command was executed."]
pub struct COMMAND_ACK { # [ doc = "Command ID, as defined by MAV_CMD enum." ] command : u16 , # [ doc = "See MAV_RESULT enum" ] result : u8 , # [ doc = "WIP: Also used as result_param1, it can be set with a enum containing the errors reasons of why the command was denied or the progress percentage or 255 if unknown the progress when result is MAV_RESULT_IN_PROGRESS." ] progress : u8 , # [ doc = "WIP: Additional parameter of the result, example: which parameter of MAV_CMD_NAV_WAYPOINT caused it to be denied." ] result_param2 : i32 , # [ doc = "WIP: System which requested the command to be executed" ] target_system : u8 , # [ doc = "WIP: Component which requested the command to be executed" ] target_component : u8 , }
#[doc = "Setpoint in roll, pitch, yaw and thrust from the operator"]
pub struct MANUAL_SETPOINT {
    #[doc = "Timestamp in milliseconds since system boot"] time_boot_ms: u32,
    #[doc = "Desired roll rate in radians per second"] roll: f32,
    #[doc = "Desired pitch rate in radians per second"] pitch: f32,
    #[doc = "Desired yaw rate in radians per second"] yaw: f32,
    #[doc = "Collective thrust, normalized to 0 .. 1"] thrust: f32,
    #[doc = "Flight mode switch position, 0.. 255"] mode_switch: u8,
    #[doc = "Override mode switch position, 0.. 255"] manual_override_switch: u8,
}
# [ doc = "Sets a desired vehicle attitude. Used by an external controller to command the vehicle (manual controller or other system)." ]
pub struct SET_ATTITUDE_TARGET { # [ doc = "Timestamp in milliseconds since system boot" ] time_boot_ms : u32 , # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 6: reserved, bit 7: throttle, bit 8: attitude" ] type_mask : u8 , # [ doc = "Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)" ] q : [ f32 ; 4usize ] , # [ doc = "Body roll rate in radians per second" ] body_roll_rate : f32 , # [ doc = "Body pitch rate in radians per second" ] body_pitch_rate : f32 , # [ doc = "Body yaw rate in radians per second" ] body_yaw_rate : f32 , # [ doc = "Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)" ] thrust : f32 , }
# [ doc = "Reports the current commanded attitude of the vehicle as specified by the autopilot. This should match the commands sent in a SET_ATTITUDE_TARGET message if the vehicle is being controlled this way." ]
pub struct ATTITUDE_TARGET { # [ doc = "Timestamp in milliseconds since system boot" ] time_boot_ms : u32 , # [ doc = "Mappings: If any of these bits are set, the corresponding input should be ignored: bit 1: body roll rate, bit 2: body pitch rate, bit 3: body yaw rate. bit 4-bit 7: reserved, bit 8: attitude" ] type_mask : u8 , # [ doc = "Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)" ] q : [ f32 ; 4usize ] , # [ doc = "Body roll rate in radians per second" ] body_roll_rate : f32 , # [ doc = "Body pitch rate in radians per second" ] body_pitch_rate : f32 , # [ doc = "Body yaw rate in radians per second" ] body_yaw_rate : f32 , # [ doc = "Collective thrust, normalized to 0 .. 1 (-1 .. 1 for vehicles capable of reverse trust)" ] thrust : f32 , }
# [ doc = "Sets a desired vehicle position in a local north-east-down coordinate frame. Used by an external controller to command the vehicle (manual controller or other system)." ]
pub struct SET_POSITION_TARGET_LOCAL_NED { # [ doc = "Timestamp in milliseconds since system boot" ] time_boot_ms : u32 , # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9" ] coordinate_frame : u8 , # [ doc = "Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate" ] type_mask : u16 , # [ doc = "X Position in NED frame in meters" ] x : f32 , # [ doc = "Y Position in NED frame in meters" ] y : f32 , # [ doc = "Z Position in NED frame in meters (note, altitude is negative in NED)" ] z : f32 , # [ doc = "X velocity in NED frame in meter / s" ] vx : f32 , # [ doc = "Y velocity in NED frame in meter / s" ] vy : f32 , # [ doc = "Z velocity in NED frame in meter / s" ] vz : f32 , # [ doc = "X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afx : f32 , # [ doc = "Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afy : f32 , # [ doc = "Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afz : f32 , # [ doc = "yaw setpoint in rad" ] yaw : f32 , # [ doc = "yaw rate setpoint in rad/s" ] yaw_rate : f32 , }
# [ doc = "Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_LOCAL_NED if the vehicle is being controlled this way." ]
pub struct POSITION_TARGET_LOCAL_NED { # [ doc = "Timestamp in milliseconds since system boot" ] time_boot_ms : u32 , # [ doc = "Valid options are: MAV_FRAME_LOCAL_NED = 1, MAV_FRAME_LOCAL_OFFSET_NED = 7, MAV_FRAME_BODY_NED = 8, MAV_FRAME_BODY_OFFSET_NED = 9" ] coordinate_frame : u8 , # [ doc = "Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate" ] type_mask : u16 , # [ doc = "X Position in NED frame in meters" ] x : f32 , # [ doc = "Y Position in NED frame in meters" ] y : f32 , # [ doc = "Z Position in NED frame in meters (note, altitude is negative in NED)" ] z : f32 , # [ doc = "X velocity in NED frame in meter / s" ] vx : f32 , # [ doc = "Y velocity in NED frame in meter / s" ] vy : f32 , # [ doc = "Z velocity in NED frame in meter / s" ] vz : f32 , # [ doc = "X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afx : f32 , # [ doc = "Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afy : f32 , # [ doc = "Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afz : f32 , # [ doc = "yaw setpoint in rad" ] yaw : f32 , # [ doc = "yaw rate setpoint in rad/s" ] yaw_rate : f32 , }
# [ doc = "Sets a desired vehicle position, velocity, and/or acceleration in a global coordinate system (WGS84). Used by an external controller to command the vehicle (manual controller or other system)." ]
pub struct SET_POSITION_TARGET_GLOBAL_INT { # [ doc = "Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency." ] time_boot_ms : u32 , # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11" ] coordinate_frame : u8 , # [ doc = "Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate" ] type_mask : u16 , # [ doc = "X Position in WGS84 frame in 1e7 * degrees" ] lat_int : i32 , # [ doc = "Y Position in WGS84 frame in 1e7 * degrees" ] lon_int : i32 , # [ doc = "Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT" ] alt : f32 , # [ doc = "X velocity in NED frame in meter / s" ] vx : f32 , # [ doc = "Y velocity in NED frame in meter / s" ] vy : f32 , # [ doc = "Z velocity in NED frame in meter / s" ] vz : f32 , # [ doc = "X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afx : f32 , # [ doc = "Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afy : f32 , # [ doc = "Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afz : f32 , # [ doc = "yaw setpoint in rad" ] yaw : f32 , # [ doc = "yaw rate setpoint in rad/s" ] yaw_rate : f32 , }
# [ doc = "Reports the current commanded vehicle position, velocity, and acceleration as specified by the autopilot. This should match the commands sent in SET_POSITION_TARGET_GLOBAL_INT if the vehicle is being controlled this way." ]
pub struct POSITION_TARGET_GLOBAL_INT { # [ doc = "Timestamp in milliseconds since system boot. The rationale for the timestamp in the setpoint is to allow the system to compensate for the transport delay of the setpoint. This allows the system to compensate processing latency." ] time_boot_ms : u32 , # [ doc = "Valid options are: MAV_FRAME_GLOBAL_INT = 5, MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6, MAV_FRAME_GLOBAL_TERRAIN_ALT_INT = 11" ] coordinate_frame : u8 , # [ doc = "Bitmask to indicate which dimensions should be ignored by the vehicle: a value of 0b0000000000000000 or 0b0000001000000000 indicates that none of the setpoint dimensions should be ignored. If bit 10 is set the floats afx afy afz should be interpreted as force instead of acceleration. Mapping: bit 1: x, bit 2: y, bit 3: z, bit 4: vx, bit 5: vy, bit 6: vz, bit 7: ax, bit 8: ay, bit 9: az, bit 10: is force setpoint, bit 11: yaw, bit 12: yaw rate" ] type_mask : u16 , # [ doc = "X Position in WGS84 frame in 1e7 * degrees" ] lat_int : i32 , # [ doc = "Y Position in WGS84 frame in 1e7 * degrees" ] lon_int : i32 , # [ doc = "Altitude in meters in AMSL altitude, not WGS84 if absolute or relative, above terrain if GLOBAL_TERRAIN_ALT_INT" ] alt : f32 , # [ doc = "X velocity in NED frame in meter / s" ] vx : f32 , # [ doc = "Y velocity in NED frame in meter / s" ] vy : f32 , # [ doc = "Z velocity in NED frame in meter / s" ] vz : f32 , # [ doc = "X acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afx : f32 , # [ doc = "Y acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afy : f32 , # [ doc = "Z acceleration or force (if bit 10 of type_mask is set) in NED frame in meter / s^2 or N" ] afz : f32 , # [ doc = "yaw setpoint in rad" ] yaw : f32 , # [ doc = "yaw rate setpoint in rad/s" ] yaw_rate : f32 , }
# [ doc = "The offset in X, Y, Z and yaw between the LOCAL_POSITION_NED messages of MAV X and the global coordinate frame in NED coordinates. Coordinate frame is right-handed, Z-axis down (aeronautical frame, NED / north-east-down convention)" ]
pub struct LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "X Position"] x: f32,
    #[doc = "Y Position"] y: f32,
    #[doc = "Z Position"] z: f32,
    #[doc = "Roll"] roll: f32,
    #[doc = "Pitch"] pitch: f32,
    #[doc = "Yaw"] yaw: f32,
}
# [ doc = "DEPRECATED PACKET! Suffers from missing airspeed fields and singularities due to Euler angles. Please use HIL_STATE_QUATERNION instead. Sent from simulation to autopilot. This packet is useful for high throughput applications such as hardware in the loop simulations." ]
pub struct HIL_STATE {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "Roll angle (rad)"] roll: f32,
    #[doc = "Pitch angle (rad)"] pitch: f32,
    #[doc = "Yaw angle (rad)"] yaw: f32,
    #[doc = "Body frame roll / phi angular speed (rad/s)"] rollspeed: f32,
    #[doc = "Body frame pitch / theta angular speed (rad/s)"] pitchspeed: f32,
    #[doc = "Body frame yaw / psi angular speed (rad/s)"] yawspeed: f32,
    #[doc = "Latitude, expressed as degrees * 1E7"] lat: i32,
    #[doc = "Longitude, expressed as degrees * 1E7"] lon: i32,
    #[doc = "Altitude in meters, expressed as * 1000 (millimeters)"] alt: i32,
    #[doc = "Ground X Speed (Latitude), expressed as m/s * 100"] vx: i16,
    #[doc = "Ground Y Speed (Longitude), expressed as m/s * 100"] vy: i16,
    #[doc = "Ground Z Speed (Altitude), expressed as m/s * 100"] vz: i16,
    #[doc = "X acceleration (mg)"] xacc: i16,
    #[doc = "Y acceleration (mg)"] yacc: i16,
    #[doc = "Z acceleration (mg)"] zacc: i16,
}
#[doc = "Sent from autopilot to simulation. Hardware in the loop control outputs"]
pub struct HIL_CONTROLS {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "Control output -1 .. 1"] roll_ailerons: f32,
    #[doc = "Control output -1 .. 1"] pitch_elevator: f32,
    #[doc = "Control output -1 .. 1"] yaw_rudder: f32,
    #[doc = "Throttle 0 .. 1"] throttle: f32,
    #[doc = "Aux 1, -1 .. 1"] aux1: f32,
    #[doc = "Aux 2, -1 .. 1"] aux2: f32,
    #[doc = "Aux 3, -1 .. 1"] aux3: f32,
    #[doc = "Aux 4, -1 .. 1"] aux4: f32,
    #[doc = "System mode (MAV_MODE)"] mode: u8,
    #[doc = "Navigation mode (MAV_NAV_MODE)"] nav_mode: u8,
}
# [ doc = "Sent from simulation to autopilot. The RAW values of the RC channels received. The standard PPM modulation is as follows: 1000 microseconds: 0%, 2000 microseconds: 100%. Individual receivers/transmitters might violate this specification." ]
pub struct HIL_RC_INPUTS_RAW {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "RC channel 1 value, in microseconds"] chan1_raw: u16,
    #[doc = "RC channel 2 value, in microseconds"] chan2_raw: u16,
    #[doc = "RC channel 3 value, in microseconds"] chan3_raw: u16,
    #[doc = "RC channel 4 value, in microseconds"] chan4_raw: u16,
    #[doc = "RC channel 5 value, in microseconds"] chan5_raw: u16,
    #[doc = "RC channel 6 value, in microseconds"] chan6_raw: u16,
    #[doc = "RC channel 7 value, in microseconds"] chan7_raw: u16,
    #[doc = "RC channel 8 value, in microseconds"] chan8_raw: u16,
    #[doc = "RC channel 9 value, in microseconds"] chan9_raw: u16,
    #[doc = "RC channel 10 value, in microseconds"] chan10_raw: u16,
    #[doc = "RC channel 11 value, in microseconds"] chan11_raw: u16,
    #[doc = "RC channel 12 value, in microseconds"] chan12_raw: u16,
    #[doc = "Receive signal strength indicator, 0: 0%, 255: 100%"] rssi: u8,
}
# [ doc = "Sent from autopilot to simulation. Hardware in the loop control outputs (replacement for HIL_CONTROLS)" ]
pub struct HIL_ACTUATOR_CONTROLS {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "Control outputs -1 .. 1. Channel assignment depends on the simulated hardware."]
    controls: [f32; 16usize],
    #[doc = "System mode (MAV_MODE), includes arming state."] mode: u8,
    #[doc = "Flags as bitfield, reserved for future use."] flags: u64,
}
#[doc = "Optical flow from a flow sensor (e.g. optical mouse sensor)"]
pub struct OPTICAL_FLOW { # [ doc = "Timestamp (UNIX)" ] time_usec : u64 , # [ doc = "Sensor ID" ] sensor_id : u8 , # [ doc = "Flow in pixels * 10 in x-sensor direction (dezi-pixels)" ] flow_x : i16 , # [ doc = "Flow in pixels * 10 in y-sensor direction (dezi-pixels)" ] flow_y : i16 , # [ doc = "Flow in meters in x-sensor direction, angular-speed compensated" ] flow_comp_m_x : f32 , # [ doc = "Flow in meters in y-sensor direction, angular-speed compensated" ] flow_comp_m_y : f32 , # [ doc = "Optical flow quality / confidence. 0: bad, 255: maximum quality" ] quality : u8 , # [ doc = "Ground distance in meters. Positive value: distance known. Negative value: Unknown distance" ] ground_distance : f32 , # [ doc = "Flow rate in radians/second about X axis" ] flow_rate_x : f32 , # [ doc = "Flow rate in radians/second about Y axis" ] flow_rate_y : f32 , }
#[doc = "Timestamp (microseconds, synced to UNIX time or since system boot)"]
pub struct GLOBAL_VISION_POSITION_ESTIMATE {
    #[doc = "Global X position"] x: f32,
    #[doc = "Global Y position"] y: f32,
    #[doc = "Global Z position"] z: f32,
    #[doc = "Roll angle in rad"] roll: f32,
    #[doc = "Pitch angle in rad"] pitch: f32,
    #[doc = "Yaw angle in rad"] yaw: f32,
}
#[doc = "Timestamp (microseconds, synced to UNIX time or since system boot)"]
pub struct VISION_POSITION_ESTIMATE {
    #[doc = "Global X position"] x: f32,
    #[doc = "Global Y position"] y: f32,
    #[doc = "Global Z position"] z: f32,
    #[doc = "Roll angle in rad"] roll: f32,
    #[doc = "Pitch angle in rad"] pitch: f32,
    #[doc = "Yaw angle in rad"] yaw: f32,
}
#[doc = "Timestamp (microseconds, synced to UNIX time or since system boot)"]
pub struct VISION_SPEED_ESTIMATE {
    #[doc = "Global X speed"] x: f32,
    #[doc = "Global Y speed"] y: f32,
    #[doc = "Global Z speed"] z: f32,
}
#[doc = "Timestamp (microseconds, synced to UNIX time or since system boot)"]
pub struct VICON_POSITION_ESTIMATE {
    #[doc = "Global X position"] x: f32,
    #[doc = "Global Y position"] y: f32,
    #[doc = "Global Z position"] z: f32,
    #[doc = "Roll angle in rad"] roll: f32,
    #[doc = "Pitch angle in rad"] pitch: f32,
    #[doc = "Yaw angle in rad"] yaw: f32,
}
#[doc = "The IMU readings in SI units in NED body frame"]
pub struct HIGHRES_IMU { # [ doc = "Timestamp (microseconds, synced to UNIX time or since system boot)" ] time_usec : u64 , # [ doc = "X acceleration (m/s^2)" ] xacc : f32 , # [ doc = "Y acceleration (m/s^2)" ] yacc : f32 , # [ doc = "Z acceleration (m/s^2)" ] zacc : f32 , # [ doc = "Angular speed around X axis (rad / sec)" ] xgyro : f32 , # [ doc = "Angular speed around Y axis (rad / sec)" ] ygyro : f32 , # [ doc = "Angular speed around Z axis (rad / sec)" ] zgyro : f32 , # [ doc = "X Magnetic field (Gauss)" ] xmag : f32 , # [ doc = "Y Magnetic field (Gauss)" ] ymag : f32 , # [ doc = "Z Magnetic field (Gauss)" ] zmag : f32 , # [ doc = "Absolute pressure in millibar" ] abs_pressure : f32 , # [ doc = "Differential pressure in millibar" ] diff_pressure : f32 , # [ doc = "Altitude calculated from pressure" ] pressure_alt : f32 , # [ doc = "Temperature in degrees celsius" ] temperature : f32 , # [ doc = "Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature" ] fields_updated : u16 , }
#[doc = "Optical flow from an angular rate flow sensor (e.g. PX4FLOW or mouse sensor)"]
pub struct OPTICAL_FLOW_RAD { # [ doc = "Timestamp (microseconds, synced to UNIX time or since system boot)" ] time_usec : u64 , # [ doc = "Sensor ID" ] sensor_id : u8 , # [ doc = "Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the." ] integration_time_us : u32 , # [ doc = "Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)" ] integrated_x : f32 , # [ doc = "Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)" ] integrated_y : f32 , # [ doc = "RH rotation around X axis (rad)" ] integrated_xgyro : f32 , # [ doc = "RH rotation around Y axis (rad)" ] integrated_ygyro : f32 , # [ doc = "RH rotation around Z axis (rad)" ] integrated_zgyro : f32 , # [ doc = "Temperature * 100 in centi-degrees Celsius" ] temperature : i16 , # [ doc = "Optical flow quality / confidence. 0: no valid flow, 255: maximum quality" ] quality : u8 , # [ doc = "Time in microseconds since the distance was sampled." ] time_delta_distance_us : u32 , # [ doc = "Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance." ] distance : f32 , }
#[doc = "The IMU readings in SI units in NED body frame"]
pub struct HIL_SENSOR { # [ doc = "Timestamp (microseconds, synced to UNIX time or since system boot)" ] time_usec : u64 , # [ doc = "X acceleration (m/s^2)" ] xacc : f32 , # [ doc = "Y acceleration (m/s^2)" ] yacc : f32 , # [ doc = "Z acceleration (m/s^2)" ] zacc : f32 , # [ doc = "Angular speed around X axis in body frame (rad / sec)" ] xgyro : f32 , # [ doc = "Angular speed around Y axis in body frame (rad / sec)" ] ygyro : f32 , # [ doc = "Angular speed around Z axis in body frame (rad / sec)" ] zgyro : f32 , # [ doc = "X Magnetic field (Gauss)" ] xmag : f32 , # [ doc = "Y Magnetic field (Gauss)" ] ymag : f32 , # [ doc = "Z Magnetic field (Gauss)" ] zmag : f32 , # [ doc = "Absolute pressure in millibar" ] abs_pressure : f32 , # [ doc = "Differential pressure (airspeed) in millibar" ] diff_pressure : f32 , # [ doc = "Altitude calculated from pressure" ] pressure_alt : f32 , # [ doc = "Temperature in degrees celsius" ] temperature : f32 , # [ doc = "Bitmask for fields that have updated since last message, bit 0 = xacc, bit 12: temperature, bit 31: full reset of attitude/position/velocities/etc was performed in sim." ] fields_updated : u32 , }
#[doc = "Status of simulation environment, if used"]
pub struct SIM_STATE { # [ doc = "True attitude quaternion component 1, w (1 in null-rotation)" ] q1 : f32 , # [ doc = "True attitude quaternion component 2, x (0 in null-rotation)" ] q2 : f32 , # [ doc = "True attitude quaternion component 3, y (0 in null-rotation)" ] q3 : f32 , # [ doc = "True attitude quaternion component 4, z (0 in null-rotation)" ] q4 : f32 , # [ doc = "Attitude roll expressed as Euler angles, not recommended except for human-readable outputs" ] roll : f32 , # [ doc = "Attitude pitch expressed as Euler angles, not recommended except for human-readable outputs" ] pitch : f32 , # [ doc = "Attitude yaw expressed as Euler angles, not recommended except for human-readable outputs" ] yaw : f32 , # [ doc = "X acceleration m/s/s" ] xacc : f32 , # [ doc = "Y acceleration m/s/s" ] yacc : f32 , # [ doc = "Z acceleration m/s/s" ] zacc : f32 , # [ doc = "Angular speed around X axis rad/s" ] xgyro : f32 , # [ doc = "Angular speed around Y axis rad/s" ] ygyro : f32 , # [ doc = "Angular speed around Z axis rad/s" ] zgyro : f32 , # [ doc = "Latitude in degrees" ] lat : f32 , # [ doc = "Longitude in degrees" ] lon : f32 , # [ doc = "Altitude in meters" ] alt : f32 , # [ doc = "Horizontal position standard deviation" ] std_dev_horz : f32 , # [ doc = "Vertical position standard deviation" ] std_dev_vert : f32 , # [ doc = "True velocity in m/s in NORTH direction in earth-fixed NED frame" ] vn : f32 , # [ doc = "True velocity in m/s in EAST direction in earth-fixed NED frame" ] ve : f32 , # [ doc = "True velocity in m/s in DOWN direction in earth-fixed NED frame" ] vd : f32 , }
#[doc = "Status generated by radio and injected into MAVLink stream."]
pub struct RADIO_STATUS {
    #[doc = "Local signal strength"] rssi: u8,
    #[doc = "Remote signal strength"] remrssi: u8,
    #[doc = "Remaining free buffer space in percent."] txbuf: u8,
    #[doc = "Background noise level"] noise: u8,
    #[doc = "Remote background noise level"] remnoise: u8,
    #[doc = "Receive errors"] rxerrors: u16,
    #[doc = "Count of error corrected packets"] fixed: u16,
}
#[doc = "File transfer message"]
pub struct FILE_TRANSFER_PROTOCOL { # [ doc = "Network ID (0 for broadcast)" ] target_network : u8 , # [ doc = "System ID (0 for broadcast)" ] target_system : u8 , # [ doc = "Component ID (0 for broadcast)" ] target_component : u8 , # [ doc = "Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification." ] payload : [ i8 ; 251usize ] , }
#[doc = "Time synchronization message."]
pub struct TIMESYNC {
    #[doc = "Time sync timestamp 1"] tc1: i64,
    #[doc = "Time sync timestamp 2"] ts1: i64,
}
#[doc = "Camera-IMU triggering and synchronisation message."]
pub struct CAMERA_TRIGGER {
    #[doc = "Timestamp for the image frame in microseconds"] time_usec: u64,
    #[doc = "Image frame sequence"] seq: u32,
}
# [ doc = "The global position, as returned by the Global Positioning System (GPS). This is\n                 NOT the global position estimate of the sytem, but rather a RAW sensor value. See message GLOBAL_POSITION for the global position estimate. Coordinate frame is right-handed, Z-axis up (GPS frame)." ]
pub struct HIL_GPS { # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , # [ doc = "0-1: no fix, 2: 2D fix, 3: 3D fix. Some applications will not use the value of this field unless it is at least two, so always correctly fill in the fix." ] fix_type : u8 , # [ doc = "Latitude (WGS84), in degrees * 1E7" ] lat : i32 , # [ doc = "Longitude (WGS84), in degrees * 1E7" ] lon : i32 , # [ doc = "Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)" ] alt : i32 , # [ doc = "GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: 65535" ] eph : u16 , # [ doc = "GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: 65535" ] epv : u16 , # [ doc = "GPS ground speed in cm/s. If unknown, set to: 65535" ] vel : u16 , # [ doc = "GPS velocity in cm/s in NORTH direction in earth-fixed NED frame" ] vn : i16 , # [ doc = "GPS velocity in cm/s in EAST direction in earth-fixed NED frame" ] ve : i16 , # [ doc = "GPS velocity in cm/s in DOWN direction in earth-fixed NED frame" ] vd : i16 , # [ doc = "Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: 65535" ] cog : u16 , # [ doc = "Number of satellites visible. If unknown, set to 255" ] satellites_visible : u8 , }
#[doc = "Simulated optical flow from a flow sensor (e.g. PX4FLOW or optical mouse sensor)"]
pub struct HIL_OPTICAL_FLOW { # [ doc = "Timestamp (microseconds, synced to UNIX time or since system boot)" ] time_usec : u64 , # [ doc = "Sensor ID" ] sensor_id : u8 , # [ doc = "Integration time in microseconds. Divide integrated_x and integrated_y by the integration time to obtain average flow. The integration time also indicates the." ] integration_time_us : u32 , # [ doc = "Flow in radians around X axis (Sensor RH rotation about the X axis induces a positive flow. Sensor linear motion along the positive Y axis induces a negative flow.)" ] integrated_x : f32 , # [ doc = "Flow in radians around Y axis (Sensor RH rotation about the Y axis induces a positive flow. Sensor linear motion along the positive X axis induces a positive flow.)" ] integrated_y : f32 , # [ doc = "RH rotation around X axis (rad)" ] integrated_xgyro : f32 , # [ doc = "RH rotation around Y axis (rad)" ] integrated_ygyro : f32 , # [ doc = "RH rotation around Z axis (rad)" ] integrated_zgyro : f32 , # [ doc = "Temperature * 100 in centi-degrees Celsius" ] temperature : i16 , # [ doc = "Optical flow quality / confidence. 0: no valid flow, 255: maximum quality" ] quality : u8 , # [ doc = "Time in microseconds since the distance was sampled." ] time_delta_distance_us : u32 , # [ doc = "Distance to the center of the flow field in meters. Positive value (including zero): distance known. Negative value: Unknown distance." ] distance : f32 , }
# [ doc = "Sent from simulation to autopilot, avoids in contrast to HIL_STATE singularities. This packet is useful for high throughput applications such as hardware in the loop simulations." ]
pub struct HIL_STATE_QUATERNION { # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , # [ doc = "Vehicle attitude expressed as normalized quaternion in w, x, y, z order (with 1 0 0 0 being the null-rotation)" ] attitude_quaternion : [ f32 ; 4usize ] , # [ doc = "Body frame roll / phi angular speed (rad/s)" ] rollspeed : f32 , # [ doc = "Body frame pitch / theta angular speed (rad/s)" ] pitchspeed : f32 , # [ doc = "Body frame yaw / psi angular speed (rad/s)" ] yawspeed : f32 , # [ doc = "Latitude, expressed as degrees * 1E7" ] lat : i32 , # [ doc = "Longitude, expressed as degrees * 1E7" ] lon : i32 , # [ doc = "Altitude in meters, expressed as * 1000 (millimeters)" ] alt : i32 , # [ doc = "Ground X Speed (Latitude), expressed as cm/s" ] vx : i16 , # [ doc = "Ground Y Speed (Longitude), expressed as cm/s" ] vy : i16 , # [ doc = "Ground Z Speed (Altitude), expressed as cm/s" ] vz : i16 , # [ doc = "Indicated airspeed, expressed as cm/s" ] ind_airspeed : u16 , # [ doc = "True airspeed, expressed as cm/s" ] true_airspeed : u16 , # [ doc = "X acceleration (mg)" ] xacc : i16 , # [ doc = "Y acceleration (mg)" ] yacc : i16 , # [ doc = "Z acceleration (mg)" ] zacc : i16 , }
# [ doc = "The RAW IMU readings for secondary 9DOF sensor setup. This message should contain the scaled values to the described units" ]
pub struct SCALED_IMU2 {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "X acceleration (mg)"] xacc: i16,
    #[doc = "Y acceleration (mg)"] yacc: i16,
    #[doc = "Z acceleration (mg)"] zacc: i16,
    #[doc = "Angular speed around X axis (millirad /sec)"] xgyro: i16,
    #[doc = "Angular speed around Y axis (millirad /sec)"] ygyro: i16,
    #[doc = "Angular speed around Z axis (millirad /sec)"] zgyro: i16,
    #[doc = "X Magnetic field (milli tesla)"] xmag: i16,
    #[doc = "Y Magnetic field (milli tesla)"] ymag: i16,
    #[doc = "Z Magnetic field (milli tesla)"] zmag: i16,
}
# [ doc = "Request a list of available logs. On some systems calling this may stop on-board logging until LOG_REQUEST_END is called." ]
pub struct LOG_REQUEST_LIST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "First log id (0 for first available)"] start: u16,
    #[doc = "Last log id (0xffff for last available)"] end: u16,
}
#[doc = "Reply to LOG_REQUEST_LIST"]
pub struct LOG_ENTRY {
    #[doc = "Log id"] id: u16,
    #[doc = "Total number of logs"] num_logs: u16,
    #[doc = "High log number"] last_log_num: u16,
    #[doc = "UTC timestamp of log in seconds since 1970, or 0 if not available"] time_utc: u32,
    #[doc = "Size of the log (may be approximate) in bytes"] size: u32,
}
#[doc = "Request a chunk of a log"]
pub struct LOG_REQUEST_DATA {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "Log id (from LOG_ENTRY reply)"] id: u16,
    #[doc = "Offset into the log"] ofs: u32,
    #[doc = "Number of bytes"] count: u32,
}
#[doc = "Reply to LOG_REQUEST_DATA"]
pub struct LOG_DATA {
    #[doc = "Log id (from LOG_ENTRY reply)"] id: u16,
    #[doc = "Offset into the log"] ofs: u32,
    #[doc = "Number of bytes (zero for end of log)"] count: u8,
    #[doc = "log data"] data: [i8; 90usize],
}
#[doc = "Erase all logs"]
pub struct LOG_ERASE {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
}
#[doc = "Stop log transfer and resume normal logging"]
pub struct LOG_REQUEST_END {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
}
#[doc = "data for injecting into the onboard GPS (used for DGPS)"]
pub struct GPS_INJECT_DATA {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "data length"] len: u8,
    #[doc = "raw data (110 is enough for 12 satellites of RTCMv2)"] data: [i8; 110usize],
}
#[doc = "Second GPS data. Coordinate frame is right-handed, Z-axis up (GPS frame)."]
pub struct GPS2_RAW { # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , # [ doc = "See the GPS_FIX_TYPE enum." ] fix_type : u8 , # [ doc = "Latitude (WGS84), in degrees * 1E7" ] lat : i32 , # [ doc = "Longitude (WGS84), in degrees * 1E7" ] lon : i32 , # [ doc = "Altitude (AMSL, not WGS84), in meters * 1000 (positive for up)" ] alt : i32 , # [ doc = "GPS HDOP horizontal dilution of position in cm (m*100). If unknown, set to: UINT16_MAX" ] eph : u16 , # [ doc = "GPS VDOP vertical dilution of position in cm (m*100). If unknown, set to: UINT16_MAX" ] epv : u16 , # [ doc = "GPS ground speed (m/s * 100). If unknown, set to: UINT16_MAX" ] vel : u16 , # [ doc = "Course over ground (NOT heading, but direction of movement) in degrees * 100, 0.0..359.99 degrees. If unknown, set to: UINT16_MAX" ] cog : u16 , # [ doc = "Number of satellites visible. If unknown, set to 255" ] satellites_visible : u8 , # [ doc = "Number of DGPS satellites" ] dgps_numch : u8 , # [ doc = "Age of DGPS info" ] dgps_age : u32 , }
#[doc = "Power supply status"]
pub struct POWER_STATUS {
    #[doc = "5V rail voltage in millivolts"] Vcc: u16,
    #[doc = "servo rail voltage in millivolts"] Vservo: u16,
    #[doc = "power supply status flags (see MAV_POWER_STATUS enum)"] flags: u16,
}
# [ doc = "Control a serial port. This can be used for raw access to an onboard serial peripheral such as a GPS or telemetry radio. It is designed to make it possible to update the devices firmware via MAVLink messages or change the devices settings. A message with zero bytes can be used to change just the baudrate." ]
pub struct SERIAL_CONTROL {
    #[doc = "See SERIAL_CONTROL_DEV enum"] device: u8,
    #[doc = "See SERIAL_CONTROL_FLAG enum"] flags: u8,
    #[doc = "Timeout for reply data in milliseconds"] timeout: u16,
    #[doc = "Baudrate of transfer. Zero means no change."] baudrate: u32,
    #[doc = "how many bytes in this transfer"] count: u8,
    #[doc = "serial data"] data: [i8; 70usize],
}
#[doc = "RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting"]
pub struct GPS_RTK {
    #[doc = "Time since boot of last baseline message received in ms."] time_last_baseline_ms: u32,
    #[doc = "Identification of connected RTK receiver."] rtk_receiver_id: u8,
    #[doc = "GPS Week Number of last baseline"] wn: u16,
    #[doc = "GPS Time of Week of last baseline"] tow: u32,
    #[doc = "GPS-specific health report for RTK data."] rtk_health: u8,
    #[doc = "Rate of baseline messages being received by GPS, in HZ"] rtk_rate: u8,
    #[doc = "Current number of sats used for RTK calculation."] nsats: u8,
    #[doc = "Coordinate system of baseline"] baseline_coords_type: u8,
    #[doc = "Current baseline in ECEF x or NED north component in mm."] baseline_a_mm: i32,
    #[doc = "Current baseline in ECEF y or NED east component in mm."] baseline_b_mm: i32,
    #[doc = "Current baseline in ECEF z or NED down component in mm."] baseline_c_mm: i32,
    #[doc = "Current estimate of baseline accuracy."] accuracy: u32,
    #[doc = "Current number of integer ambiguity hypotheses."] iar_num_hypotheses: i32,
}
#[doc = "RTK GPS data. Gives information on the relative baseline calculation the GPS is reporting"]
pub struct GPS2_RTK {
    #[doc = "Time since boot of last baseline message received in ms."] time_last_baseline_ms: u32,
    #[doc = "Identification of connected RTK receiver."] rtk_receiver_id: u8,
    #[doc = "GPS Week Number of last baseline"] wn: u16,
    #[doc = "GPS Time of Week of last baseline"] tow: u32,
    #[doc = "GPS-specific health report for RTK data."] rtk_health: u8,
    #[doc = "Rate of baseline messages being received by GPS, in HZ"] rtk_rate: u8,
    #[doc = "Current number of sats used for RTK calculation."] nsats: u8,
    #[doc = "Coordinate system of baseline"] baseline_coords_type: u8,
    #[doc = "Current baseline in ECEF x or NED north component in mm."] baseline_a_mm: i32,
    #[doc = "Current baseline in ECEF y or NED east component in mm."] baseline_b_mm: i32,
    #[doc = "Current baseline in ECEF z or NED down component in mm."] baseline_c_mm: i32,
    #[doc = "Current estimate of baseline accuracy."] accuracy: u32,
    #[doc = "Current number of integer ambiguity hypotheses."] iar_num_hypotheses: i32,
}
# [ doc = "The RAW IMU readings for 3rd 9DOF sensor setup. This message should contain the scaled values to the described units" ]
pub struct SCALED_IMU3 {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "X acceleration (mg)"] xacc: i16,
    #[doc = "Y acceleration (mg)"] yacc: i16,
    #[doc = "Z acceleration (mg)"] zacc: i16,
    #[doc = "Angular speed around X axis (millirad /sec)"] xgyro: i16,
    #[doc = "Angular speed around Y axis (millirad /sec)"] ygyro: i16,
    #[doc = "Angular speed around Z axis (millirad /sec)"] zgyro: i16,
    #[doc = "X Magnetic field (milli tesla)"] xmag: i16,
    #[doc = "Y Magnetic field (milli tesla)"] ymag: i16,
    #[doc = "Z Magnetic field (milli tesla)"] zmag: i16,
}
# [ doc = "type of requested/acknowledged data (as defined in ENUM DATA_TYPES in mavlink/include/mavlink_types.h)" ]
pub struct DATA_TRANSMISSION_HANDSHAKE { # [ doc = "total data size in bytes (set on ACK only)" ] size : u32 , # [ doc = "Width of a matrix or image" ] width : u16 , # [ doc = "Height of a matrix or image" ] height : u16 , # [ doc = "number of packets beeing sent (set on ACK only)" ] packets : u16 , # [ doc = "payload size per packet (normally 253 byte, see DATA field size in message ENCAPSULATED_DATA) (set on ACK only)" ] payload : u8 , # [ doc = "JPEG quality out of [1,100]" ] jpg_quality : u8 , }
#[doc = "sequence number (starting with 0 on every transmission)"]
pub struct ENCAPSULATED_DATA {
    #[doc = "image data bytes"] data: [i8; 253usize],
}
#[doc = "Time since system boot"]
pub struct DISTANCE_SENSOR { # [ doc = "Minimum distance the sensor can measure in centimeters" ] min_distance : u16 , # [ doc = "Maximum distance the sensor can measure in centimeters" ] max_distance : u16 , # [ doc = "Current distance reading" ] current_distance : u16 , # [ doc = "Type from MAV_DISTANCE_SENSOR enum." ] typ : u8 , # [ doc = "Onboard ID of the sensor" ] id : u8 , # [ doc = "Direction the sensor faces from MAV_SENSOR_ORIENTATION enum. downward-facing: ROTATION_PITCH_270, upward-facing: ROTATION_PITCH_90, backward-facing: ROTATION_PITCH_180, forward-facing: ROTATION_NONE, left-facing: ROTATION_YAW_90, right-facing: ROTATION_YAW_270" ] orientation : u8 , # [ doc = "Measurement covariance in centimeters, 0 for unknown / invalid readings" ] covariance : u8 , }
#[doc = "Request for terrain data and terrain status"]
pub struct TERRAIN_REQUEST {
    #[doc = "Latitude of SW corner of first grid (degrees *10^7)"] lat: i32,
    #[doc = "Longitude of SW corner of first grid (in degrees *10^7)"] lon: i32,
    #[doc = "Grid spacing in meters"] grid_spacing: u16,
    #[doc = "Bitmask of requested 4x4 grids (row major 8x7 array of grids, 56 bits)"] mask: u64,
}
# [ doc = "Terrain data sent from GCS. The lat/lon and grid_spacing must be the same as a lat/lon from a TERRAIN_REQUEST" ]
pub struct TERRAIN_DATA {
    #[doc = "Latitude of SW corner of first grid (degrees *10^7)"] lat: i32,
    #[doc = "Longitude of SW corner of first grid (in degrees *10^7)"] lon: i32,
    #[doc = "Grid spacing in meters"] grid_spacing: u16,
    #[doc = "bit within the terrain request mask"] gridbit: u8,
    #[doc = "Terrain data in meters AMSL"] data: [i16; 16usize],
}
# [ doc = "Request that the vehicle report terrain height at the given location. Used by GCS to check if vehicle has all terrain data needed for a mission." ]
pub struct TERRAIN_CHECK {
    #[doc = "Latitude (degrees *10^7)"] lat: i32,
    #[doc = "Longitude (degrees *10^7)"] lon: i32,
}
#[doc = "Response from a TERRAIN_CHECK request"]
pub struct TERRAIN_REPORT {
    #[doc = "Latitude (degrees *10^7)"] lat: i32,
    #[doc = "Longitude (degrees *10^7)"] lon: i32,
    #[doc = "grid spacing (zero if terrain at this location unavailable)"] spacing: u16,
    #[doc = "Terrain height in meters AMSL"] terrain_height: f32,
    #[doc = "Current vehicle height above lat/lon terrain height (meters)"] current_height: f32,
    #[doc = "Number of 4x4 terrain blocks waiting to be received or read from disk"] pending: u16,
    #[doc = "Number of 4x4 terrain blocks in memory"] loaded: u16,
}
#[doc = "Barometer readings for 2nd barometer"]
pub struct SCALED_PRESSURE2 {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Absolute pressure (hectopascal)"] press_abs: f32,
    #[doc = "Differential pressure 1 (hectopascal)"] press_diff: f32,
    #[doc = "Temperature measurement (0.01 degrees celsius)"] temperature: i16,
}
#[doc = "Motion capture attitude and position"]
pub struct ATT_POS_MOCAP {
    #[doc = "Timestamp (micros since boot or Unix epoch)"] time_usec: u64,
    #[doc = "Attitude quaternion (w, x, y, z order, zero-rotation is 1, 0, 0, 0)"] q: [f32; 4usize],
    #[doc = "X position in meters (NED)"] x: f32,
    #[doc = "Y position in meters (NED)"] y: f32,
    #[doc = "Z position in meters (NED)"] z: f32,
}
#[doc = "Set the vehicle attitude and body angular rates."]
pub struct SET_ACTUATOR_CONTROL_TARGET { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "Actuator group. The \"_mlx\" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances." ] group_mlx : u8 , # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs." ] controls : [ f32 ; 8usize ] , }
#[doc = "Set the vehicle attitude and body angular rates."]
pub struct ACTUATOR_CONTROL_TARGET { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "Actuator group. The \"_mlx\" indicates this is a multi-instance message and a MAVLink parser should use this field to difference between instances." ] group_mlx : u8 , # [ doc = "Actuator controls. Normed to -1..+1 where 0 is neutral position. Throttle for single rotation direction motors is 0..1, negative range for reverse direction. Standard mapping for attitude controls (group 0): (index 0-7): roll, pitch, yaw, throttle, flaps, spoilers, airbrakes, landing gear. Load a pass-through mixer to repurpose them as generic outputs." ] controls : [ f32 ; 8usize ] , }
#[doc = "The current system altitude."]
pub struct ALTITUDE { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "This altitude measure is initialized on system boot and monotonic (it is never reset, but represents the local altitude change). The only guarantee on this field is that it will never be reset and is consistent within a flight. The recommended value for this field is the uncorrected barometric altitude at boot time. This altitude will also drift and vary between flights." ] altitude_monotonic : f32 , # [ doc = "This altitude measure is strictly above mean sea level and might be non-monotonic (it might reset on events like GPS lock or when a new QNH value is set). It should be the altitude to which global altitude waypoints are compared to. Note that it is *not* the GPS altitude, however, most GPS modules already output AMSL by default and not the WGS84 altitude." ] altitude_amsl : f32 , # [ doc = "This is the local altitude in the local coordinate frame. It is not the altitude above home, but in reference to the coordinate origin (0, 0, 0). It is up-positive." ] altitude_local : f32 , # [ doc = "This is the altitude above the home position. It resets on each change of the current home position." ] altitude_relative : f32 , # [ doc = "This is the altitude above terrain. It might be fed by a terrain database or an altimeter. Values smaller than -1000 should be interpreted as unknown." ] altitude_terrain : f32 , # [ doc = "This is not the altitude, but the clear space below the system according to the fused clearance estimate. It generally should max out at the maximum range of e.g. the laser altimeter. It is generally a moving target. A negative value indicates no measurement available." ] bottom_clearance : f32 , }
#[doc = "The autopilot is requesting a resource (file, binary, other type of data)"]
pub struct RESOURCE_REQUEST { # [ doc = "Request ID. This ID should be re-used when sending back URI contents" ] request_id : u8 , # [ doc = "The type of requested URI. 0 = a file via URL. 1 = a UAVCAN binary" ] uri_type : u8 , # [ doc = "The requested unique resource identifier (URI). It is not necessarily a straight domain name (depends on the URI type enum)" ] uri : [ i8 ; 120usize ] , # [ doc = "The way the autopilot wants to receive the URI. 0 = MAVLink FTP. 1 = binary stream." ] transfer_type : u8 , # [ doc = "The storage path the autopilot wants the URI to be stored in. Will only be valid if the transfer_type has a storage associated (e.g. MAVLink FTP)." ] storage : [ i8 ; 120usize ] , }
#[doc = "Barometer readings for 3rd barometer"]
pub struct SCALED_PRESSURE3 {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Absolute pressure (hectopascal)"] press_abs: f32,
    #[doc = "Differential pressure 1 (hectopascal)"] press_diff: f32,
    #[doc = "Temperature measurement (0.01 degrees celsius)"] temperature: i16,
}
#[doc = "current motion information from a designated system"]
pub struct FOLLOW_TARGET { # [ doc = "Timestamp in milliseconds since system boot" ] timestamp : u64 , # [ doc = "bit positions for tracker reporting capabilities (POS = 0, VEL = 1, ACCEL = 2, ATT + RATES = 3)" ] est_capabilities : u8 , # [ doc = "Latitude (WGS84), in degrees * 1E7" ] lat : i32 , # [ doc = "Longitude (WGS84), in degrees * 1E7" ] lon : i32 , # [ doc = "AMSL, in meters" ] alt : f32 , # [ doc = "target velocity (0,0,0) for unknown" ] vel : [ f32 ; 3usize ] , # [ doc = "linear target acceleration (0,0,0) for unknown" ] acc : [ f32 ; 3usize ] , # [ doc = "(1 0 0 0 for unknown)" ] attitude_q : [ f32 ; 4usize ] , # [ doc = "(0 0 0 for unknown)" ] rates : [ f32 ; 3usize ] , # [ doc = "eph epv" ] position_cov : [ f32 ; 3usize ] , # [ doc = "button states or switches of a tracker device" ] custom_state : u64 , }
#[doc = "The smoothed, monotonic system state used to feed the control loops of the system."]
pub struct CONTROL_SYSTEM_STATE {
    #[doc = "Timestamp (micros since boot or Unix epoch)"] time_usec: u64,
    #[doc = "X acceleration in body frame"] x_acc: f32,
    #[doc = "Y acceleration in body frame"] y_acc: f32,
    #[doc = "Z acceleration in body frame"] z_acc: f32,
    #[doc = "X velocity in body frame"] x_vel: f32,
    #[doc = "Y velocity in body frame"] y_vel: f32,
    #[doc = "Z velocity in body frame"] z_vel: f32,
    #[doc = "X position in local frame"] x_pos: f32,
    #[doc = "Y position in local frame"] y_pos: f32,
    #[doc = "Z position in local frame"] z_pos: f32,
    #[doc = "Airspeed, set to -1 if unknown"] airspeed: f32,
    #[doc = "Variance of body velocity estimate"] vel_variance: [f32; 3usize],
    #[doc = "Variance in local position"] pos_variance: [f32; 3usize],
    #[doc = "The attitude, represented as Quaternion"] q: [f32; 4usize],
    #[doc = "Angular rate in roll axis"] roll_rate: f32,
    #[doc = "Angular rate in pitch axis"] pitch_rate: f32,
    #[doc = "Angular rate in yaw axis"] yaw_rate: f32,
}
#[doc = "Battery information"]
pub struct BATTERY_STATUS { # [ doc = "Battery ID" ] id : u8 , # [ doc = "Function of the battery" ] battery_function : u8 , # [ doc = "Type (chemistry) of the battery" ] typ : u8 , # [ doc = "Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature." ] temperature : i16 , # [ doc = "Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value." ] voltages : [ i16 ; 10usize ] , # [ doc = "Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current" ] current_battery : i16 , # [ doc = "Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate" ] current_consumed : i32 , # [ doc = "Consumed energy, in HectoJoules (intergrated U*I*dt)  (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate" ] energy_consumed : i32 , # [ doc = "Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery" ] battery_remaining : i8 , }
#[doc = "Version and capability of autopilot software"]
pub struct AUTOPILOT_VERSION { # [ doc = "bitmask of capabilities (see MAV_PROTOCOL_CAPABILITY enum)" ] capabilities : u64 , # [ doc = "Firmware version number" ] flight_sw_version : u32 , # [ doc = "Middleware version number" ] middleware_sw_version : u32 , # [ doc = "Operating system version number" ] os_sw_version : u32 , # [ doc = "HW / board version (last 8 bytes should be silicon ID, if any)" ] board_version : u32 , # [ doc = "Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases." ] flight_custom_version : [ i8 ; 8usize ] , # [ doc = "Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases." ] middleware_custom_version : [ i8 ; 8usize ] , # [ doc = "Custom version field, commonly the first 8 bytes of the git hash. This is not an unique identifier, but should allow to identify the commit using the main version number even for very large code bases." ] os_custom_version : [ i8 ; 8usize ] , # [ doc = "ID of the board vendor" ] vendor_id : u16 , # [ doc = "ID of the product" ] product_id : u16 , # [ doc = "UID if provided by hardware (see uid2)" ] uid : u64 , # [ doc = "UID if provided by hardware (supersedes the uid field. If this is non-zero, use this field, otherwise use uid)" ] uid2 : [ i8 ; 18usize ] , }
#[doc = "The location of a landing area captured from a downward facing camera"]
pub struct LANDING_TARGET { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "The ID of the target if multiple targets are present" ] target_num : u8 , # [ doc = "MAV_FRAME enum specifying the whether the following feilds are earth-frame, body-frame, etc." ] frame : u8 , # [ doc = "X-axis angular offset (in radians) of the target from the center of the image" ] angle_x : f32 , # [ doc = "Y-axis angular offset (in radians) of the target from the center of the image" ] angle_y : f32 , # [ doc = "Distance to the target from the vehicle in meters" ] distance : f32 , # [ doc = "Size in radians of target along x-axis" ] size_x : f32 , # [ doc = "Size in radians of target along y-axis" ] size_y : f32 , # [ doc = "X Position of the landing target on MAV_FRAME" ] x : f32 , # [ doc = "Y Position of the landing target on MAV_FRAME" ] y : f32 , # [ doc = "Z Position of the landing target on MAV_FRAME" ] z : f32 , # [ doc = "Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)" ] q : [ f32 ; 4usize ] , # [ doc = "LANDING_TARGET_TYPE enum specifying the type of landing target" ] typ : u8 , # [ doc = "Boolean indicating known position (1) or default unkown position (0), for validation of positioning of the landing target" ] position_valid : u8 , }
# [ doc = "Estimator status message including flags, innovation test ratios and estimated accuracies. The flags message is an integer bitmask containing information on which EKF outputs are valid. See the ESTIMATOR_STATUS_FLAGS enum definition for further information. The innovaton test ratios show the magnitude of the sensor innovation divided by the innovation check threshold. Under normal operation the innovaton test ratios should be below 0.5 with occasional values up to 1.0. Values greater than 1.0 should be rare under normal operation and indicate that a measurement has been rejected by the filter. The user should be notified if an innovation test ratio greater than 1.0 is recorded. Notifications for values in the range between 0.5 and 1.0 should be optional and controllable by the user." ]
pub struct ESTIMATOR_STATUS { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "Integer bitmask indicating which EKF outputs are valid. See definition for ESTIMATOR_STATUS_FLAGS." ] flags : u16 , # [ doc = "Velocity innovation test ratio" ] vel_ratio : f32 , # [ doc = "Horizontal position innovation test ratio" ] pos_horiz_ratio : f32 , # [ doc = "Vertical position innovation test ratio" ] pos_vert_ratio : f32 , # [ doc = "Magnetometer innovation test ratio" ] mag_ratio : f32 , # [ doc = "Height above terrain innovation test ratio" ] hagl_ratio : f32 , # [ doc = "True airspeed innovation test ratio" ] tas_ratio : f32 , # [ doc = "Horizontal position 1-STD accuracy relative to the EKF local origin (m)" ] pos_horiz_accuracy : f32 , # [ doc = "Vertical position 1-STD accuracy relative to the EKF local origin (m)" ] pos_vert_accuracy : f32 , }
#[doc = "Timestamp (micros since boot or Unix epoch)"]
pub struct WIND_COV {
    #[doc = "Wind in X (NED) direction in m/s"] wind_x: f32,
    #[doc = "Wind in Y (NED) direction in m/s"] wind_y: f32,
    #[doc = "Wind in Z (NED) direction in m/s"] wind_z: f32,
    #[doc = "Variability of the wind in XY. RMS of a 1 Hz lowpassed wind estimate."] var_horiz: f32,
    #[doc = "Variability of the wind in Z. RMS of a 1 Hz lowpassed wind estimate."] var_vert: f32,
    #[doc = "AMSL altitude (m) this measurement was taken at"] wind_alt: f32,
    #[doc = "Horizontal speed 1-STD accuracy"] horiz_accuracy: f32,
    #[doc = "Vertical speed 1-STD accuracy"] vert_accuracy: f32,
}
# [ doc = "GPS sensor input message.  This is a raw sensor value sent by the GPS. This is NOT the global position estimate of the sytem." ]
pub struct GPS_INPUT { # [ doc = "Timestamp (micros since boot or Unix epoch)" ] time_usec : u64 , # [ doc = "ID of the GPS for multiple GPS inputs" ] gps_id : u8 , # [ doc = "Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).  All other fields must be provided." ] ignore_flags : u16 , # [ doc = "GPS time (milliseconds from start of GPS week)" ] time_week_ms : u32 , # [ doc = "GPS week number" ] time_week : u16 , # [ doc = "0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK" ] fix_type : u8 , # [ doc = "Latitude (WGS84), in degrees * 1E7" ] lat : i32 , # [ doc = "Longitude (WGS84), in degrees * 1E7" ] lon : i32 , # [ doc = "Altitude (AMSL, not WGS84), in m (positive for up)" ] alt : f32 , # [ doc = "GPS HDOP horizontal dilution of position in m" ] hdop : f32 , # [ doc = "GPS VDOP vertical dilution of position in m" ] vdop : f32 , # [ doc = "GPS velocity in m/s in NORTH direction in earth-fixed NED frame" ] vn : f32 , # [ doc = "GPS velocity in m/s in EAST direction in earth-fixed NED frame" ] ve : f32 , # [ doc = "GPS velocity in m/s in DOWN direction in earth-fixed NED frame" ] vd : f32 , # [ doc = "GPS speed accuracy in m/s" ] speed_accuracy : f32 , # [ doc = "GPS horizontal accuracy in m" ] horiz_accuracy : f32 , # [ doc = "GPS vertical accuracy in m" ] vert_accuracy : f32 , # [ doc = "Number of satellites visible." ] satellites_visible : u8 , }
#[doc = "RTCM message for injecting into the onboard GPS (used for DGPS)"]
pub struct GPS_RTCM_DATA { # [ doc = "LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn\'t corrupt RTCM data, and to recover from a unreliable transport delivery order." ] flags : u8 , # [ doc = "data length" ] len : u8 , # [ doc = "RTCM message (may be fragmented)" ] data : [ i8 ; 180usize ] , }
#[doc = "Message appropriate for high latency connections like Iridium"]
pub struct HIGH_LATENCY { # [ doc = "System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h" ] base_mode : u8 , # [ doc = "A bitfield for use for autopilot-specific flags." ] custom_mode : u32 , # [ doc = "The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown." ] landed_state : u8 , # [ doc = "roll (centidegrees)" ] roll : i16 , # [ doc = "pitch (centidegrees)" ] pitch : i16 , # [ doc = "heading (centidegrees)" ] heading : u16 , # [ doc = "throttle (percentage)" ] throttle : i8 , # [ doc = "heading setpoint (centidegrees)" ] heading_sp : i16 , # [ doc = "Latitude, expressed as degrees * 1E7" ] latitude : i32 , # [ doc = "Longitude, expressed as degrees * 1E7" ] longitude : i32 , # [ doc = "Altitude above mean sea level (meters)" ] altitude_amsl : i16 , # [ doc = "Altitude setpoint relative to the home position (meters)" ] altitude_sp : i16 , # [ doc = "airspeed (m/s)" ] airspeed : u8 , # [ doc = "airspeed setpoint (m/s)" ] airspeed_sp : u8 , # [ doc = "groundspeed (m/s)" ] groundspeed : u8 , # [ doc = "climb rate (m/s)" ] climb_rate : i8 , # [ doc = "Number of satellites visible. If unknown, set to 255" ] gps_nsat : u8 , # [ doc = "See the GPS_FIX_TYPE enum." ] gps_fix_type : u8 , # [ doc = "Remaining battery (percentage)" ] battery_remaining : u8 , # [ doc = "Autopilot temperature (degrees C)" ] temperature : i8 , # [ doc = "Air temperature (degrees C) from airspeed sensor" ] temperature_air : i8 , # [ doc = "failsafe (each bit represents a failsafe where 0=ok, 1=failsafe active (bit0:RC, bit1:batt, bit2:GPS, bit3:GCS, bit4:fence)" ] failsafe : u8 , # [ doc = "current waypoint number" ] wp_num : u8 , # [ doc = "distance to target (meters)" ] wp_distance : u16 , }
#[doc = "WIP: Message appropriate for high latency connections like Iridium (version 2)"]
pub struct HIGH_LATENCY2 { # [ doc = "Timestamp (milliseconds since boot or Unix epoch)" ] timestamp : u32 , # [ doc = "Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)" ] typ : u8 , # [ doc = "Autopilot type / class. defined in MAV_AUTOPILOT ENUM" ] autopilot : u8 , # [ doc = "A bitfield for use for autopilot-specific flags (2 byte version)." ] custom_mode : u16 , # [ doc = "Latitude, expressed as degrees * 1E7" ] latitude : i32 , # [ doc = "Longitude, expressed as degrees * 1E7" ] longitude : i32 , # [ doc = "Altitude above mean sea level" ] altitude : i16 , # [ doc = "Altitude setpoint" ] target_altitude : i16 , # [ doc = "Heading (degrees / 2)" ] heading : u8 , # [ doc = "Heading setpoint (degrees / 2)" ] target_heading : u8 , # [ doc = "Distance to target waypoint or position (meters / 10)" ] target_distance : u16 , # [ doc = "Throttle (percentage)" ] throttle : u8 , # [ doc = "Airspeed (m/s * 5)" ] airspeed : u8 , # [ doc = "Airspeed setpoint (m/s * 5)" ] airspeed_sp : u8 , # [ doc = "Groundspeed (m/s * 5)" ] groundspeed : u8 , # [ doc = "Windspeed (m/s * 5)" ] windspeed : u8 , # [ doc = "Wind heading (deg / 2)" ] wind_heading : u8 , # [ doc = "Maximum error horizontal position since last message (m * 10)" ] eph : u8 , # [ doc = "Maximum error vertical position since last message (m * 10)" ] epv : u8 , # [ doc = "Air temperature (degrees C) from airspeed sensor" ] temperature_air : i8 , # [ doc = "Maximum climb rate magnitude since last message (m/s * 10)" ] climb_rate : i8 , # [ doc = "Battery (percentage, -1 for DNU)" ] battery : i8 , # [ doc = "Current waypoint number" ] wp_num : u16 , # [ doc = "Indicates failures as defined in HL_FAILURE_FLAG ENUM. " ] failure_flags : u16 , # [ doc = "Field for custom payload." ] custom0 : i8 , # [ doc = "Field for custom payload." ] custom1 : i8 , # [ doc = "Field for custom payload." ] custom2 : i8 , }
#[doc = "Vibration levels and accelerometer clipping"]
pub struct VIBRATION {
    #[doc = "Timestamp (micros since boot or Unix epoch)"] time_usec: u64,
    #[doc = "Vibration levels on X-axis"] vibration_x: f32,
    #[doc = "Vibration levels on Y-axis"] vibration_y: f32,
    #[doc = "Vibration levels on Z-axis"] vibration_z: f32,
    #[doc = "first accelerometer clipping count"] clipping_0: u32,
    #[doc = "second accelerometer clipping count"] clipping_1: u32,
    #[doc = "third accelerometer clipping count"] clipping_2: u32,
}
# [ doc = "This message can be requested by sending the MAV_CMD_GET_HOME_POSITION command. The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The position the system will return to and land on. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector." ]
pub struct HOME_POSITION { # [ doc = "Latitude (WGS84), in degrees * 1E7" ] latitude : i32 , # [ doc = "Longitude (WGS84, in degrees * 1E7" ] longitude : i32 , # [ doc = "Altitude (AMSL), in meters * 1000 (positive for up)" ] altitude : i32 , # [ doc = "Local X position of this position in the local coordinate frame" ] x : f32 , # [ doc = "Local Y position of this position in the local coordinate frame" ] y : f32 , # [ doc = "Local Z position of this position in the local coordinate frame" ] z : f32 , # [ doc = "World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground" ] q : [ f32 ; 4usize ] , # [ doc = "Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_x : f32 , # [ doc = "Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_y : f32 , # [ doc = "Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_z : f32 , # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , }
# [ doc = "The position the system will return to and land on. The position is set automatically by the system during the takeoff in case it was not explicitely set by the operator before or after. The global and local positions encode the position in the respective coordinate frames, while the q parameter encodes the orientation of the surface. Under normal conditions it describes the heading and terrain slope, which can be used by the aircraft to adjust the approach. The approach 3D vector describes the point to which the system should fly in normal flight mode and then perform a landing sequence along the vector." ]
pub struct SET_HOME_POSITION { # [ doc = "System ID." ] target_system : u8 , # [ doc = "Latitude (WGS84), in degrees * 1E7" ] latitude : i32 , # [ doc = "Longitude (WGS84, in degrees * 1E7" ] longitude : i32 , # [ doc = "Altitude (AMSL), in meters * 1000 (positive for up)" ] altitude : i32 , # [ doc = "Local X position of this position in the local coordinate frame" ] x : f32 , # [ doc = "Local Y position of this position in the local coordinate frame" ] y : f32 , # [ doc = "Local Z position of this position in the local coordinate frame" ] z : f32 , # [ doc = "World to surface normal and heading transformation of the takeoff position. Used to indicate the heading and slope of the ground" ] q : [ f32 ; 4usize ] , # [ doc = "Local X position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_x : f32 , # [ doc = "Local Y position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_y : f32 , # [ doc = "Local Z position of the end of the approach vector. Multicopters should set this position based on their takeoff path. Grass-landing fixed wing aircraft should set it the same way as multicopters. Runway-landing fixed wing aircraft should set it to the opposite direction of the takeoff, assuming the takeoff happened from the threshold / touchdown zone." ] approach_z : f32 , # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , }
#[doc = "This interface replaces DATA_STREAM"]
pub struct MESSAGE_INTERVAL { # [ doc = "The ID of the requested MAVLink message. v1.0 is limited to 254 messages." ] message_id : u16 , # [ doc = "The interval between two messages, in microseconds. A value of -1 indicates this stream is disabled, 0 indicates it is not available, > 0 indicates the interval at which it is sent." ] interval_us : i32 , }
#[doc = "Provides state for additional features"]
pub struct EXTENDED_SYS_STATE { # [ doc = "The VTOL state if applicable. Is set to MAV_VTOL_STATE_UNDEFINED if UAV is not in VTOL configuration." ] vtol_state : u8 , # [ doc = "The landed state. Is set to MAV_LANDED_STATE_UNDEFINED if landed state is unknown." ] landed_state : u8 , }
#[doc = "The location and information of an ADSB vehicle"]
pub struct ADSB_VEHICLE {
    #[doc = "ICAO address"] ICAO_address: u32,
    #[doc = "Latitude, expressed as degrees * 1E7"] lat: i32,
    #[doc = "Longitude, expressed as degrees * 1E7"] lon: i32,
    #[doc = "Type from ADSB_ALTITUDE_TYPE enum"] altitude_type: u8,
    #[doc = "Altitude(ASL) in millimeters"] altitude: i32,
    #[doc = "Course over ground in centidegrees"] heading: u16,
    #[doc = "The horizontal velocity in centimeters/second"] hor_velocity: u16,
    #[doc = "The vertical velocity in centimeters/second, positive is up"] ver_velocity: i16,
    #[doc = "The callsign, 8+null"] callsign: [char; 9usize],
    #[doc = "Type from ADSB_EMITTER_TYPE enum"] emitter_type: u8,
    #[doc = "Time since last communication in seconds"] tslc: u8,
    #[doc = "Flags to indicate various statuses including valid data fields"] flags: u16,
    #[doc = "Squawk code"] squawk: u16,
}
#[doc = "Information about a potential collision"]
pub struct COLLISION {
    #[doc = "Collision data source"] src: u8,
    #[doc = "Unique identifier, domain based on src field"] id: u32,
    #[doc = "Action that is being taken to avoid this collision"] action: u8,
    #[doc = "How concerned the aircraft is about this collision"] threat_level: u8,
    #[doc = "Estimated time until collision occurs (seconds)"] time_to_minimum_delta: f32,
    #[doc = "Closest vertical distance in meters between vehicle and object"]
    altitude_minimum_delta: f32,
    #[doc = "Closest horizontal distance in meteres between vehicle and object"]
    horizontal_minimum_delta: f32,
}
#[doc = "Message implementing parts of the V2 payload specs in V1 frames for transitional support."]
pub struct V2_EXTENSION { # [ doc = "Network ID (0 for broadcast)" ] target_network : u8 , # [ doc = "System ID (0 for broadcast)" ] target_system : u8 , # [ doc = "Component ID (0 for broadcast)" ] target_component : u8 , # [ doc = "A code that identifies the software component that understands this message (analogous to usb device classes or mime type strings).  If this code is less than 32768, it is considered a \'registered\' protocol extension and the corresponding entry should be added to https://github.com/mavlink/mavlink/extension-message-ids.xml.  Software creators can register blocks of message IDs as needed (useful for GCS specific metadata, etc...). Message_types greater than 32767 are considered local experiments and should not be checked in to any widely distributed codebase." ] message_type : u16 , # [ doc = "Variable length payload. The length is defined by the remaining message length when subtracting the header and other fields.  The entire content of this block is opaque unless you understand any the encoding message_type.  The particular encoding used can be extension specific and might not always be documented as part of the mavlink specification." ] payload : [ i8 ; 249usize ] , }
# [ doc = "Send raw controller memory. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output." ]
pub struct MEMORY_VECT { # [ doc = "Starting address of the debug variables" ] address : u16 , # [ doc = "Version code of the type variable. 0=unknown, type ignored and assumed int16_t. 1=as below" ] ver : u8 , # [ doc = "Type code of the memory variables. for ver = 1: 0=16 x int16_t, 1=16 x uint16_t, 2=16 x Q15, 3=16 x 1Q14" ] typ : u8 , # [ doc = "Memory contents at specified address" ] value : [ i8 ; 32usize ] , }
#[doc = "Name"]
pub struct DEBUG_VECT {
    #[doc = "Timestamp"] time_usec: u64,
    #[doc = "x"] x: f32,
    #[doc = "y"] y: f32,
    #[doc = "z"] z: f32,
}
# [ doc = "Send a key-value pair as float. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output." ]
pub struct NAMED_VALUE_FLOAT {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Name of the debug variable"] name: [char; 10usize],
    #[doc = "Floating point value"] value: f32,
}
# [ doc = "Send a key-value pair as integer. The use of this message is discouraged for normal packets, but a quite efficient way for testing new messages and getting experimental debug output." ]
pub struct NAMED_VALUE_INT {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Name of the debug variable"] name: [char; 10usize],
    #[doc = "Signed integer value"] value: i32,
}
# [ doc = "Status text message. These messages are printed in yellow in the COMM console of QGroundControl. WARNING: They consume quite some bandwidth, so use only for important status and error messages. If implemented wisely, these messages are buffered on the MCU and sent only at a limited rate (e.g. 10 Hz)." ]
pub struct STATUSTEXT {
    #[doc = "Severity of status. Relies on the definitions within RFC-5424. See enum MAV_SEVERITY."]
    severity: u8,
    #[doc = "Status text message, without null termination character"] text: [char; 50usize],
}
# [ doc = "Send a debug value. The index is used to discriminate between values. These values show up in the plot of QGroundControl as DEBUG N." ]
pub struct DEBUG {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "index of debug variable"] ind: u8,
    #[doc = "DEBUG value"] value: f32,
}
# [ doc = "Setup a MAVLink2 signing key. If called with secret_key of all zero and zero initial_timestamp will disable signing" ]
pub struct SETUP_SIGNING {
    #[doc = "system id of the target"] target_system: u8,
    #[doc = "component ID of the target"] target_component: u8,
    #[doc = "signing key"] secret_key: [i8; 32usize],
    #[doc = "initial timestamp"] initial_timestamp: u64,
}
#[doc = "Report button state change"]
pub struct BUTTON_CHANGE {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Time of last change of button state"] last_change_ms: u32,
    #[doc = "Bitmap state of buttons"] state: u8,
}
#[doc = "Control vehicle tone generation (buzzer)"]
pub struct PLAY_TUNE {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
    #[doc = "tune in board specific format"] tune: [char; 30usize],
}
#[doc = "Information about a camera"]
pub struct CAMERA_INFORMATION { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Name of the camera vendor" ] vendor_name : [ i8 ; 32usize ] , # [ doc = "Name of the camera model" ] model_name : [ i8 ; 32usize ] , # [ doc = "Version of the camera firmware (v << 24 & 0xff = Dev, v << 16 & 0xff = Patch, v << 8 & 0xff = Minor, v & 0xff = Major)" ] firmware_version : u32 , # [ doc = "Focal length in mm" ] focal_length : f32 , # [ doc = "Image sensor size horizontal in mm" ] sensor_size_h : f32 , # [ doc = "Image sensor size vertical in mm" ] sensor_size_v : f32 , # [ doc = "Image resolution in pixels horizontal" ] resolution_h : u16 , # [ doc = "Image resolution in pixels vertical" ] resolution_v : u16 , # [ doc = "Reserved for a lens ID" ] lens_id : u8 , # [ doc = "CAMERA_CAP_FLAGS enum flags (bitmap) describing camera capabilities." ] flags : u32 , # [ doc = "Camera definition version (iteration)" ] cam_definition_version : u16 , # [ doc = "Camera definition URI (if any, otherwise only basic functions will be available)." ] cam_definition_uri : [ char ; 140usize ] , }
#[doc = "Settings of a camera, can be requested using MAV_CMD_REQUEST_CAMERA_SETTINGS."]
pub struct CAMERA_SETTINGS {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Camera mode (CAMERA_MODE)"] mode_id: u8,
}
#[doc = "WIP: Information about a storage medium."]
pub struct STORAGE_INFORMATION {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Storage ID (1 for first, 2 for second, etc.)"] storage_id: u8,
    #[doc = "Number of storage devices"] storage_count: u8,
    #[doc = "Status of storage (0 not available, 1 unformatted, 2 formatted)"] status: u8,
    #[doc = "Total capacity in MiB"] total_capacity: f32,
    #[doc = "Used capacity in MiB"] used_capacity: f32,
    #[doc = "Available capacity in MiB"] available_capacity: f32,
    #[doc = "Read speed in MiB/s"] read_speed: f32,
    #[doc = "Write speed in MiB/s"] write_speed: f32,
}
#[doc = "Information about the status of a capture"]
pub struct CAMERA_CAPTURE_STATUS { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Current status of image capturing (0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress)" ] image_status : u8 , # [ doc = "Current status of video capturing (0: idle, 1: capture in progress)" ] video_status : u8 , # [ doc = "Image capture interval in seconds" ] image_interval : f32 , # [ doc = "Time in milliseconds since recording started" ] recording_time_ms : u32 , # [ doc = "Available storage capacity in MiB" ] available_capacity : f32 , }
#[doc = "Information about a captured image"]
pub struct CAMERA_IMAGE_CAPTURED { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Timestamp (microseconds since UNIX epoch) in UTC. 0 for unknown." ] time_utc : u64 , # [ doc = "Camera ID (1 for first, 2 for second, etc.)" ] camera_id : u8 , # [ doc = "Latitude, expressed as degrees * 1E7 where image was taken" ] lat : i32 , # [ doc = "Longitude, expressed as degrees * 1E7 where capture was taken" ] lon : i32 , # [ doc = "Altitude in meters, expressed as * 1E3 (AMSL, not WGS84) where image was taken" ] alt : i32 , # [ doc = "Altitude above ground in meters, expressed as * 1E3 where image was taken" ] relative_alt : i32 , # [ doc = "Quaternion of camera orientation (w, x, y, z order, zero-rotation is 0, 0, 0, 0)" ] q : [ f32 ; 4usize ] , # [ doc = "Zero based index of this image (image count since armed -1)" ] image_index : i32 , # [ doc = "Boolean indicating success (1) or failure (0) while capturing this image." ] capture_result : i8 , # [ doc = "URL of image taken. Either local storage or http://foo.jpg if camera provides an HTTP interface." ] file_url : [ char ; 205usize ] , }
#[doc = "WIP: Information about flight since last arming"]
pub struct FLIGHT_INFORMATION {
    #[doc = "Timestamp (milliseconds since system boot)"] time_boot_ms: u32,
    #[doc = "Timestamp at arming (microseconds since UNIX epoch) in UTC, 0 for unknown"]
    arming_time_utc: u64,
    #[doc = "Timestamp at takeoff (microseconds since UNIX epoch) in UTC, 0 for unknown"]
    takeoff_time_utc: u64,
    #[doc = "Universally unique identifier (UUID) of flight, should correspond to name of logfiles"]
    flight_uuid: u64,
}
#[doc = "Orientation of a mount"]
pub struct MOUNT_ORIENTATION { # [ doc = "Timestamp (milliseconds since system boot)" ] time_boot_ms : u32 , # [ doc = "Roll in global frame in degrees (set to NaN for invalid)." ] roll : f32 , # [ doc = "Pitch in global frame in degrees (set to NaN for invalid)." ] pitch : f32 , # [ doc = "Yaw relative to vehicle in degrees (set to NaN for invalid)." ] yaw : f32 , # [ doc = "Yaw in absolute frame in degrees, North is 0 (set to NaN for invalid)." ] yaw_absolute : f32 , }
#[doc = "A message containing logged data (see also MAV_CMD_LOGGING_START)"]
pub struct LOGGING_DATA { # [ doc = "system ID of the target" ] target_system : u8 , # [ doc = "component ID of the target" ] target_component : u8 , # [ doc = "sequence number (can wrap)" ] sequence : u16 , # [ doc = "data length" ] length : u8 , # [ doc = "offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists)." ] first_message_offset : u8 , # [ doc = "logged data" ] data : [ i8 ; 249usize ] , }
#[doc = "A message containing logged data which requires a LOGGING_ACK to be sent back"]
pub struct LOGGING_DATA_ACKED { # [ doc = "system ID of the target" ] target_system : u8 , # [ doc = "component ID of the target" ] target_component : u8 , # [ doc = "sequence number (can wrap)" ] sequence : u16 , # [ doc = "data length" ] length : u8 , # [ doc = "offset into data where first message starts. This can be used for recovery, when a previous message got lost (set to 255 if no start exists)." ] first_message_offset : u8 , # [ doc = "logged data" ] data : [ i8 ; 249usize ] , }
#[doc = "An ack for a LOGGING_DATA_ACKED message"]
pub struct LOGGING_ACK {
    #[doc = "system ID of the target"] target_system: u8,
    #[doc = "component ID of the target"] target_component: u8,
    #[doc = "sequence number (must match the one in LOGGING_DATA_ACKED)"] sequence: u16,
}
#[doc = "WIP: Information about video stream"]
pub struct VIDEO_STREAM_INFORMATION {
    #[doc = "Camera ID (1 for first, 2 for second, etc.)"] camera_id: u8,
    #[doc = "Current status of video streaming (0: not running, 1: in progress)"] status: u8,
    #[doc = "Frames per second"] framerate: f32,
    #[doc = "Resolution horizontal in pixels"] resolution_h: u16,
    #[doc = "Resolution vertical in pixels"] resolution_v: u16,
    #[doc = "Bit rate in bits per second"] bitrate: u32,
    #[doc = "Video image rotation clockwise"] rotation: u16,
    #[doc = "Video stream URI"] uri: [char; 230usize],
}
#[doc = "WIP: Message that sets video stream settings"]
pub struct SET_VIDEO_STREAM_SETTINGS {
    #[doc = "system ID of the target"] target_system: u8,
    #[doc = "component ID of the target"] target_component: u8,
    #[doc = "Camera ID (1 for first, 2 for second, etc.)"] camera_id: u8,
    #[doc = "Frames per second (set to -1 for highest framerate possible)"] framerate: f32,
    #[doc = "Resolution horizontal in pixels (set to -1 for highest resolution possible)"]
    resolution_h: u16,
    #[doc = "Resolution vertical in pixels (set to -1 for highest resolution possible)"]
    resolution_v: u16,
    #[doc = "Bit rate in bits per second (set to -1 for auto)"] bitrate: u32,
    #[doc = "Video image rotation clockwise (0-359 degrees)"] rotation: u16,
    #[doc = "Video stream URI"] uri: [char; 230usize],
}
#[doc = "Configure AP SSID and Password."]
pub struct WIFI_CONFIG_AP {
    #[doc = "Name of Wi-Fi network (SSID). Leave it blank to leave it unchanged."]
    ssid: [char; 32usize],
    #[doc = "Password. Leave it blank for an open AP."] password: [char; 64usize],
}
# [ doc = "WIP: Version and capability of protocol version. This message is the response to REQUEST_PROTOCOL_VERSION and is used as part of the handshaking to establish which MAVLink version should be used on the network. Every node should respond to REQUEST_PROTOCOL_VERSION to enable the handshaking. Library implementers should consider adding this into the default decoding state machine to allow the protocol core to respond directly." ]
pub struct PROTOCOL_VERSION {
    #[doc = "Currently active MAVLink version number * 100: v1.0 is 100, v2.0 is 200, etc."]
    version: u16,
    #[doc = "Minimum MAVLink version supported"] min_version: u16,
    #[doc = "Maximum MAVLink version supported (set to the same value as version by default)"]
    max_version: u16,
    #[doc = "The first 8 bytes (not characters printed in hex!) of the git hash."]
    spec_version_hash: [i8; 8usize],
    #[doc = "The first 8 bytes (not characters printed in hex!) of the git hash."]
    library_version_hash: [i8; 8usize],
}
# [ doc = "General status information of an UAVCAN node. Please refer to the definition of the UAVCAN message \"uavcan.protocol.NodeStatus\" for the background information. The UAVCAN specification is available at http://uavcan.org." ]
pub struct UAVCAN_NODE_STATUS {
    #[doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)"]
    time_usec: u64,
    #[doc = "The number of seconds since the start-up of the node."] uptime_sec: u32,
    #[doc = "Generalized node health status."] health: u8,
    #[doc = "Generalized operating mode."] mode: u8,
    #[doc = "Not used currently."] sub_mode: u8,
    #[doc = "Vendor-specific status information."] vendor_specific_status_code: u16,
}
# [ doc = "General information describing a particular UAVCAN node. Please refer to the definition of the UAVCAN service \"uavcan.protocol.GetNodeInfo\" for the background information. This message should be emitted by the system whenever a new node appears online, or an existing node reboots. Additionally, it can be emitted upon request from the other end of the MAVLink channel (see MAV_CMD_UAVCAN_GET_NODE_INFO). It is also not prohibited to emit this message unconditionally at a low frequency. The UAVCAN specification is available at http://uavcan.org." ]
pub struct UAVCAN_NODE_INFO { # [ doc = "Timestamp (microseconds since UNIX epoch or microseconds since system boot)" ] time_usec : u64 , # [ doc = "The number of seconds since the start-up of the node." ] uptime_sec : u32 , # [ doc = "Node name string. For example, \"sapog.px4.io\"." ] name : [ char ; 80usize ] , # [ doc = "Hardware major version number." ] hw_version_major : u8 , # [ doc = "Hardware minor version number." ] hw_version_minor : u8 , # [ doc = "Hardware unique 128-bit ID." ] hw_unique_id : [ i8 ; 16usize ] , # [ doc = "Software major version number." ] sw_version_major : u8 , # [ doc = "Software minor version number." ] sw_version_minor : u8 , # [ doc = "Version control system (VCS) revision identifier (e.g. git short commit hash). Zero if unknown." ] sw_vcs_commit : u32 , }
# [ doc = "Request to read the value of a parameter with the either the param_id string id or param_index." ]
pub struct PARAM_EXT_REQUEST_READ { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter index. Set to -1 to use the Parameter ID field as identifier (else param_id will be ignored)" ] param_index : i16 , }
#[doc = "Request all parameters of this component. After this request, all parameters are emitted."]
pub struct PARAM_EXT_REQUEST_LIST {
    #[doc = "System ID"] target_system: u8,
    #[doc = "Component ID"] target_component: u8,
}
# [ doc = "Emit the value of a parameter. The inclusion of param_count and param_index in the message allows the recipient to keep track of received parameters and allows them to re-request missing parameters after a loss or timeout." ]
pub struct PARAM_EXT_VALUE { # [ doc = "Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter value" ] param_value : [ char ; 128usize ] , # [ doc = "Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types." ] param_type : u8 , # [ doc = "Total number of parameters" ] param_count : u16 , # [ doc = "Index of this parameter" ] param_index : u16 , }
# [ doc = "Set a parameter value. In order to deal with message loss (and retransmission of PARAM_EXT_SET), when setting a parameter value and the new value is the same as the current value, you will immediately get a PARAM_ACK_ACCEPTED response. If the current state is PARAM_ACK_IN_PROGRESS, you will accordingly receive a PARAM_ACK_IN_PROGRESS in response." ]
pub struct PARAM_EXT_SET { # [ doc = "System ID" ] target_system : u8 , # [ doc = "Component ID" ] target_component : u8 , # [ doc = "Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter value" ] param_value : [ char ; 128usize ] , # [ doc = "Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types." ] param_type : u8 , }
#[doc = "Response from a PARAM_EXT_SET message."]
pub struct PARAM_EXT_ACK { # [ doc = "Parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string" ] param_id : [ char ; 16usize ] , # [ doc = "Parameter value (new value if PARAM_ACK_ACCEPTED, current value otherwise)" ] param_value : [ char ; 128usize ] , # [ doc = "Parameter type: see the MAV_PARAM_EXT_TYPE enum for supported data types." ] param_type : u8 , # [ doc = "Result code: see the PARAM_ACK enum for possible codes." ] param_result : u8 , }
# [ doc = "Obstacle distances in front of the sensor, starting from the left in increment degrees to the right" ]
pub struct OBSTACLE_DISTANCE { # [ doc = "Timestamp (microseconds since system boot or since UNIX epoch)." ] time_usec : u64 , # [ doc = "Class id of the distance sensor type." ] sensor_type : u8 , # [ doc = "Distance of obstacles around the UAV with index 0 corresponding to local North. A value of 0 means that the obstacle is right in front of the sensor. A value of max_distance +1 means no obstacle is present. A value of UINT16_MAX for unknown/not used. In a array element, one unit corresponds to 1cm." ] distances : [ i16 ; 72usize ] , # [ doc = "Angular width in degrees of each array element." ] increment : u8 , # [ doc = "Minimum distance the sensor can measure in centimeters." ] min_distance : u16 , # [ doc = "Maximum distance the sensor can measure in centimeters." ] max_distance : u16 , }
