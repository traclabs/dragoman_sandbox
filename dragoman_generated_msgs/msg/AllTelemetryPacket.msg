# ROS 2 Message: AllTelemetryPacket.msg
# Generated automatically from XTCE file: /dragoman/install/dragoman_sample_xtce/share/dragoman_sample_xtce/xtce/AllTypes.xtce
# Generation Date: 2025-12-08 14:03:54

int32 t_integer_signed # Signed Integer, 32-bit, Two's Complement
float64 t_float_raw64 # Raw 64-bit Float, no calibration
uint8 t_enumerated_alarm # 8-bit Enumerated Type with states
string t_string_utf8 # String up to 64 bytes, UTF-8 encoded
bool t_boolean_flag # Boolean flag, 1 byte (0/1)
builtin_interfaces/Time t_absolute_time # Absolute Time, 64-bit integer (epoch ms)
uint32 t_relative_time_raw # Relative time, 32-bit integer (raw counts)
uint8[16] t_binary_blob # Raw Binary Data/Block, 128 bits
uint16[10] t_integer_array # Array of 10 unsigned 16-bit integers
float32 t_status_aggregate_current_draw # Member of T_StatusAggregate: Aggregate/Struct: CurrentDraw (Float), HeaterEnabled (Boolean), and RawStatusFlags (Binary)
bool t_status_aggregate_heater_enabled # Member of T_StatusAggregate: Aggregate/Struct: CurrentDraw (Float), HeaterEnabled (Boolean), and RawStatusFlags (Binary)
uint8[4] t_status_aggregate_raw_status_flags # Member of T_StatusAggregate: Aggregate/Struct: CurrentDraw (Float), HeaterEnabled (Boolean), and RawStatusFlags (Binary)
