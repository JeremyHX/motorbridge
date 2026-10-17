#ifndef MOTOR_ABI_H
#define MOTOR_ABI_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct MotorController MotorController;
typedef struct MotorHandle MotorHandle;

typedef struct MotorState {
  int32_t has_value;
  uint8_t can_id;
  uint32_t arbitration_id;
  uint8_t status_code;
  float pos;
  float vel;
  float torq;
  float t_mos;
  float t_rotor;
  // HighTorque: table-3 fault code (0 = no fault). Other vendors: always 0.
  uint8_t fault_code;
} MotorState;

typedef struct MotorStateMetadata {
  uint64_t feedback_age_ns;
  uint64_t feedback_sequence;
} MotorStateMetadata;

// Unified units across all vendors:
// - position: rad
// - velocity: rad/s
// - torque: Nm
//
// Unified mode IDs for motor_handle_ensure_mode:
// - 1: MIT
// - 2: POS_VEL
// - 3: VEL
// - 4: FORCE_POS
//
// Vendor support in current ABI:
// - Damiao
// - Hexfellow (CAN-FD transport via motor_controller_new_socketcanfd)
// - RobStride
// - MyActuator
// - HighTorque (native ht_can v1.5.5)

// Returns the last error message for the calling thread.
//
// Lifetime: the returned pointer is owned by the ABI and remains valid only
// until the next ABI call on the same thread changes the thread-local error
// slot. Copy the string immediately if it must be kept.
const char* motor_last_error_message(void);
const char* motor_abi_version(void);
const char* motor_abi_capabilities_json(void);

// Thread-safety:
// - Calls using the same MotorController or MotorHandle are serialized inside
//   the ABI.
// - Free functions are exclusive ownership operations: do not call
//   motor_controller_free or motor_handle_free concurrently with any other
//   operation using the same pointer, and do not use a pointer after free.
// - Different handles may be used from different threads.

MotorController* motor_controller_new_socketcan(const char* channel);
MotorController* motor_controller_new_socketcanfd(const char* channel);
MotorController* motor_controller_new_dm_serial(const char* serial_port, uint32_t baud);
MotorController* motor_controller_new_dm_device(const char* dm_device_type, const char* dm_channel);
MotorController* motor_controller_new_mcu_serial(const char* serial_port, uint32_t baud);
void motor_controller_free(MotorController* controller);
int32_t motor_controller_poll_feedback_once(MotorController* controller);
int32_t motor_controller_enable_all(MotorController* controller);
int32_t motor_controller_disable_all(MotorController* controller);
int32_t motor_controller_shutdown(MotorController* controller);
int32_t motor_controller_close_bus(MotorController* controller);

MotorHandle* motor_controller_add_damiao_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_hexfellow_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_myactuator_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_robstride_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
MotorHandle* motor_controller_add_hightorque_motor(MotorController* controller, uint16_t motor_id, uint16_t feedback_id, const char* model);
void motor_handle_free(MotorHandle* motor);

int32_t motor_handle_enable(MotorHandle* motor);
int32_t motor_handle_disable(MotorHandle* motor);
int32_t motor_handle_clear_error(MotorHandle* motor);
int32_t motor_handle_set_zero_position(MotorHandle* motor);
int32_t motor_handle_ensure_mode(MotorHandle* motor, uint32_t mode, uint32_t timeout_ms);
int32_t motor_handle_stop(MotorHandle* motor);

int32_t motor_handle_send_mit(MotorHandle* motor, float target_position, float target_velocity, float stiffness, float damping, float feedforward_torque);
int32_t motor_handle_send_pos_vel(MotorHandle* motor, float target_position, float velocity_limit);
int32_t motor_handle_robstride_send_pos_vel_pp(MotorHandle* motor, float target_position, float velocity_max, float acceleration);
int32_t motor_handle_robstride_send_pos_vel_csp(MotorHandle* motor, float target_position, float velocity_limit);
int32_t motor_handle_send_vel(MotorHandle* motor, float target_velocity);
int32_t motor_handle_send_force_pos(MotorHandle* motor, float target_position, float velocity_limit, float torque_limit_ratio);

int32_t motor_handle_store_parameters(MotorHandle* motor);
int32_t motor_handle_request_feedback(MotorHandle* motor);
int32_t motor_handle_set_can_timeout_ms(MotorHandle* motor, uint32_t timeout_ms);

int32_t motor_handle_write_register_f32(MotorHandle* motor, uint8_t rid, float value);
int32_t motor_handle_write_register_u32(MotorHandle* motor, uint8_t rid, uint32_t value);
int32_t motor_handle_get_register_f32(MotorHandle* motor, uint8_t rid, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_get_register_u32(MotorHandle* motor, uint8_t rid, uint32_t timeout_ms, uint32_t* out_value);

int32_t motor_handle_robstride_ping(MotorHandle* motor, uint8_t* out_device_id, uint8_t* out_responder_id);
int32_t motor_handle_robstride_ping_host_id(MotorHandle* motor, uint16_t host_id, uint32_t timeout_ms, uint8_t* out_device_id, uint8_t* out_responder_id);
int32_t motor_handle_robstride_get_param_f32_host_id(MotorHandle* motor, uint16_t param_id, uint16_t host_id, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_robstride_get_fault_report(MotorHandle* motor, uint32_t* out_fault_raw, uint32_t* out_warning_raw);
int32_t motor_handle_robstride_set_device_id(MotorHandle* motor, uint8_t new_device_id);
int32_t motor_handle_robstride_set_active_report(MotorHandle* motor, uint8_t enabled);
int32_t motor_handle_robstride_write_param_i8(MotorHandle* motor, uint16_t param_id, int8_t value);
int32_t motor_handle_robstride_write_param_u8(MotorHandle* motor, uint16_t param_id, uint8_t value);
int32_t motor_handle_robstride_write_param_u16(MotorHandle* motor, uint16_t param_id, uint16_t value);
int32_t motor_handle_robstride_write_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t value);
int32_t motor_handle_robstride_write_param_f32(MotorHandle* motor, uint16_t param_id, float value);
int32_t motor_handle_robstride_get_param_i8(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, int8_t* out_value);
int32_t motor_handle_robstride_get_param_u8(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint8_t* out_value);
int32_t motor_handle_robstride_get_param_u16(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint16_t* out_value);
int32_t motor_handle_robstride_get_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint32_t* out_value);
int32_t motor_handle_robstride_get_param_f32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, float* out_value);

int32_t motor_handle_get_state(MotorHandle* motor, MotorState* out_state);
int32_t motor_handle_get_state_snapshot(MotorHandle* motor, MotorState* out_state, MotorStateMetadata* out_metadata);

int32_t motor_handle_damiao_get_param_f32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, float* out_value);
int32_t motor_handle_damiao_get_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t timeout_ms, uint32_t* out_value);
int32_t motor_handle_damiao_write_param_f32(MotorHandle* motor, uint16_t param_id, float value);
int32_t motor_handle_damiao_write_param_u32(MotorHandle* motor, uint16_t param_id, uint32_t value);

// HighTorque-only extensions (reject non-HighTorque motors):
// Trapezoid control (07 2d, firmware v4.6.0+).
int32_t motor_handle_hightorque_send_pos_vel_acc(MotorHandle* motor, float target_position, float velocity_limit, float acceleration_limit);
// Classic position mode (07 07): torque limit in Nm; pass NaN for unlimited.
int32_t motor_handle_hightorque_send_pos_classic(MotorHandle* motor, float target_position, float torque_limit_nm);
// Firmware version query (15 B5 02), blocking with timeout.
int32_t motor_handle_hightorque_request_firmware_version(MotorHandle* motor, uint32_t timeout_ms, uint8_t* out_major, uint8_t* out_minor, uint8_t* out_patch);
// Generic register read (17 family): ty 0=Int8/1=Int16/2=Int32/3=Float, count 1-3
// consecutive registers; writes `count` f32 values (raw integer values cast to f32).
int32_t motor_handle_hightorque_get_register(MotorHandle* motor, uint8_t addr, uint8_t ty, uint8_t count, uint32_t timeout_ms, float* out_values);
// Torque mode (05 13): direct torque target in Nm (model coeff compensated).
int32_t motor_handle_hightorque_send_torque(MotorHandle* motor, float torque_nm);
// DQ voltage mode (01 00 08 05 1B): Q-axis voltage in V (0.1 V/LSB).
int32_t motor_handle_hightorque_send_dq_voltage(MotorHandle* motor, float voltage_v);
// DQ current mode (01 00 09 05 1C): Q-axis current in A (0.1 A/LSB).
int32_t motor_handle_hightorque_send_dq_current(MotorHandle* motor, float current_a);
// Brake (run mode 15, 01 00 0F): all phases shorted damping brake.
int32_t motor_handle_hightorque_brake(MotorHandle* motor);
// Run mode switch (01 00 <mode>, register 0x000): mode 0-17 per protocol table 2
// (0 = stop & clear error). Sends a real frame, unlike motor_handle_ensure_mode.
int32_t motor_handle_hightorque_set_run_mode(MotorHandle* motor, uint8_t mode);
// Generic register write (05 <addr> <val16>): config_class != 0 sends via extended
// 0x8000|id frame (config registers), 0 via the control channel (control targets).
int32_t motor_handle_hightorque_write_register(MotorHandle* motor, uint8_t addr, int16_t value, uint8_t config_class);
// Periodic status report (05 B4 02 <t_ms16>, extended frame): period_ms >= 1 ms;
// pass 0 to stop the motor-initiated 27 01 status stream.
int32_t motor_handle_hightorque_set_periodic_report(MotorHandle* motor, uint16_t period_ms);

#ifdef __cplusplus
}
#endif

#endif
