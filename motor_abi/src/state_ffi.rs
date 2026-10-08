use super::*;
use motor_core::feedback::FeedbackSnapshot;

fn snapshot_metadata<T>(snapshot: &FeedbackSnapshot<T>) -> MotorStateMetadata {
    MotorStateMetadata {
        feedback_age_ns: snapshot.age().as_nanos().min(u128::from(u64::MAX)) as u64,
        feedback_sequence: snapshot.sequence,
    }
}

fn robstride_status_code(state: &motor_vendor_robstride::MotorFeedbackState) -> u8 {
    let mut status = 0u8;
    if state.uncalibrated {
        status |= 1 << 5;
    }
    if state.stall {
        status |= 1 << 4;
    }
    if state.magnetic_encoder_fault {
        status |= 1 << 3;
    }
    if state.overtemperature {
        status |= 1 << 2;
    }
    if state.overcurrent {
        status |= 1 << 1;
    }
    if state.undervoltage {
        status |= 1;
    }
    status
}

fn latest_snapshot(
    motor: &MotorHandleInner,
) -> Result<Option<(MotorState, MotorStateMetadata)>, String> {
    match motor {
        MotorHandleInner::Damiao(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Hexfellow(m) => {
            m.query_status(Duration::from_millis(20))
                .map_err(|e| e.to_string())?;
            let snapshot = m
                .latest_state_snapshot()
                .ok_or_else(|| "Hexfellow status query returned no state".to_string())?;
            let state = snapshot.state;
            Ok(Some((
                MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: 0,
                    status_code: state.heartbeat_state.unwrap_or(0),
                    pos: state.position_rev * (2.0 * PI),
                    vel: state.velocity_rev_s * (2.0 * PI),
                    torq: state.torque_permille as f32 / 1000.0,
                    t_mos: 0.0,
                    t_rotor: 0.0,
                },
                snapshot_metadata(&snapshot),
            )))
        }
        MotorHandleInner::MyActuator(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: m.motor_id as u8,
                    arbitration_id: state.arbitration_id,
                    status_code: state.command,
                    pos: state.shaft_angle_deg * (PI / 180.0),
                    vel: state.speed_dps * (PI / 180.0),
                    torq: state.current_a,
                    t_mos: f32::from(state.temperature_c),
                    t_rotor: 0.0,
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Robstride(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.device_id,
                    arbitration_id: state.arbitration_id,
                    status_code: robstride_status_code(&state),
                    pos: state.position,
                    vel: state.velocity,
                    torq: state.torque,
                    t_mos: state.temperature_c,
                    t_rotor: 0.0,
                },
                snapshot_metadata(&snapshot),
            )
        })),
        MotorHandleInner::Hightorque(m) => Ok(m.latest_state_snapshot().map(|snapshot| {
            let state = snapshot.state;
            (
                MotorState {
                    has_value: 1,
                    can_id: state.can_id,
                    arbitration_id: state.arbitration_id,
                    status_code: state.status_code,
                    pos: state.pos,
                    vel: state.vel,
                    torq: state.torq,
                    t_mos: state.t_mos,
                    t_rotor: state.t_rotor,
                },
                snapshot_metadata(&snapshot),
            )
        })),
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
) -> i32 {
    if motor.is_null() || out_state.is_null() {
        set_last_error("motor or out_state is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_state };
    match latest_snapshot(&motor) {
        Ok(Some((state, _))) => *out = state,
        Ok(None) => *out = MotorState::default(),
        Err(error) => {
            set_last_error(error);
            return -1;
        }
    }
    0
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_state_snapshot(
    motor: *mut MotorHandle,
    out_state: *mut MotorState,
    out_metadata: *mut MotorStateMetadata,
) -> i32 {
    if motor.is_null() || out_state.is_null() || out_metadata.is_null() {
        set_last_error("motor, out_state, or out_metadata is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out_state = unsafe { &mut *out_state };
    let out_metadata = unsafe { &mut *out_metadata };
    match latest_snapshot(&motor) {
        Ok(Some((state, metadata))) => {
            *out_state = state;
            *out_metadata = metadata;
        }
        Ok(None) => {
            *out_state = MotorState::default();
            *out_metadata = MotorStateMetadata::default();
        }
        Err(error) => {
            set_last_error(error);
            return -1;
        }
    }
    0
}
