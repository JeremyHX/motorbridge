#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParameterDataType {
    Int8,
    UInt8,
    UInt16,
    UInt32,
    Float32,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u16)]
pub enum ParameterId {
    MechanicalOffset = 0x2005,
    // 0x30xx host-tool observation addresses (e.g. MeasuredTorque = 0x302C)
    // are intentionally NOT modeled here: they belong to the vendor's serial
    // host protocol and are rejected by CAN type-17 reads/writes on real
    // hardware. Feedback torque comes from status frames instead.
    Mode = 0x7005,
    IqTarget = 0x7006,
    VelocityTarget = 0x700A,
    TorqueLimit = 0x700B,
    CurrentKp = 0x7010,
    CurrentKi = 0x7011,
    CurrentFilterGain = 0x7014,
    PositionTarget = 0x7016,
    VelocityLimit = 0x7017,
    CurrentLimit = 0x7018,
    MechanicalPosition = 0x7019,
    IqFiltered = 0x701A,
    MechanicalVelocity = 0x701B,
    Vbus = 0x701C,
    PositionKp = 0x701E,
    VelocityKp = 0x701F,
    VelocityKi = 0x7020,
    VelocityFilterGain = 0x7021,
    VelocityAccelerationTarget = 0x7022,
    PpVelocityMax = 0x7024,
    PpAccelerationTarget = 0x7025,
    EpscanTime = 0x7026,
    CanTimeout = 0x7028,
    ZeroState = 0x7029,
    Damper = 0x702A,
    AddOffset = 0x702B,
    AlveolousOpen = 0x702C,
    IqTest = 0x702D,
    DccSet = 0x702E,
    ProtocolFlag = 0x2022,
}

#[derive(Debug, Clone, Copy)]
pub struct ParameterInfo {
    pub id: u16,
    pub name: &'static str,
    pub data_type: ParameterDataType,
}

macro_rules! param {
    ($id:expr, $name:expr, $ty:ident) => {
        ParameterInfo {
            id: $id,
            name: $name,
            data_type: ParameterDataType::$ty,
        }
    };
}

pub static PARAMETER_TABLE: &[ParameterInfo] = &[
    param!(0x2000, "echoPara1", UInt16),
    param!(0x2001, "echoPara2", UInt16),
    param!(0x2002, "echoPara3", UInt16),
    param!(0x2003, "echoPara4", UInt16),
    param!(0x2004, "echoFreHz", UInt32),
    param!(0x2005, "mechOffset", Float32),
    param!(0x2006, "MechPos_init", Float32),
    param!(0x2007, "limit_torque", Float32),
    param!(0x2008, "I_FW_MAX", Float32),
    param!(0x2009, "motor_baud", UInt8),
    param!(0x200A, "CAN_ID", UInt8),
    param!(0x200B, "CAN_MASTER", UInt8),
    param!(0x200C, "CAN_TIMEOUT", UInt32),
    param!(0x200E, "status3", UInt32),
    param!(0x200F, "status1", Float32),
    param!(0x2010, "status6", UInt8),
    param!(0x2011, "cur_filt_gain", Float32),
    param!(0x2012, "cur_kp", Float32),
    param!(0x2013, "cur_ki", Float32),
    param!(0x2014, "spd_kp", Float32),
    param!(0x2015, "spd_ki", Float32),
    param!(0x2016, "loc_kp", Float32),
    param!(0x2017, "spd_filt_gain", Float32),
    param!(0x2018, "limit_spd", Float32),
    param!(0x2019, "limit_cur", Float32),
    param!(0x201A, "loc_ref_filt_gai", Float32),
    param!(0x201B, "limit_loc", Float32),
    param!(0x201C, "position_offset", Float32),
    param!(0x201D, "chasu_angle_offs", Float32),
    param!(0x201E, "spd_step_value", Float32),
    param!(0x201F, "vol_max", Float32),
    param!(0x2020, "acc_set", Float32),
    param!(0x2022, "protocol_1", UInt8),
    // The 0x30xx observation block from the vendor manual is deliberately
    // omitted: those addresses are host-tool serial protocol only; type-17
    // (read single parameter) and type-18 (write) frames to them are
    // rejected by real firmware with a non-zero read status. Position,
    // velocity, torque, and temperatures are available through the
    // operation-status / active-report feedback frames instead.
    // RobStride protocol section 4 "read/write single parameter list".
    // Wire format: index is little-endian in byte0..1, value is little-endian
    // in byte4..7. Writes use communication type 18; parameters that must
    // persist after power-cycle require communication type 22 save afterwards.
    param!(0x7005, "run_mode", Int8), // W/R, enum: 0 MIT, 1 PP, 2 velocity, 3 current, 5 CSP.
    param!(0x7006, "iq_ref", Float32), // W/R, A, current-mode Iq target, -43..43.
    param!(0x700A, "spd_ref", Float32), // W/R, rad/s, velocity-mode target, -20..20.
    param!(0x700B, "limit_torque", Float32), // W/R, Nm, torque limit, 0..60.
    param!(0x7010, "cur_kp", Float32), // W/R, current-loop Kp, default 0.17.
    param!(0x7011, "cur_ki", Float32), // W/R, current-loop Ki, default 0.012.
    param!(0x7014, "cur_filter_gain", Float32), // W/R, ratio, current filter gain, 0..1, default 0.1.
    param!(0x7016, "loc_ref", Float32),         // W/R, rad, position target.
    param!(0x7017, "limit_spd", Float32),       // W/R, rad/s, CSP position speed limit, 0..20.
    param!(0x7018, "limit_cur", Float32),       // W/R, A, velocity/position current limit, 0..43.
    param!(0x7019, "mechPos", Float32),         // R, rad, load-side counted mechanical angle.
    param!(0x701A, "iqf", Float32),             // R, A, filtered iq.
    param!(0x701B, "mechVel", Float32),         // R, rad/s, load-side velocity.
    param!(0x701C, "VBUS", Float32),            // R, V, bus voltage.
    param!(0x701E, "loc_kp", Float32),          // W/R, position-loop Kp, default 60.
    param!(0x701F, "spd_kp", Float32),          // W/R, speed-loop Kp, default 6.
    param!(0x7020, "spd_ki", Float32),          // W/R, speed-loop Ki, default 0.02.
    param!(0x7021, "spd_filter_gain", Float32), // W/R, speed filter gain, default 0.1.
    param!(0x7022, "acc_rad", Float32), // W/R, rad/s^2, velocity-mode acceleration, default 20.
    param!(0x7024, "vel_max", Float32), // W/R, rad/s, PP mode velocity, default 10.
    param!(0x7025, "acc_set", Float32), // W/R, rad/s^2, PP mode acceleration, default 10.
    param!(0x7026, "EPScan_time", UInt16), // W, report period, default 1: 1=10ms, +1 adds 5ms.
    param!(0x7028, "canTimeout", UInt32), // W, CAN timeout, default 0; 20000 means 1s.
    param!(0x7029, "zero_sta", UInt8),  // W, default 0; 0: 0..2pi, 1: -pi..pi; save with type 22.
    param!(0x702A, "damper", UInt8),    // W/R, switch; 1 disables power-off back-drive damping.
    param!(0x702B, "add_offset", Float32), // W/R, rad, zero offset, default 0.
    param!(0x702C, "alveolous_open", UInt8), // W/R, switch; 1 enables cogging compensation.
    param!(0x702D, "iq_test", UInt8), // W/R, switch; 1 enables more precise initialization calibration.
    param!(0x702E, "dcc_set", Float32), // W/R, rad/s^2, PP-mode deceleration, default 10.
];

pub fn parameter_info(id: u16) -> Option<&'static ParameterInfo> {
    PARAMETER_TABLE.iter().find(|info| info.id == id)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn section4_runtime_parameter_list_is_complete() {
        let expected = [
            0x7005, 0x7006, 0x700A, 0x700B, 0x7010, 0x7011, 0x7014, 0x7016, 0x7017, 0x7018, 0x7019,
            0x701A, 0x701B, 0x701C, 0x701E, 0x701F, 0x7020, 0x7021, 0x7022, 0x7024, 0x7025, 0x7026,
            0x7028, 0x7029, 0x702A, 0x702B, 0x702C, 0x702D, 0x702E,
        ];
        for id in expected {
            assert!(parameter_info(id).is_some(), "missing 0x{id:04X}");
        }
    }

    #[test]
    fn newly_added_section4_parameters_have_expected_types() {
        assert_eq!(
            parameter_info(0x702A).unwrap().data_type,
            ParameterDataType::UInt8
        );
        assert_eq!(
            parameter_info(0x702B).unwrap().data_type,
            ParameterDataType::Float32
        );
        assert_eq!(
            parameter_info(0x702C).unwrap().data_type,
            ParameterDataType::UInt8
        );
        assert_eq!(
            parameter_info(0x702D).unwrap().data_type,
            ParameterDataType::UInt8
        );
        assert_eq!(
            parameter_info(0x702E).unwrap().data_type,
            ParameterDataType::Float32
        );
    }
}
