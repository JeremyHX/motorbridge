//! 高擎 (Hightorque) 电机寄存器表。
//!
//! 数据来源:高擎电机寄存器功能说明表(寄存器功能、电机运行模式、报错代码、
//! 一托多模式说明.xlsx sheet1)。地址范围 0x00–0x45,含保留项。
//!
//! 2026-10-22 决策(用户):寄存器读**固定按 Int16** 读取,不再让调用方
//! 选数据类型;读返回值按本表 `quantity` 归属的物理量换算成物理值
//! (用户提供换算关系:位置/速度→rad、力矩→N·m 套型号 k 系数、电流/电压
//! →A/V、温度→°C、PWM/rKp/rKd→归一化、PID→0.1/LSB),**表内未分类的
//! 寄存器 1:1 返回原值**。名称按量归类的条目(位置指令/速度命令/上下限
//! 等)为推断,待真机校准。
//!
//! 同日追补决策:**缺号与表外地址拒绝访问**——0x00-0x45 内 xlsx 没有
//! 的洞(0x07-0x0C/0x13/0x17/0x1E/0x1F/0x2A/0x2E/0x2F/0x35-0x3F)与
//! 0x46-0xFF 均不在本表,读/写一律整段报错(这些地址能发帧能回值,但
//! 语义是猜的,不再放行)。

use crate::protocol::{
    decode_unit_raw, tqe_restore_raw, AngleUnit, RegisterType, RegisterValue, TorqueCoeff, ACC_SCALE,
    CUR_SCALE, PID_SCALE, POS_SCALE, PWM_SCALE, TEMP_SCALE, TIME_SCALE, TQE_SCALE, VEL_SCALE,
    VOL_SCALE,
};

/// 寄存器所属物理量(决定 raw→物理量换算)。`None`(表内未分类或表外
/// 地址)= 1:1 恒等,返回原值。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Quantity {
    /// 位置(rad)。
    Pos,
    /// 速度(rad/s)。
    Vel,
    /// 加速度(rad/s²;int16 scale 沿用固件 convert.c 1000,与旧文档 0.01
    /// 的争议见 protocol.rs ACC_SCALE 注,待真机校准)。
    Acc,
    /// 力矩(N·m;套型号 k/d 系数,与状态路径同式)。
    Torque,
    /// 电流(A)。
    Current,
    /// 电压(V)。
    Voltage,
    /// 温度(°C)。
    Temperature,
    /// 时间(s;当前表内无时间寄存器,量程备用)。
    Time,
    /// PWM / rKp / rKd(归一化,raw=32767 对应 1.0)。
    Pwm,
    /// PID 增益(0.1/LSB)。
    Pid,
}

impl Quantity {
    /// 输出单位标签(CLI/网关展示用;归一化与 PID 无量纲返回空)。
    pub fn unit_label(self) -> &'static str {
        match self {
            Self::Pos => "rad",
            Self::Vel => "rad/s",
            Self::Acc => "rad/s^2",
            Self::Torque => "N·m",
            Self::Current => "A",
            Self::Voltage => "V",
            Self::Temperature => "°C",
            Self::Time => "s",
            Self::Pwm => "",
            Self::Pid => "",
        }
    }
}

/// 寄存器元数据。
#[derive(Debug, Clone, Copy)]
pub struct RegisterInfo {
    /// 寄存器地址(0x00–0x45)。
    pub addr: u8,
    /// 寄存器名称。
    pub name: &'static str,
    /// 是否可写。
    pub writable: bool,
    /// 归属物理量(决定换算);`None` 表示无换算关系,读值 1:1。
    pub quantity: Option<Quantity>,
}

/// 完整寄存器表(按地址升序)。quantity 归类:0x01-0x03 对照状态帧
/// 换算;电流/电压/PID 对照固件 convert.c;温度/PWM/rKp/rKd 为用户
/// 2026-10-22 提供的换算关系;其余按名称推断(见各项行内注)。
pub const PARAMETER_TABLE: &[RegisterInfo] = &[
    RegisterInfo {
        addr: 0x00,
        name: "模式",
        writable: true,
        quantity: None,
    },
    // 位置/速度/力矩:参考固件 motor.c 用 0x17 0x01 一次读 3 个 int16,
    // 换算与状态帧 27 01 同式(POS/VEL/TQE_SCALE + k)。
    RegisterInfo {
        addr: 0x01,
        name: "位置",
        writable: false,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x02,
        name: "速度",
        writable: false,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x03,
        name: "转矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x04,
        name: "Q 相电流",
        writable: false,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x05,
        name: "D 相电流",
        writable: false,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x06,
        name: "保留",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x0D,
        name: "电压",
        writable: false,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x0E,
        name: "温度",
        writable: false,
        quantity: Some(Quantity::Temperature),
    },
    RegisterInfo {
        addr: 0x0F,
        name: "错误代码",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x10,
        name: "PWM 相位 A",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x11,
        name: "PWM 相位 B",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x12,
        name: "PWM 相位 C",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x14,
        name: "电压相位 A",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x15,
        name: "电压相位 B",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x16,
        name: "电压相位 C",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x18,
        name: "电压 FOC 角度",
        // 角度量,但用户未提供该寄存器的换算关系 → 1:1 原值。
        writable: true,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x19,
        name: "电压 FOC 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1A,
        name: "D 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1B,
        name: "Q 电压",
        writable: true,
        quantity: Some(Quantity::Voltage),
    },
    RegisterInfo {
        addr: 0x1C,
        name: "Q 电流",
        writable: true,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x1D,
        name: "D 电流",
        writable: true,
        quantity: Some(Quantity::Current),
    },
    RegisterInfo {
        addr: 0x20,
        name: "位置指令",
        // 名称归类为位置(推断,待真机校准)。
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x21,
        name: "速度命令",
        writable: true,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x22,
        name: "前馈扭矩",
        writable: true,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x23,
        name: "Kp 比例",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x24,
        name: "Kd 比例",
        writable: true,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x25,
        name: "最大扭矩",
        writable: true,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x26,
        name: "停止位置",
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x27,
        name: "保留",
        writable: false,
        quantity: None,
    },
    RegisterInfo {
        addr: 0x28,
        name: "速度限制",
        writable: true,
        quantity: Some(Quantity::Vel),
    },
    RegisterInfo {
        addr: 0x29,
        name: "加速度限制",
        writable: true,
        quantity: Some(Quantity::Acc),
    },
    RegisterInfo {
        addr: 0x2B,
        name: "Kp",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x2C,
        name: "Kd",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x2D,
        name: "Ki",
        writable: true,
        quantity: Some(Quantity::Pid),
    },
    RegisterInfo {
        addr: 0x30,
        name: "比例扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x31,
        name: "积分扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x32,
        name: "微分扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x33,
        name: "前馈扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x34,
        name: "总控制扭矩",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x40,
        name: "下限",
        // 名称归类为位置限位(推断,待真机校准)。
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x41,
        name: "上限",
        writable: true,
        quantity: Some(Quantity::Pos),
    },
    RegisterInfo {
        addr: 0x42,
        name: "前馈扭矩(映射 0x22)",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
    RegisterInfo {
        addr: 0x43,
        name: "Kp 比例(映射 0x23)",
        writable: false,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x44,
        name: "Kd 比例(映射 0x24)",
        writable: false,
        quantity: Some(Quantity::Pwm),
    },
    RegisterInfo {
        addr: 0x45,
        name: "最大扭矩(映射 0x25)",
        writable: false,
        quantity: Some(Quantity::Torque),
    },
];

/// 按地址查表。
pub fn parameter_info(addr: u8) -> Option<&'static RegisterInfo> {
    PARAMETER_TABLE.iter().find(|info| info.addr == addr)
}

/// 地址是否在寄存器表内(2026-10-22 追补决策:缺号与表外地址一律
/// `false`,读/写整段拒绝)。
pub fn is_known_register(addr: u8) -> bool {
    parameter_info(addr).is_some()
}

/// 校验一段连续读地址全部在表内(整段原子,任一越界即报错;0xB3/0xB4
/// 是命令地址不算寄存器)。报错文本带具体地址供上层透传。
pub fn ensure_known_span(addr: u8, count: u8) -> Result<(), String> {
    for i in 0..count {
        let a = addr.wrapping_add(i);
        if !is_known_register(a) {
            return Err(format!(
                "register 0x{a:02X} is not in the parameter table \
                 (缺号/表外地址拒绝访问, 2026-10-22)"
            ));
        }
    }
    Ok(())
}

/// 按地址取物理量归类;表内未分类或表外地址返回 `None`(换算 1:1)。
pub fn parameter_quantity(addr: u8) -> Option<Quantity> {
    parameter_info(addr).and_then(|info| info.quantity)
}

/// raw 寄存器值 → 物理量(2026-10-22 决策:读固定 Int16、返回换算后
/// 物理值;无换算关系 1:1)。
///
/// - `RegisterValue::F32` 恒等返回(浮点 raw 本身就是物理值);
/// - 位置/速度/加速度:raw/scale 换圈后 ×2π 得 rad(与状态路径
///   `decode_read_reply` 同式,数值逐位一致);
/// - 力矩:Int16 直接走 `tqe_restore_raw`(raw×0.01×k+d,与状态路径
///   同一函数);外来 Int8/I32 缓存条目按 raw/scale×k+d 推广;
/// - 其余量:raw/scale(经 `decode_unit_raw` 复用 QuantityScale 单源);
/// - `None`(表内未分类):1:1 原值。缺号/表外地址在访问入口已被
///   `ensure_known_span`/`is_known_register` 拒绝,不会到达本函数。
pub fn register_physical_value(addr: u8, v: RegisterValue, c: TorqueCoeff) -> f32 {
    let raw = v.as_f32();
    let rt = v.register_type();
    if rt == RegisterType::Float {
        return raw;
    }
    match parameter_quantity(addr) {
        None => raw,
        Some(Quantity::Torque) => {
            if let RegisterValue::I16(x) = v {
                tqe_restore_raw(x, c)
            } else {
                // 外来帧(Int8/I32)按 scale 推广的同一公式。
                raw / TQE_SCALE.scale(rt.data_type()) * c.k + c.d
            }
        }
        Some(q) => {
            let (scale, angle) = match q {
                Quantity::Pos => (POS_SCALE, true),
                Quantity::Vel => (VEL_SCALE, true),
                Quantity::Acc => (ACC_SCALE, true),
                Quantity::Current => (CUR_SCALE, false),
                Quantity::Voltage => (VOL_SCALE, false),
                Quantity::Temperature => (TEMP_SCALE, false),
                Quantity::Time => (TIME_SCALE, false),
                Quantity::Pwm => (PWM_SCALE, false),
                Quantity::Pid => (PID_SCALE, false),
                Quantity::Torque => unreachable!("handled above"),
            };
            // 角度量换 rad;非角量用 Turn 恒等过单位换算,退化为 raw/scale。
            let unit = if angle { AngleUnit::Rad } else { AngleUnit::Turn };
            decode_unit_raw(raw, unit, rt.data_type(), scale)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::protocol::RegisterValue;

    #[test]
    fn parameter_table_has_no_duplicate_addresses() {
        let mut addrs: Vec<u8> = PARAMETER_TABLE.iter().map(|i| i.addr).collect();
        addrs.sort_unstable();
        let before = addrs.len();
        addrs.dedup();
        assert_eq!(addrs.len(), before, "duplicate register addresses present");
    }

    #[test]
    fn known_registers_resolved() {
        assert_eq!(parameter_info(0x01).unwrap().name, "位置");
        assert_eq!(parameter_info(0x0F).unwrap().name, "错误代码");
        assert_eq!(parameter_info(0x45).unwrap().name, "最大扭矩(映射 0x25)");
    }

    #[test]
    fn unknown_address_returns_none() {
        assert!(parameter_info(0x07).is_none());
        assert!(parameter_info(0x50).is_none());
    }

    /// 2026-10-22 追补决策:缺号与表外地址一律拒绝访问。
    #[test]
    fn known_register_membership() {
        // 表内
        assert!(is_known_register(0x00));
        assert!(is_known_register(0x01));
        assert!(is_known_register(0x45));
        // 缺号(0x00-0x45 内的洞)
        assert!(!is_known_register(0x07));
        assert!(!is_known_register(0x0A));
        assert!(!is_known_register(0x13));
        assert!(!is_known_register(0x2E));
        assert!(!is_known_register(0x35));
        // 表外(0x46-0xFF)
        assert!(!is_known_register(0x46));
        assert!(!is_known_register(0x50));
        assert!(!is_known_register(0xB3));
        assert!(!is_known_register(0xFF));
    }

    /// 整段校验:任一地址越界即报错,报错文本带具体地址。
    #[test]
    fn ensure_known_span_atomic() {
        assert!(ensure_known_span(0x01, 3).is_ok());
        assert!(ensure_known_span(0x45, 1).is_ok());
        // 单点缺号/表外
        let e = ensure_known_span(0x0A, 1).unwrap_err();
        assert!(e.contains("0x0A"), "err: {e}");
        let e = ensure_known_span(0x50, 1).unwrap_err();
        assert!(e.contains("0x50"), "err: {e}");
        // 连续读跨入缺号:0x2D 表内,0x2E 缺号
        let e = ensure_known_span(0x2D, 2).unwrap_err();
        assert!(e.contains("0x2E"), "err: {e}");
        // 连续读跨出表尾:0x45 表内,0x46 表外
        let e = ensure_known_span(0x45, 2).unwrap_err();
        assert!(e.contains("0x46"), "err: {e}");
        // 0xB3/0xB4 是命令地址,不算寄存器
        assert!(!is_known_register(0xB3));
        assert!(!is_known_register(0xB4));
    }

    /// 2026-10-22 决策:各地址按物理量归类;无换算关系的(0x00/0x06/
    /// 0x0F/0x18/0x27/表外)quantity 为 None → 读值 1:1。
    #[test]
    fn quantity_assignment_by_class() {
        assert_eq!(parameter_quantity(0x01), Some(Quantity::Pos));
        assert_eq!(parameter_quantity(0x02), Some(Quantity::Vel));
        assert_eq!(parameter_quantity(0x03), Some(Quantity::Torque));
        assert_eq!(parameter_quantity(0x04), Some(Quantity::Current));
        assert_eq!(parameter_quantity(0x0D), Some(Quantity::Voltage));
        assert_eq!(parameter_quantity(0x0E), Some(Quantity::Temperature));
        assert_eq!(parameter_quantity(0x10), Some(Quantity::Pwm));
        assert_eq!(parameter_quantity(0x2B), Some(Quantity::Pid));
        assert_eq!(parameter_quantity(0x29), Some(Quantity::Acc));
        // None(1:1):模式/保留/错误代码/FOC 角度(无换算关系)。
        assert_eq!(parameter_quantity(0x00), None);
        assert_eq!(parameter_quantity(0x06), None);
        assert_eq!(parameter_quantity(0x0F), None);
        assert_eq!(parameter_quantity(0x18), None);
        assert_eq!(parameter_quantity(0x27), None);
        assert_eq!(parameter_quantity(0x50), None);
    }

    /// 单位标签(CLI/网关展示)。
    #[test]
    fn unit_labels() {
        assert_eq!(Quantity::Pos.unit_label(), "rad");
        assert_eq!(Quantity::Vel.unit_label(), "rad/s");
        assert_eq!(Quantity::Torque.unit_label(), "N·m");
        assert_eq!(Quantity::Temperature.unit_label(), "°C");
        assert_eq!(Quantity::Pwm.unit_label(), "");
    }

    /// 换算抽查:与状态路径同式。0x01 raw 0x1234 → 0.1234 圈 × 2π。
    #[test]
    fn converts_pos_like_state_path() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x01, RegisterValue::I16(0x1234), c);
        assert!((v - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU).abs() < 1e-6);
    }

    /// 速度:raw/4000 × 2π(状态路径同式;注意用 /4000.0 而非 ×0.00025,
    /// 两者在 f32 下有 ~2e-6 的表示差,值域 ~35 时会越过 1e-6 容差)。
    #[test]
    fn converts_vel_like_state_path() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x02, RegisterValue::I16(0x5678), c);
        assert!((v - 0x5678 as f32 / 4000.0 * std::f32::consts::TAU).abs() < 1e-6);
    }

    /// 力矩:Int16 走 tqe_restore_raw(raw×0.01×k+d),与状态路径同一函数。
    #[test]
    fn converts_torque_with_model_coeff() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x03, RegisterValue::I16(200), c);
        assert!((v - 200.0 * 0.01 * 0.5).abs() < 1e-6);
    }

    /// 温度 0.1 °C/LSB、PWM 1/32767、PID 0.1/LSB。
    #[test]
    fn converts_temperature_pwm_pid() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        assert!((register_physical_value(0x0E, RegisterValue::I16(100), c) - 10.0).abs() < 1e-6);
        assert!(
            (register_physical_value(0x10, RegisterValue::I16(32767), c) - 1.0).abs() < 1e-6
        );
        assert!((register_physical_value(0x2B, RegisterValue::I16(25), c) - 2.5).abs() < 1e-6);
    }

    /// 无换算关系 1:1;F32 恒等。
    #[test]
    fn identity_for_unclassified_and_float() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        assert!((register_physical_value(0x0F, RegisterValue::I16(5), c) - 5.0).abs() < 1e-6);
        assert_eq!(register_physical_value(0x50, RegisterValue::I16(-3), c), -3.0);
        assert_eq!(register_physical_value(0x03, RegisterValue::F32(1.5), c), 1.5);
    }

    /// 外来 Int8 缓存条目(如别的主机发的 int8 力矩帧)按同式推广换算:
    /// Int8 力矩 scale=2 → raw/2 × k。
    #[test]
    fn converts_foreign_int8_torque_entry() {
        let c = TorqueCoeff { k: 0.5, d: 0.0 };
        let v = register_physical_value(0x22, RegisterValue::I8(17), c);
        assert!((v - 17.0 / 2.0 * 0.5).abs() < 1e-6);
    }
}
