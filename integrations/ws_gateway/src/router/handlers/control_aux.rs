use crate::commands::{as_f32, as_u64};
use crate::model::{ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use motor_vendor_hightorque::{parameter_info, ReadCmd, RegisterType, RegisterValue};
use serde_json::{json, Value};
use std::time::Duration;

pub(crate) fn handle(op: &str, v: &Value, ctx: &mut SessionCtx) -> Option<Result<Value, String>> {
    match op {
        "current" => Some(handle_current(v, ctx)),
        "pos" => Some(handle_pos(v, ctx)),
        "pos_classic" | "pos-classic" => Some(handle_pos_classic(v, ctx)),
        "version" => Some(handle_version(v, ctx)),
        "mode_query" | "mode-query" => Some(handle_mode_query(ctx)),
        "read" => Some(handle_read(v, ctx)),
        "read_register" | "read-register" => Some(handle_read_register(v, ctx)),
        "dq_voltage" | "dq-voltage" => Some(handle_dq_voltage(v, ctx)),
        "dq_current" | "dq-current" => Some(handle_dq_current(v, ctx)),
        "brake" => Some(handle_brake(ctx)),
        "run_mode" | "run-mode" => Some(handle_run_mode(v, ctx)),
        "write_register" | "write-register" => Some(handle_write_register(v, ctx)),
        "periodic_report" | "periodic-report" => Some(handle_periodic_report(v, ctx)),
        _ => None,
    }
}

fn handle_current(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let current = as_f32(v, "current", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.send_current_setpoint(current)
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"current", "current": current}))
        }
        Some(_) => Err("current is supported for myactuator only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_pos(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    let max_speed = as_f32(v, "max_speed", 8.726646);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.send_position_absolute_setpoint(pos.to_degrees(), max_speed.to_degrees())
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"pos", "pos": pos, "max_speed": max_speed}))
        }
        Some(_) => Err("pos is supported for myactuator only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque 普通位置模式(`07 07`):目标位置 + 可选力矩上限。
/// `tqe` 缺省表示力矩无限制(0x8000 哨兵);力矩按型号自适应补偿。
/// 一次性命令,不进入 continuous 活动指令(与 myactuator "pos" 同型)。
fn handle_pos_classic(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    let tqe = v.get("tqe").and_then(Value::as_f64).map(|x| x as f32);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_pos_classic(pos, tqe)
                .map_err(|e| e.to_string())?;
            Ok(json!({"op":"pos_classic","pos": pos,"tqe": tqe}))
        }
        Some(_) => Err("pos_classic is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_version(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let timeout_ms = as_u64(v, "timeout_ms", 500);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Myactuator(m)) => {
            m.request_version_date().map_err(|e| e.to_string())?;
            let version = m
                .await_version_date(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            Ok(json!({"version": version}))
        }
        // 高擎固件版本(15 B5 02):vendor 原语收敛后接入,解除
        // "version 为 myactuator-only" 的限制。
        Some(MotorHandle::Hightorque(m)) => {
            let fw = m
                .request_firmware_version(Duration::from_millis(timeout_ms))
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "vendor": "hightorque",
                "major": fw.major,
                "minor": fw.minor,
                "patch": fw.patch,
            }))
        }
        Some(_) => Err("version is supported for myactuator and hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn handle_mode_query(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Myactuator(c)), Some(MotorHandle::Myactuator(m))) => {
            m.request_control_mode().map_err(|e| e.to_string())?;
            c.poll_feedback_once().map_err(|e| e.to_string())?;
            Ok(json!({"mode": m.latest_control_mode()}))
        }
        (Some(_), Some(_)) => Err("mode_query is supported for myactuator only".to_string()),
        _ => Err("motor not connected".to_string()),
    }
}

fn handle_read(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match (&ctx.controller, &ctx.motor) {
        (Some(ControllerHandle::Hightorque(_)), Some(MotorHandle::Hightorque(m))) => {
            // 收敛(2026-09-29):17 01 查询 + 等待 + 解码全部走 vendor,
            // 物理量带力矩系数补偿(旧手写路径力矩平移 ÷100)。
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            m.request_motor_feedback(timeout).map_err(|e| e.to_string())?;
            match m.latest_state() {
                Some(s) => Ok(json!({
                    "motor_id": s.can_id,
                    "arbitration_id": s.arbitration_id,
                    "pos": s.pos,
                    "vel": s.vel,
                    "torq": s.torq,
                    "status_code": s.status_code,
                    "fault_code": s.fault_code,
                })),
                None => Err("hightorque read timeout".to_string()),
            }
        }
        (Some(_), Some(_)) => Err("read op is reserved for hightorque".to_string()),
        _ => Err("motor not connected".to_string()),
    }
}

/// HighTorque 通用寄存器读取(17 族):从 `addr` 连续读 `count`(1-3) 个寄存器。
/// 数据类型 `ty`(0=Int8/1=Int16/2=Int32/3=Float)优先取报文显式指定,
/// 否则回落 `parameter_info` 已核实类型;两者皆无则报错要求显式指定
/// (规范表未给出所有寄存器类型,盲读会按错误宽度解码)。
/// 返回原始值(物理量标度换算按寄存器功能表由调用方完成)。
fn handle_read_register(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            let addr = v
                .get("addr")
                .and_then(Value::as_u64)
                .ok_or_else(|| "read_register requires integer 'addr' (0x00-0x45)".to_string())?
                as u8;
            let count = as_u64(v, "count", 1).clamp(1, 3) as u8;
            let info = parameter_info(addr);
            let ty = match v.get("ty").and_then(Value::as_u64) {
                Some(0) => RegisterType::Int8,
                Some(1) => RegisterType::Int16,
                Some(2) => RegisterType::Int32,
                Some(3) => RegisterType::Float,
                Some(_) => {
                    return Err(
                        "read_register 'ty' must be 0=Int8/1=Int16/2=Int32/3=Float".to_string(),
                    )
                }
                None => info
                    .and_then(|i| i.ty)
                    .ok_or_else(|| format!(
                        "read_register: register 0x{addr:02X} data type unknown; pass 'ty' explicitly (0=Int8/1=Int16/2=Int32/3=Float)"
                    ))?,
            };
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            let vals = m
                .read_registers(&[ReadCmd { addr, ty, count }], timeout)
                .map_err(|e| e.to_string())?;
            let registers: Vec<Value> = vals
                .iter()
                .map(|(a, rv)| {
                    json!({
                        "addr": a,
                        "value": match rv {
                            RegisterValue::I8(x) => json!(x),
                            RegisterValue::I16(x) => json!(x),
                            RegisterValue::I32(x) => json!(x),
                            RegisterValue::F32(x) => json!(x),
                        },
                    })
                })
                .collect();
            Ok(json!({
                "op": "read_register",
                "vendor": "hightorque",
                "addr": addr,
                "count": count,
                "name": info.map(|i| i.name),
                "type": hightorque_register_type_name(ty),
                "registers": registers,
            }))
        }
        Some(_) => Err("read_register is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

fn hightorque_register_type_name(ty: RegisterType) -> &'static str {
    match ty {
        RegisterType::Int8 => "i8",
        RegisterType::Int16 => "i16",
        RegisterType::Int32 => "i32",
        RegisterType::Float => "f32",
    }
}

/// HighTorque DQ 电压模式(`01 00 08 05 1B`,模式写+寄存器写拼一帧):
/// Q 轴电压(V,0.1 V/LSB)。协议只定义 Q 分量。一次性命令,不进入 continuous
/// (持续 DQ 电压控制请循环调用)。
fn handle_dq_voltage(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let volt = as_f32(v, "volt", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_dq_voltage(volt).map_err(|e| e.to_string())?;
            Ok(json!({"op":"dq_voltage","volt": volt}))
        }
        Some(_) => Err("dq_voltage is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque DQ 电流模式(`01 00 09 05 1C`,拼一帧):Q 轴电流(A,0.1 A/LSB)。
fn handle_dq_current(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let cur = as_f32(v, "cur", 0.0);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_cmd_dq_current(cur).map_err(|e| e.to_string())?;
            Ok(json!({"op":"dq_current","cur": cur}))
        }
        Some(_) => Err("dq_current is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque 刹车(表2 模式 15):三相短接阻尼刹车。一次性命令。
fn handle_brake(ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.brake().map_err(|e| e.to_string())?;
            Ok(json!({"op":"brake","mode": 15}))
        }
        Some(_) => Err("brake is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque 运行模式切换(`01 00 <mode>` 写模式寄存器 0x000):
/// mode 0-17(表2;0=停止并清错,8=DQ 电压,9=DQ 电流,10=位置,15=刹车)。
/// 真发帧,与 `ensure_mode`(仅校验)不同。
fn handle_run_mode(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let mode = v
        .get("mode")
        .and_then(Value::as_u64)
        .ok_or_else(|| "run_mode requires integer 'mode' (0-17)".to_string())?;
    if mode > 17 {
        return Err("run_mode 'mode' must be 0-17 (表2)".to_string());
    }
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.send_run_mode(mode as u8).map_err(|e| e.to_string())?;
            Ok(json!({"op":"run_mode","mode": mode}))
        }
        Some(_) => Err("run_mode is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque 通用寄存器写(`05 <addr> <val16>`):int16 值。
/// `config_class` 非 0 表示配置类寄存器(走扩展 0x8000|id 帧)。
/// 模式寄存器 0x000 不适用本 op,请用 run_mode。
fn handle_write_register(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let addr = v
        .get("addr")
        .and_then(Value::as_u64)
        .ok_or_else(|| "write_register requires integer 'addr'".to_string())? as u8;
    let value = v
        .get("value")
        .and_then(Value::as_i64)
        .ok_or_else(|| "write_register requires integer 'value' (int16)".to_string())?;
    if !(-32768..=32767).contains(&value) {
        return Err(format!("write_register 'value' must fit int16, got {value}"));
    }
    let config_class = v.get("config_class").and_then(Value::as_u64).unwrap_or(0) != 0;
    let info = parameter_info(addr);
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.write_register(addr, value as i16, config_class)
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "op": "write_register",
                "vendor": "hightorque",
                "addr": addr,
                "value": value,
                "config_class": config_class,
                "name": info.map(|i| i.name),
            }))
        }
        Some(_) => Err("write_register is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}

/// HighTorque 周期上报配置(`05 B4 02 <t_ms16>`,扩展帧):
/// `period_ms` >= 1 开启电机按周期主动发 27 01 状态帧(由被动反馈路径
/// 入状态缓存);`period_ms`=0 停止上报。
fn handle_periodic_report(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let period_ms = v
        .get("period_ms")
        .and_then(Value::as_u64)
        .ok_or_else(|| "periodic_report requires integer 'period_ms' (>=1 enable, 0 stop)".to_string())?;
    if period_ms > u16::MAX as u64 {
        return Err(format!("periodic_report 'period_ms' must fit uint16, got {period_ms}"));
    }
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            m.set_periodic_report(period_ms as u16)
                .map_err(|e| e.to_string())?;
            Ok(json!({
                "op": "periodic_report",
                "period_ms": period_ms,
                "enabled": period_ms > 0,
            }))
        }
        Some(_) => Err("periodic_report is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
    }
}
