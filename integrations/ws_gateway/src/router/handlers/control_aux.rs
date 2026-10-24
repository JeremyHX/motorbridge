use crate::commands::{as_f32, as_u64};
use crate::model::{ControllerHandle, MotorHandle};
use crate::session::SessionCtx;
use motor_vendor_hightorque::{parameter_info, ReadCmd};
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
/// 字段存在但非数字时报错拒绝(评审 A2:限幅是安全字段,类型错误
/// 不能静默降级为"无限制")。一次性命令,不进入 continuous 活动指令
/// (与 myactuator "pos" 同型)。
fn handle_pos_classic(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    let pos = as_f32(v, "pos", 0.0);
    // 区分"字段缺席 = 故意不限力矩"与"字段存在但类型错 = 客户端笔误":
    // 后者报错,避免 "tqe":"1.5" 被静默编码为 0x8000 无限制力矩。
    let tqe = match v.get("tqe") {
        None => None,
        Some(value) => Some(
            value
                .as_f64()
                .map(|x| x as f32)
                .ok_or_else(|| "pos_classic 'tqe' must be a number (Nm)".to_string())?,
        ),
    };
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
        Some(_) => Err("version is supported for myactuator only".to_string()),
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
            // 2026-10-21 起 vendor 阻塞式状态查询为 request_fresh_state
            // (request_motor_feedback 归还 DM 非阻塞语义)。
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            let s = m.request_fresh_state(timeout).map_err(|e| e.to_string())?;
            Ok(json!({
                "motor_id": s.can_id,
                "arbitration_id": s.arbitration_id,
                "pos": s.pos,
                "vel": s.vel,
                "torq": s.torq,
                "status_code": s.status_code,
                "fault_code": s.fault_code,
            }))
        }
        (Some(_), Some(_)) => Err("read op is reserved for hightorque".to_string()),
        _ => Err("motor not connected".to_string()),
    }
}

/// HighTorque 通用寄存器读取(17 族):从 `addr` 连续读 `count`(1-3) 个寄存器。
/// 2026-10-22 起:读取**固定按 Int16**,报文 `ty` 字段已删除——显式携带
/// `ty` 直接报错(不静默忽略,旧客户端迁移时能立刻发现);返回值为
/// **换算后物理量**(按 vendor 寄存器表 quantity 归类:位置/速度→rad 族、
/// 力矩套型号 k、电流/电压/温度/PWM/PID 按各自 scale;无换算关系的
/// 寄存器 1:1)。每行的 name/unit 按该行自身地址查表(而非基础地址)。
fn handle_read_register(v: &Value, ctx: &mut SessionCtx) -> Result<Value, String> {
    ctx.ensure_connected()?;
    match ctx.motor.as_ref() {
        Some(MotorHandle::Hightorque(m)) => {
            // B2 修复(2026-10-21):越界地址(> 0xFF)直接拒绝,不再
            // `as u8` 静默截断(addr=300 → 0x2C 读错寄存器)。
            let addr_raw = v
                .get("addr")
                .and_then(Value::as_u64)
                .ok_or_else(|| "read_register requires integer 'addr' (0x00-0xFF)".to_string())?;
            let addr = u8::try_from(addr_raw).map_err(|_| {
                format!("read_register 'addr' out of range 0x00-0xFF, got {addr_raw}")
            })?;
            if v.get("ty").is_some() {
                return Err(
                    "read_register 'ty' was removed: reads are fixed Int16 (2026-10-22)"
                        .to_string(),
                );
            }
            let count = as_u64(v, "count", 1).clamp(1, 3) as u8;
            let timeout = Duration::from_millis(as_u64(v, "timeout_ms", 500));
            let vals = m
                .read_registers(&[ReadCmd { addr, count }], timeout)
                .map_err(|e| e.to_string())?;
            // C2:每行按自身地址查名称/单位,而非基础地址。
            let registers: Vec<Value> = vals
                .iter()
                .map(|(a, val)| {
                    let (name, unit) = match parameter_info(*a) {
                        Some(i) => (
                            json!(i.name),
                            json!(i.quantity.map(|q| q.unit_label()).unwrap_or("")),
                        ),
                        None => (json!("unknown"), json!("")),
                    };
                    json!({
                        "addr": a,
                        "name": name,
                        "unit": unit,
                        "value": val,
                    })
                })
                .collect();
            Ok(json!({
                "op": "read_register",
                "vendor": "hightorque",
                "addr": addr,
                "count": count,
                "registers": registers,
            }))
        }
        Some(_) => Err("read_register is supported for hightorque only".to_string()),
        None => Err("motor not connected".to_string()),
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
    // B2 修复(2026-10-21):同 read 侧,越界地址拒绝,不再 as u8 截断。
    let addr_raw = v
        .get("addr")
        .and_then(Value::as_u64)
        .ok_or_else(|| "write_register requires integer 'addr' (0x00-0xFF)".to_string())?;
    let addr = u8::try_from(addr_raw).map_err(|_| {
        format!("write_register 'addr' out of range 0x00-0xFF, got {addr_raw}")
    })?;
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::model::{Target, Transport, Vendor};
    use motor_core::bus::CanBus;
    use motor_core::test_support::MockBus;
    use motor_vendor_hightorque::HightorqueController;
    use std::sync::Arc;

    fn ht_ctx() -> (Arc<MockBus>, SessionCtx) {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        let ctrl = HightorqueController::new(bus.clone() as Arc<dyn CanBus>);
        let motor = ctrl.add_motor(1, 1, "hightorque").expect("add motor");
        let target = Target {
            vendor: Vendor::Hightorque,
            transport: Transport::SocketCan,
            channel: "mock".to_string(),
            serial_port: String::new(),
            serial_baud: 0,
            dm_device_type: String::new(),
            dm_channel: String::new(),
            model: "hightorque".to_string(),
            motor_id: 1,
            feedback_id: 1,
        };
        (
            bus,
            SessionCtx {
                target,
                controller: Some(ControllerHandle::Hightorque(ctrl)),
                motor: Some(MotorHandle::Hightorque(motor)),
                active: None,
            },
        )
    }

    /// 评审 A2 回归:pos_classic 的 tqe 传非数字 JSON 类型(字符串/布尔/
    /// null)时报错拒绝,且报错发生在任何控制帧发出之前——限幅是安全
    /// 字段,类型错不得静默降级为 0x8000 无限制力矩。
    #[test]
    fn pos_classic_rejects_non_numeric_tqe_without_sending() {
        for bad in [json!("1.5"), json!(true), json!(null), json!([1.5])] {
            let (bus, mut ctx) = ht_ctx();
            let r = handle_pos_classic(&json!({ "pos": 1.0, "tqe": bad }), &mut ctx);
            let err = r.expect_err("non-numeric tqe must be rejected");
            assert!(err.contains("tqe"), "error should name the field: {err}");
            let sent = bus.sent.lock().expect("sent");
            assert!(
                sent.is_empty(),
                "no control frame may be sent on type error, got {sent:?}"
            );
        }
    }

    /// B2 回归(2026-10-21):寄存器地址越界(> 0xFF)直接拒绝,不再
    /// `as u8` 静默截断。读、写两个 op 都验证(旧实现 addr=300 会截成
    /// 0x2C 读/写错误寄存器),且拒绝发生在发帧之前(总线零帧)。
    #[test]
    fn read_write_register_reject_out_of_range_addr() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 300 }), &mut ctx);
        assert!(r.is_err());
        assert!(r.unwrap_err().contains("out of range"));

        let r = handle_write_register(&json!({ "addr": 0x145, "value": 0 }), &mut ctx);
        assert!(r.is_err());
        assert!(r.unwrap_err().contains("out of range"));

        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "越界地址应在发帧前拒绝");

        // 合法域边界:入参域上限 0xFF 现属表外(vendor 表校验拒绝,
        // 见 read_write_register_rejects_unknown_table_addr);取表内
        // 最高地址 0x45 作为"入参放行且发帧"的边界用例(会走到超时,
        // 但帧已发出)
        let (_bus, mut ctx) = ht_ctx();
        let _ = handle_read_register(&json!({ "addr": 0x45, "timeout_ms": 10 }), &mut ctx);
        let sent = _bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "0x45 边界地址应正常发帧");
        assert_eq!(sent[0].data[1], 0x45);
    }

    /// 2026-10-22 追补决策回归:缺号/表外地址拒绝访问——0x00-0xFF 入参
    /// 域内的洞(缺号 0x0A)与 0x46-0xFF(表外 0x50/0xFF)一律报错且
    /// 总线零发帧(vendor `ensure_known_span` 在发帧前拒绝);写路径同拒。
    #[test]
    fn read_write_register_rejects_unknown_table_addr() {
        let (bus, mut ctx) = ht_ctx();
        for addr in [0x0Au32, 0x50, 0xFF] {
            let r = handle_read_register(&json!({ "addr": addr }), &mut ctx);
            assert!(r.is_err(), "读 0x{addr:02X} 应被表校验拒绝");
            assert!(
                r.unwrap_err().contains("not in the parameter table"),
                "报错应指明不在寄存器表"
            );
        }
        let r = handle_write_register(&json!({ "addr": 0x0A, "value": 0 }), &mut ctx);
        assert!(r.is_err(), "写缺号地址应被拒绝");
        assert!(r.unwrap_err().contains("not in the parameter table"));
        let r = handle_write_register(&json!({ "addr": 0x50, "value": 0 }), &mut ctx);
        assert!(r.is_err(), "写表外地址应被拒绝");
        assert!(r.unwrap_err().contains("not in the parameter table"));
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝应发生在发帧之前");
    }

    /// 2026-10-22 决策回归:读取固定 Int16——0x0E(温度)读帧 cmd=0x15
    /// (read|Int16|count1),扩展 0x8000|id、dlc 2。Mock 总线无回复,
    /// 阻塞读超时属预期。
    #[test]
    fn read_register_defaults_unverified_addr_to_int16() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 14, "timeout_ms": 10 }), &mut ctx);
        assert!(r.is_err(), "no reply on mock bus: timeout expected");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "exactly one register read frame");
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert!(sent[0].is_extended, "register reads go on 0x8000|id");
        assert_eq!(sent[0].dlc, 2);
        assert_eq!(
            &sent[0].data[..2],
            &[0x15, 0x0E],
            "cmd 0x15 = read | Int16 | count 1"
        );
    }

    /// 2026-10-22:报文显式携带 `ty` 直接报错(字段已删除,读取固定
    /// Int16),不静默忽略;拒绝发生在发帧之前(总线零帧)。
    #[test]
    fn read_register_rejects_removed_ty_field() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_read_register(&json!({ "addr": 14, "ty": 1, "timeout_ms": 10 }), &mut ctx);
        let err = r.expect_err("ty must be rejected");
        assert!(err.contains("ty"), "error should name the field: {err}");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "ty 拒绝应在发帧前");
    }

    /// 字段缺席 = 故意不限力矩(None);数字正常解析。两种合法形态
    /// 都照常下发 07 07 帧。
    #[test]
    fn pos_classic_accepts_absent_and_numeric_tqe() {
        let (bus, mut ctx) = ht_ctx();
        let r = handle_pos_classic(&json!({ "pos": 1.0 }), &mut ctx).expect("absent tqe ok");
        assert_eq!(r["tqe"], serde_json::Value::Null);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "absent tqe still sends the 07 07 frame");

        let (_bus, mut ctx) = ht_ctx();
        let r = handle_pos_classic(
            &json!({ "pos": 1.0, "tqe": 1.5 }),
            &mut ctx,
        )
        .expect("numeric tqe ok");
        assert_eq!(r["tqe"], json!(1.5));
    }
}
