use crate::args::{get_f32, get_i16, get_str, get_u16_hex_or_dec, get_u64};
use motor_core::bus::{open_transport, CanBus, Transport, TransportParams};
use motor_core::CanFrame;
use motor_vendor_hightorque::{
    parameter_info, HightorqueController, HightorqueFeedbackState, ReadCmd, RegisterType,
    RegisterValue,
};
use std::collections::HashMap;
use std::convert::TryFrom;
use std::sync::Arc;
use std::time::{Duration, Instant};

const TWO_PI: f32 = std::f32::consts::PI * 2.0;

/// 高创(HT)广播发现:vendor crate 无 scan API,广播查询必须裸总线手写
/// 17 01 帧 + 收 27 01 回帧(收敛 2026-09-29 后,这里是 CLI 仅存的 HT
/// 裸总线代码,仅用于发现;控制/状态/参数全部走 vendor)。
#[derive(Debug, Clone, Copy)]
struct HtScanHit {
    motor_id: u16,
    pos_raw: i16,
    vel_raw: i16,
    tqe_raw: i16,
}

fn send_ht_scan_query(bus: &dyn CanBus, motor_id: u16) -> Result<(), Box<dyn std::error::Error>> {
    let payload = [0x17u8, 0x01, 0, 0, 0, 0, 0, 0];
    bus.send(CanFrame {
        arbitration_id: u32::from(0x8000u16 | motor_id),
        data: payload,
        dlc: payload.len() as u8,
        is_extended: true,
        is_rx: false,
    })?;
    Ok(())
}

fn decode_ht_scan_reply(frame: CanFrame) -> Option<HtScanHit> {
    if frame.dlc < 8 || frame.data[0] != 0x27 || frame.data[1] != 0x01 {
        return None;
    }
    // 电机把自身 id 放在回复 ID 高字节(标准/扩展帧皆然,v2.0.0 固件
    // motor.c::motor_process_state_all:`id = Identifier >> 8`),回复 dest=0。
    if (frame.arbitration_id & 0x00FF) != 0 {
        return None;
    }
    let motor_id = ((frame.arbitration_id >> 8) & 0x7F) as u16;
    Some(HtScanHit {
        motor_id,
        pos_raw: i16::from_le_bytes([frame.data[2], frame.data[3]]),
        vel_raw: i16::from_le_bytes([frame.data[4], frame.data[5]]),
        tqe_raw: i16::from_le_bytes([frame.data[6], frame.data[7]]),
    })
}

fn wait_ht_scan_reply(
    bus: &dyn CanBus,
    motor_id: u16,
    timeout: Duration,
) -> Result<Option<HtScanHit>, Box<dyn std::error::Error>> {
    let deadline = Instant::now() + timeout;
    while Instant::now() < deadline {
        let left = deadline.saturating_duration_since(Instant::now());
        if let Some(frame) = bus.recv(left.min(Duration::from_millis(20)))? {
            if let Some(hit) = decode_ht_scan_reply(frame) {
                if hit.motor_id == motor_id {
                    return Ok(Some(hit));
                }
            }
        }
    }
    Ok(None)
}

fn print_scan_hit(prefix: &str, s: HtScanHit) {
    println!(
        "{} id={} pos_raw={} vel_raw={} tqe_raw={} pos_turn={:+.4} vel_rps={:+.4}",
        prefix,
        s.motor_id,
        s.pos_raw,
        s.vel_raw,
        s.tqe_raw,
        s.pos_raw as f32 * 0.0001,
        s.vel_raw as f32 * 0.00025
    );
}

/// 收敛(2026-09-29):read/ping 打印 vendor 解码后的物理量(带力矩系数
/// 补偿与 status/fault/温度),取代旧手写 raw + 自行换算。
fn print_state(prefix: &str, s: &HightorqueFeedbackState) {
    println!(
        "{} id={} arb_id=0x{:X} status={} fault={} pos={:+.4}rad vel={:+.4}rad/s torq={:+.3}Nm t_mos={:.1}C t_rotor={:.1}C",
        prefix,
        s.can_id,
        s.arbitration_id,
        s.status_code,
        s.fault_code,
        s.pos,
        s.vel,
        s.torq,
        s.t_mos,
        s.t_rotor
    );
}

/// raw int16 刻度 → 物理量(pos: 1 raw = 0.0001 圈;vel: 1 raw = 0.00025 圈/s)。
fn pos_rad_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-pos") {
        let raw = get_i16(args, "raw-pos", 0)?;
        return Ok(raw as f32 * 0.0001 * TWO_PI);
    }
    if args.contains_key("pos-deg") {
        let deg = get_f32(args, "pos-deg", 0.0)?;
        return Ok(deg.to_radians());
    }
    get_f32(args, "pos", 0.0)
}

fn vel_rad_s_from_args(args: &HashMap<String, String>) -> Result<f32, String> {
    if args.contains_key("raw-vel") {
        let raw = get_i16(args, "raw-vel", 0)?;
        return Ok(raw as f32 * 0.00025 * TWO_PI);
    }
    if args.contains_key("vel-deg-s") {
        let deg_s = get_f32(args, "vel-deg-s", 0.0)?;
        return Ok(deg_s.to_radians());
    }
    get_f32(args, "vel", 0.0)
}

fn tau_nm_from_args(args: &HashMap<String, String>) -> Result<Option<f32>, String> {
    if args.contains_key("raw-tqe") {
        return Err(
            "raw-tqe is no longer accepted here: torque is physical Nm (use tau); encoding is vendor-side with model torque coeff".to_string(),
        );
    }
    if args.contains_key("tau") {
        return Ok(Some(get_f32(args, "tau", 0.0)?));
    }
    Ok(None)
}

/// `--ty`(0|i8 / 1|i16 / 2|i32 / 3|f32);缺省回落寄存器表中已核实的类型,
/// 两者皆无则报错(规范表未覆盖所有寄存器,盲读会按错误宽度解码)。
fn reg_ty_from_args(
    args: &HashMap<String, String>,
    addr: u8,
) -> Result<RegisterType, Box<dyn std::error::Error>> {
    if let Some(raw) = args.get("ty") {
        let ty = match raw.trim() {
            "0" | "i8" | "int8" => RegisterType::Int8,
            "1" | "i16" | "int16" => RegisterType::Int16,
            "2" | "i32" | "int32" => RegisterType::Int32,
            "3" | "f32" | "float" => RegisterType::Float,
            _ => {
                return Err(format!(
                    "invalid --ty '{raw}' (expected 0|i8, 1|i16, 2|i32, 3|f32)"
                )
                .into())
            }
        };
        return Ok(ty);
    }
    match parameter_info(addr).and_then(|i| i.ty) {
        Some(ty) => Ok(ty),
        None => Err(format!(
            "register 0x{addr:02X} data type unknown; pass --ty (0|i8, 1|i16, 2|i32, 3|f32)"
        )
        .into()),
    }
}

fn reg_value_text(rv: &RegisterValue) -> String {
    match rv {
        RegisterValue::I8(x) => format!("{x}"),
        RegisterValue::I16(x) => format!("{x}"),
        RegisterValue::I32(x) => format!("{x}"),
        RegisterValue::F32(x) => format!("{x:.4}"),
    }
}

/// Open the CAN bus for the requested transport. Routes universal transports
/// (socketcan / mcu-serial) through `open_transport` so the platform driver
/// construction lives in one place (core). HighTorque uses standard CAN only,
/// so socketcanfd is rejected (it is not a CAN-FD device here); damiao-only
/// transports are rejected.
fn open_hightorque_bus(
    transport: &str,
    channel: &str,
    serial_port: &str,
    serial_baud: u32,
) -> Result<Arc<dyn CanBus>, Box<dyn std::error::Error>> {
    let p = TransportParams {
        channel,
        serial_port,
        serial_baud,
    };
    let bus: Arc<dyn CanBus> = match transport {
        "auto" | "socketcan" => open_transport(Transport::SocketCan, &p)?,
        "mcu-serial" => open_transport(Transport::McuSerial, &p)?,
        "socketcanfd" => {
            return Err(
                "transport socketcanfd unsupported (hightorque uses standard CAN only)".into(),
            )
        }
        "dm-serial" | "dm-device" => {
            return Err(format!(
                "transport {transport} is damiao-only (hightorque supports auto|socketcan|mcu-serial)"
            )
            .into());
        }
        _ => {
            return Err(format!(
                "unknown HighTorque transport: {transport} (expected auto|socketcan|mcu-serial)"
            )
            .into());
        }
    };
    Ok(bus)
}

/// vendor 无对应命令的裸诊断帧已全部消灭(2026-09-30):tqe/volt/cur/brake/
/// 周期上报配置五类命令全部走 vendor 原语,CLI 的 HT 裸总线代码只剩 scan
/// (vendor 无 scan API,广播发现必须手写 17 01 帧 + 收 27 01 回帧)。

pub fn run_hightorque(
    args: &HashMap<String, String>,
    channel: &str,
    model: &str,
    motor_id: u16,
    feedback_id: u16,
) -> Result<(), Box<dyn std::error::Error>> {
    let mode = get_str(args, "mode", "ping");
    let loop_n = get_u64(args, "loop", 1)?;
    let dt_ms = get_u64(args, "dt-ms", 20)?;
    let transport = get_str(args, "transport", "auto");
    let serial_port = get_str(args, "serial-port", "/dev/ttyACM0");
    let serial_baud_u64 = get_u64(args, "serial-baud", 921600)?;
    let serial_baud = u32::try_from(serial_baud_u64)
        .map_err(|_| format!("invalid --serial-baud (too large): {serial_baud_u64}"))?;
    let bus = open_hightorque_bus(&transport, channel, &serial_port, serial_baud)?;

    if mode == "scan" {
        let start_id = get_u16_hex_or_dec(args, "start-id", 1)?.clamp(1, 127);
        let end_id = get_u16_hex_or_dec(args, "end-id", 32)?.clamp(1, 127);
        if start_id > end_id {
            return Err("invalid scan range after clamp (start-id > end-id)".into());
        }
        println!(
            "[scan] probing hightorque IDs {}..{} on {} by 0x17/0x01",
            start_id, end_id, channel
        );
        let mut hits = 0usize;
        for id in start_id..=end_id {
            send_ht_scan_query(bus.as_ref(), id)?;
            if let Some(s) = wait_ht_scan_reply(bus.as_ref(), id, Duration::from_millis(80))? {
                print_scan_hit("[hit]", s);
                hits += 1;
            }
            std::thread::sleep(Duration::from_millis(2));
        }
        println!("[scan] done vendor=hightorque hits={hits}");
        bus.shutdown()?;
        return Ok(());
    }

    // 收敛(2026-09-29):控制/状态/参数命令全部经 vendor crate
    // (HightorqueController 包住 CoreController + 后台收帧线程),
    // 电机经 add_motor 注册,--model 从此真正生效(型号决定力矩补偿系数,
    // 未知型号码会在 add_motor 处被拒)。2026-09-30 起裸诊断帧路径
    // (diag_bus)删除,原 diag 模式(tqe/volt/cur/brake/timed-read)
    // 全部改走 vendor 物理量原语。
    let ctrl = HightorqueController::new(bus);
    let motor = ctrl
        .add_motor(motor_id, feedback_id, model)
        .map_err(|e| format!("add motor failed: {e}"))?;

    let mut send_count = loop_n.max(1);
    if matches!(mode.as_str(), "ping" | "read" | "read-reg" | "write-reg") {
        send_count = 1;
    }

    for i in 0..send_count {
        match mode.as_str() {
            "ping" | "read" => {
                motor
                    .request_motor_feedback(Duration::from_millis(500))
                    .map_err(|e| e.to_string())?;
                match motor.latest_state() {
                    Some(s) => print_state("[ok]", &s),
                    None => {
                        return Err(format!(
                            "hightorque {} timeout on id={} (request cmd=0x17,0x01)",
                            mode, motor_id
                        )
                        .into());
                    }
                }
            }
            "pos" => {
                // 普通位置模式 07 07:vendor send_cmd_pos_classic,力矩为
                // 物理量 Nm(型号自适应补偿),缺省无限制(0x8000)。
                let pos = pos_rad_from_args(args)?;
                let tqe = tau_nm_from_args(args)?;
                println!(
                    "[tx] mode=pos id={} pos={:+.4}rad tqe_limit={:?}",
                    motor_id, pos, tqe
                );
                motor
                    .send_cmd_pos_classic(pos, tqe)
                    .map_err(|e| e.to_string())?;
            }
            "vel" => {
                let vel = vel_rad_s_from_args(args)?;
                println!("[tx] mode=vel id={} vel={:+.4}rad/s", motor_id, vel);
                motor.send_cmd_vel(vel).map_err(|e| e.to_string())?;
            }
            "tqe" => {
                // 力矩模式 05 13:vendor send_cmd_torque,物理量 Nm
                // (0.01 N·m/LSB 基准 + 型号力矩系数补偿)。
                if args.contains_key("raw-tqe") {
                    return Err(
                        "raw-tqe is no longer accepted: torque is physical Nm (use --tau); encoding is vendor-side with model torque coeff".into(),
                    );
                }
                let tau = get_f32(args, "tau", 0.0)?;
                println!(
                    "[tx] mode=tqe id={} tau={:+.3}Nm (model coeff k={:.3})",
                    motor_id,
                    tau,
                    motor.torque_coeff().k
                );
                motor.send_cmd_torque(tau).map_err(|e| e.to_string())?;
            }
            "mit" => {
                // v2.0.0 MIT(电机固件 v4.6.0+):vendor send_cmd_mit,位打包
                // pos(16)/vel(12)/tqe(12)/kp(12)/kd(12),量程内饱和。
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                let tau = get_f32(args, "tau", 0.0)?;
                let kp = get_f32(args, "kp", 0.0)?;
                let kd = get_f32(args, "kd", 0.0)?;
                println!(
                    "[tx] mode=mit id={} pos={:+.4}rad vel={:+.4}rad/s tau={:+.3}Nm kp={:.3} kd={:.3}",
                    motor_id, pos, vel, tau, kp, kd
                );
                motor
                    .send_cmd_mit(pos, vel, kp, kd, tau)
                    .map_err(|e| e.to_string())?;
            }
            "volt" => {
                // DQ 电压模式 01 00 08 05 1B:vendor send_cmd_dq_voltage,
                // 物理量 V(0.1 V/LSB),模式写与寄存器写拼一帧。
                if args.contains_key("raw-vol") {
                    return Err(
                        "raw-vol is no longer accepted: voltage is physical V (use --volt); scale is 0.1 V/LSB vendor-side".into(),
                    );
                }
                let volt = get_f32(args, "volt", 0.0)?;
                println!("[tx] mode=volt id={} volt={:+.3}V", motor_id, volt);
                motor
                    .send_cmd_dq_voltage(volt)
                    .map_err(|e| e.to_string())?;
            }
            "cur" => {
                // DQ 电流模式 01 00 09 05 1C:vendor send_cmd_dq_current,
                // 物理量 A(0.1 A/LSB),拼一帧。
                if args.contains_key("raw-cur") {
                    return Err(
                        "raw-cur is no longer accepted: current is physical A (use --cur); scale is 0.1 A/LSB vendor-side".into(),
                    );
                }
                let cur = get_f32(args, "cur", 0.0)?;
                println!("[tx] mode=cur id={} cur={:+.3}A", motor_id, cur);
                motor
                    .send_cmd_dq_current(cur)
                    .map_err(|e| e.to_string())?;
            }
            "set-mode" => {
                // 运行模式切换 01 00 <mode>:写模式寄存器 0x000(表2,0-17)。
                // 与 stop/clear-error(模式 0)互补,可显式进 DQ 电压(8)/
                // DQ 电流(9)/位置(10)/刹车(15)等模式。
                let run_mode = get_u64(args, "run-mode", 0)?;
                if run_mode > 17 {
                    return Err(format!(
                        "run-mode must be 0-17 (表2), got {run_mode}"
                    )
                    .into());
                }
                println!(
                    "[tx] mode=set-mode id={} run-mode={} (register 0x000)",
                    motor_id, run_mode
                );
                motor
                    .send_run_mode(run_mode as u8)
                    .map_err(|e| e.to_string())?;
            }
            "pos-vel-tqe" => {
                // 协同模式 07 35:vendor send_cmd_pos_vel(速度上限换算),
                // 力矩由 vendor 固定为无限制(0x8000),raw-tqe 不再接受。
                let pos = pos_rad_from_args(args)?;
                let vel = vel_rad_s_from_args(args)?;
                println!(
                    "[tx] mode=pos-vel-tqe id={} pos={:+.4}rad vel={:+.4}rad/s (torque unlimited)",
                    motor_id, pos, vel
                );
                motor
                    .send_cmd_pos_vel(pos, vel)
                    .map_err(|e| e.to_string())?;
            }
            "pos-vel-acc" => {
                // 梯形模式 07 2d(固件 v4.6.0+):vendor send_cmd_pos_vel_acc,
                // 位置 + 速度上限 + 加速度上限,均物理量。
                let pos = pos_rad_from_args(args)?;
                let vlim = get_f32(args, "vlim", 1.0)?;
                let acc = get_f32(args, "acc", 1.0)?;
                println!(
                    "[tx] mode=pos-vel-acc id={} pos={:+.4}rad vlim={:+.4}rad/s acc={:+.4}rad/s^2",
                    motor_id, pos, vlim, acc
                );
                motor
                    .send_cmd_pos_vel_acc(pos, vlim, acc)
                    .map_err(|e| e.to_string())?;
            }
            "read-reg" => {
                // 通用寄存器读 17 族:vendor read_registers。返回 raw 值,
                // 物理量标度换算按寄存器功能表由调用方完成;名称/类型
                // 取自 vendor 寄存器表(parameter_info)。
                let addr = match args.get("addr") {
                    Some(_) => get_u16_hex_or_dec(args, "addr", 0)? as u8,
                    None => {
                        return Err(
                            "read-reg requires --addr (hex or decimal, 0x00-0x45)".into(),
                        )
                    }
                };
                let count = get_u64(args, "count", 1)?.clamp(1, 3) as u8;
                let ty = reg_ty_from_args(args, addr)?;
                let timeout_ms = get_u64(args, "timeout-ms", 500)?;
                let vals = motor
                    .read_registers(&[ReadCmd { addr, ty, count }], Duration::from_millis(timeout_ms))
                    .map_err(|e| e.to_string())?;
                let name = parameter_info(addr)
                    .map(|i| i.name)
                    .unwrap_or("unknown");
                for (a, rv) in &vals {
                    println!(
                        "[ok] mode=read-reg id={} addr=0x{:02X} name={} value={}",
                        motor_id,
                        a,
                        name,
                        reg_value_text(rv)
                    );
                }
            }
            "stop" => {
                motor.disable().map_err(|e| e.to_string())?;
            }
            "brake" => {
                // 刹车(表2 模式 15):vendor brake,三相短接阻尼刹车。
                println!("[tx] mode=brake id={} (run mode 15)", motor_id);
                motor.brake().map_err(|e| e.to_string())?;
            }
            "conf-write" => {
                // 落盘到 flash(05 B3),vendor store_parameters。
                motor.store_parameters().map_err(|e| e.to_string())?;
            }
            "rezero" => {
                // 0x40 置零 + 自动落盘,vendor set_zero_position。
                motor.set_zero_position().map_err(|e| e.to_string())?;
            }
            "periodic-report" | "timed-read" => {
                // 周期上报配置 05 B4 02 <t_ms>:vendor set_periodic_report
                // (扩展帧)。period-ms >= 1 开启电机主动发 27 01 状态帧,
                // 0 停止。timed-read 为旧名别名,兼容既有脚本。
                let t_ms = get_u64(args, "period-ms", 100)?;
                if t_ms > u16::MAX as u64 {
                    return Err(format!(
                        "period-ms must fit uint16, got {t_ms}"
                    )
                    .into());
                }
                println!(
                    "[tx] mode=periodic-report id={} period_ms={} ({})",
                    motor_id,
                    t_ms,
                    if t_ms == 0 { "stop" } else { "start" }
                );
                motor
                    .set_periodic_report(t_ms as u16)
                    .map_err(|e| e.to_string())?;
            }
            "write-reg" => {
                // 通用寄存器写 05 <addr> <val16>:vendor write_register。
                // --config 1 表示配置类寄存器(扩展 0x8000|id 帧),
                // 缺省 0 走控制通道(控制目标类)。
                let addr = match args.get("addr") {
                    Some(_) => get_u16_hex_or_dec(args, "addr", 0)? as u8,
                    None => {
                        return Err(
                            "write-reg requires --addr (hex or decimal, 0x00-0x45)".into(),
                        )
                    }
                };
                let value = get_i16(args, "value", 0)?;
                let config = get_u64(args, "config", 0)? != 0;
                let name = parameter_info(addr)
                    .map(|i| i.name)
                    .unwrap_or("unknown");
                println!(
                    "[tx] mode=write-reg id={} addr=0x{addr:02X} name={} value={} frame={}",
                    motor_id,
                    name,
                    value,
                    if config { "config(extended)" } else { "control" }
                );
                motor
                    .write_register(addr, value, config)
                    .map_err(|e| e.to_string())?;
                // --store 1:写完补一帧 conf-write(05 B3)落盘,对齐 Damiao/RS
                // write-param 的 --store 组合便利。默认 0 只写 RAM;落盘为整机
                // 保存(非单寄存器),固件建议重新上电后生效。write-reg 强制
                // 单次发送(send_count=1),无循环重复写 flash 问题。
                let store_after_write = get_u64(args, "store", 0)? != 0;
                if store_after_write {
                    motor.store_parameters().map_err(|e| e.to_string())?;
                    println!("[ok] conf-write sent (store=1; 整机落盘,建议重新上电使配置生效)");
                }
            }
            _ => {
                return Err(format!(
                "unknown hightorque mode: {}. expected ping|scan|read|mit|pos|vel|tqe|volt|cur|set-mode|pos-vel-tqe|pos-vel-acc|read-reg|write-reg|stop|brake|rezero|conf-write|periodic-report",
                mode
            )
            .into());
            }
        }
        if send_count > 1 {
            println!("[loop] #{i} sent mode={mode}");
        }
        if i + 1 < send_count {
            std::thread::sleep(Duration::from_millis(dt_ms));
        }
    }

    ctrl.close_bus()?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::reg_ty_from_args;
    use motor_vendor_hightorque::RegisterType;

    use std::collections::HashMap;

    fn args(pairs: &[(&str, &str)]) -> HashMap<String, String> {
        pairs
            .iter()
            .map(|(k, v)| (k.to_string(), v.to_string()))
            .collect()
    }

    #[test]
    fn read_reg_ty_explicit_wins_over_table() {
        // 0x01 位置在寄存器表中为 Int16,显式 --ty 覆盖为 Int32。
        let a = args(&[("ty", "2")]);
        assert!(matches!(
            reg_ty_from_args(&a, 0x01),
            Ok(RegisterType::Int32)
        ));
    }

    #[test]
    fn read_reg_ty_falls_back_to_table() {
        let a: HashMap<String, String> = HashMap::new();
        // 0x01(位置)表内已核实 Int16;0x0F(错误代码)表内未给出类型。
        assert!(matches!(
            reg_ty_from_args(&a, 0x01),
            Ok(RegisterType::Int16)
        ));
        assert!(reg_ty_from_args(&a, 0x0F).is_err());
    }

    #[test]
    fn read_reg_ty_accepts_string_aliases() {
        for (raw, want) in [
            ("i8", RegisterType::Int8),
            ("i16", RegisterType::Int16),
            ("i32", RegisterType::Int32),
            ("f32", RegisterType::Float),
            ("3", RegisterType::Float),
        ] {
            let a = args(&[("ty", raw)]);
            assert!(matches!(reg_ty_from_args(&a, 0x01), Ok(t) if t == want));
        }
    }

    #[test]
    fn read_reg_ty_rejects_invalid() {
        let a = args(&[("ty", "9")]);
        assert!(reg_ty_from_args(&a, 0x01).is_err());
    }
}
