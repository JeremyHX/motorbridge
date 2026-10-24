use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_set_can_timeout_ms(motor: *mut MotorHandle, timeout_ms: u32) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let reg_value = timeout_ms.saturating_mul(20);
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .write_register_u32(9, reg_value)
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hexfellow(_) => {
            Err("set_can_timeout_ms is not supported for Hexfellow".to_string())
        }
        MotorHandleInner::MyActuator(_) => {
            Err("set_can_timeout_ms is not supported for MyActuator".to_string())
        }
        MotorHandleInner::Robstride(m) => m
            .write_parameter(0x7028, ParameterValue::U32(timeout_ms))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Hightorque(_) => {
            Err("set_can_timeout_ms is not supported for HighTorque".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    value: f32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_f32(rid, value).map_err(|e| e.to_string()),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_write_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    value: u32,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m.write_register_u32(rid, value).map_err(|e| e.to_string()),
        _ => Err("Damiao register write is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_f32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_f32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_get_register_u32(
    motor: *mut MotorHandle,
    rid: u8,
    timeout_ms: u32,
    out_value: *mut u32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or out_value is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Damiao(m) => m
            .get_register_u32(rid, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        _ => Err("Damiao register read is only available for Damiao motors".to_string()),
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping(
    motor: *mut MotorHandle,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping(Duration::from_millis(500))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_ping requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_ping_host_id(
    motor: *mut MotorHandle,
    host_id: u16,
    timeout_ms: u32,
    out_device_id: *mut u8,
    out_responder_id: *mut u8,
) -> i32 {
    if motor.is_null() || out_device_id.is_null() || out_responder_id.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .ping_with_host_id(host_id, Duration::from_millis(timeout_ms as u64))
            .map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_ping_host_id requires a RobStride motor".to_string())
        }
    };
    match rc {
        Ok(reply) => {
            unsafe {
                *out_device_id = reply.device_id;
                *out_responder_id = reply.responder_id;
            }
            0
        }
        Err(e) => {
            set_last_error(e);
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_param_f32_host_id(
    motor: *mut MotorHandle,
    param_id: u16,
    host_id: u16,
    timeout_ms: u32,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_value.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let out = unsafe { &mut *out_value };
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m
            .get_parameter_with_host_id(param_id, host_id, Duration::from_millis(timeout_ms as u64))
            .and_then(|value| match value {
                ParameterValue::F32(v) => Ok(v),
                _ => Err(motor_core::error::MotorError::Protocol(format!(
                    "parameter 0x{param_id:04X} is not f32"
                ))),
            })
            .map_err(|e| e.to_string())
            .map(|v| *out = v),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_get_param_f32_host_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_get_fault_report(
    motor: *mut MotorHandle,
    out_fault_raw: *mut u32,
    out_warning_raw: *mut u32,
) -> i32 {
    if motor.is_null() || out_fault_raw.is_null() || out_warning_raw.is_null() {
        set_last_error("motor or output pointer is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    match &*motor {
        MotorHandleInner::Robstride(m) => {
            let report = m.latest_fault_report();
            unsafe {
                *out_fault_raw = report.map(|r| r.fault_raw).unwrap_or(0);
                *out_warning_raw = report.map(|r| r.warning_raw).unwrap_or(0);
            }
            0
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            set_last_error("robstride_get_fault_report requires a RobStride motor");
            -1
        }
    }
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_device_id(
    motor: *mut MotorHandle,
    new_device_id: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => m.set_device_id(new_device_id).map_err(|e| e.to_string()),
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_set_device_id requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_set_active_report(
    motor: *mut MotorHandle,
    enabled: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Robstride(m) => {
            m.set_active_report(enabled != 0).map_err(|e| e.to_string())
        }
        MotorHandleInner::Damiao(_)
        | MotorHandleInner::Hexfellow(_)
        | MotorHandleInner::MyActuator(_)
        | MotorHandleInner::Hightorque(_) => {
            Err("robstride_set_active_report requires a RobStride motor".to_string())
        }
    };
    ffi_rc(rc)
}

/// HighTorque 通用寄存器读取(协议 §1.3 `17 <cmd> <addr>` 族)。
///
/// 2026-10-22 起:读取**固定按 Int16**(`cmd[3:2]` 恒 01),`ty` 参数已
/// 从签名删除;返回值为**换算后物理量**(位置/速度/加速度→rad 族,
/// 力矩套型号 k/d,电流/电压/温度/PWM/PID 按各自 scale;无换算关系
/// 的寄存器及表外地址 1:1 返回 raw)。
///
/// - `addr`: 寄存器地址(表1,0x00–0x45)
/// - `count`: 连续读取个数(1–3),地址从 `addr` 起连续递增(0xFF 后回绕)
/// - `out_values`: 写出 `count` 个 f32(换算后物理量)
///
/// 单地址读的简化封装;多段任意地址列表读保留在 vendor `read_registers`。
/// 仅 HighTorque 电机支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_get_register(
    motor: *mut MotorHandle,
    addr: u8,
    count: u8,
    timeout_ms: u32,
    out_values: *mut f32,
) -> i32 {
    if motor.is_null() || out_values.is_null() {
        set_last_error("motor or out_values is null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => m
            .read_registers(
                &[ReadCmd { addr, count }],
                Duration::from_millis(timeout_ms as u64),
            )
            .map(|vals| {
                // 返回列表按 count 连续递增;不足 count 时逐个写出已有值。
                unsafe {
                    for (i, (_, v)) in vals.iter().take(count as usize).enumerate() {
                        *out_values.add(i) = *v;
                    }
                };
            })
            .map_err(|e| e.to_string()),
        _ => Err("hightorque_get_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

/// HighTorque 非阻塞寄存器查询(DM 风格"请求即发送"):只发 `17` 族读
/// 请求帧(2026-10-22 起**固定 Int16**),立即返回不等回帧。回复由
/// CoreController 后台收帧线程解码,按地址写入电机寄存器缓存(带入缓
/// 时刻);之后 `hightorque_get_cached_register` 读最近值(非阻塞,已换算
/// 为物理量),或 `hightorque_get_register`/vendor `read_registers` 阻塞
/// 等新鲜值。供固定频率监测循环使用(发完即走,下一拍读缓存)。
///
/// - `addr`: 寄存器地址(表1,0x00–0x45)
/// - `count`: 连续读取个数(1–3)
///
/// 仅 HighTorque 电机支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_request_register(
    motor: *mut MotorHandle,
    addr: u8,
    count: u8,
) -> i32 {
    if motor.is_null() {
        set_last_error("motor is null");
        return -1;
    }
    if !(1..=3).contains(&count) {
        set_last_error("count must be in 1..=3");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => m
            .request_registers(&[ReadCmd { addr, count }])
            .map_err(|e| e.to_string()),
        _ => Err("hightorque_request_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

/// HighTorque 读寄存器缓存最近值(不发帧不等待)。
/// `out_has_value` 写 1 且 `out_value` 写**换算后物理量**(`f32`);从未
/// 收到过该地址回复时 `out_has_value` 写 0。注意:不区分新鲜/陈旧
/// (电机掉线后停留在最后一值);需要新鲜值用 `hightorque_get_register`
/// 或先 `hightorque_request_register` 发查询、隔一拍回读本缓存。
/// 仅 HighTorque 电机支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_get_cached_register(
    motor: *mut MotorHandle,
    addr: u8,
    out_has_value: *mut u8,
    out_value: *mut f32,
) -> i32 {
    if motor.is_null() || out_has_value.is_null() || out_value.is_null() {
        set_last_error("motor or out pointers are null");
        return -1;
    }
    let motor = lock_motor_inner!(motor, "motor is null");
    let rc = match &*motor {
        MotorHandleInner::Hightorque(m) => {
            match m.cached_register(addr) {
                Some(v) => unsafe {
                    *out_has_value = 1;
                    *out_value = v;
                },
                None => unsafe { *out_has_value = 0 },
            }
            Ok(())
        }
        _ => Err("hightorque_get_cached_register requires a HighTorque motor".to_string()),
    };
    ffi_rc(rc)
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::{CanBus, CanFrame};
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use motor_vendor_hightorque::HightorqueController;

    fn ht_handle() -> (Arc<MockBus>, MotorHandle) {
        let bus = Arc::new(MockBus::new());
        let ctrl = HightorqueController::new(Arc::clone(&bus) as Arc<dyn CanBus>);
        let motor = ctrl.add_motor(1, 0x101, "ht").expect("add motor");
        (
            bus,
            MotorHandle {
                inner: Mutex::new(MotorHandleInner::Hightorque(motor)),
            },
        )
    }

    fn rx_frame(id: u32, data: [u8; 8], dlc: u8, ext: bool) -> CanFrame {
        CanFrame {
            arbitration_id: id,
            data,
            dlc,
            is_extended: ext,
            is_rx: true,
        }
    }

    /// F4:通用寄存器读,固定 Int16 单地址(count=1)。回复 0x25(cmd=
    /// reply nibble|Int16<<2|count) + addr + i16 值。0x0F(错误代码,
    /// 表内未分类)→ 无换算关系,1:1 返回 raw。2026-10-22 追补后缺号/
    /// 表外地址在 vendor 层整段拒绝(见 hightorque_get_register_rejects_
    /// unknown_addr)。
    #[test]
    fn hightorque_get_register_int16_single() {
        let (bus, mut handle) = ht_handle();
        // 回复须在请求之后入缓(时间戳新鲜度): 模拟后台轮询线程在
        // 请求发出 10ms 后投递寄存器回复
        let deliver = {
            let m = match &*handle.inner.lock().expect("inner") {
                MotorHandleInner::Hightorque(m) => Arc::clone(m),
                _ => panic!("expected hightorque motor"),
            };
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(10));
                m.process_feedback_frame(rx_frame(
                        0x0100,
                        [0x25, 0x0F, 0x34, 0x12, 0, 0, 0, 0],
                        4,
                        true,
                    ))
                    .expect("merge reply");
            })
        };
        let mut out = [0f32; 3];
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x0F,
            1,
            500,
            out.as_mut_ptr(),
        );
        deliver.join().expect("join deliver thread");
        assert_eq!(rc, 0);
        assert_eq!(out[0], 0x1234 as f32);
        // 查询帧:扩展 0x8000|id,载荷 = 读命令 0x15(0x1X 族|Int16<<2|count1) + addr
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8001);
        assert_eq!(&sent[0].data[..2], &[0x15, 0x0F]);
    }

    /// F4:缺号/表外地址拒绝访问(2026-10-22 追补决策)——读返回
    /// rc=-1 且总线零发帧(拒绝发生在发帧之前)。
    #[test]
    fn hightorque_get_register_rejects_unknown_addr() {
        let (bus, mut handle) = ht_handle();
        let mut out = [0f32; 3];
        // 缺号(0x0A)
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x0A,
            1,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        // 表外(0x50)
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x50,
            1,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        // 连续读跨入缺号(0x2D 表内 + 0x2E 缺号)整段拒绝
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x2D,
            2,
            50,
            out.as_mut_ptr(),
        );
        assert_eq!(rc, -1);
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝应发生在发帧之前");
    }

    /// F4:非阻塞寄存器查询 + 缓存读取闭环:request_register 发帧即返回,
    /// get_cached_register 在回复入缓前 has=0、入缓后返回换算后物理量
    /// (0x0E 温度: raw 100 → 10.0°C)。
    #[test]
    fn hightorque_request_register_and_cached_register_roundtrip() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_request_register(
            (&mut handle) as *mut MotorHandle,
            0x0E,
            1,
        );
        assert_eq!(rc, 0);
        // 查询帧:扩展 0x8000|id,载荷 0x15 0x0E(固定 int16 count1 读 0x0E)
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8001);
        assert_eq!(&sent[0].data[..2], &[0x15, 0x0E]);
        drop(sent);

        let (mut has, mut val) = (0u8, 0f32);
        let rc = motor_handle_hightorque_get_cached_register(
            (&mut handle) as *mut MotorHandle,
            0x0E,
            &mut has,
            &mut val,
        );
        assert_eq!(rc, 0);
        assert_eq!(has, 0, "入缓前应无值");

        // 模拟被动路径(后台轮询线程)入缓
        let m = match &*handle.inner.lock().expect("inner") {
            MotorHandleInner::Hightorque(m) => Arc::clone(m),
            _ => panic!("expected hightorque motor"),
        };
        m.process_feedback_frame(rx_frame(0x0100, [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0], 4, true))
            .expect("merge");
        let rc = motor_handle_hightorque_get_cached_register(
            (&mut handle) as *mut MotorHandle,
            0x0E,
            &mut has,
            &mut val,
        );
        assert_eq!(rc, 0);
        assert_eq!(has, 1);
        assert!((val - 10.0).abs() < 1e-4, "raw 100 → 10.0°C, 实得 {val}");
    }

    /// 2026-10-22:读取返回换算后物理量。0x03 力矩, "ht"→General k=0.5:
    /// raw 2560 → 2560×0.01×0.5 = 12.8 N·m。
    #[test]
    fn hightorque_get_register_converts_torque_with_model_coeff() {
        let (bus, mut handle) = ht_handle();
        let deliver = {
            let m = match &*handle.inner.lock().expect("inner") {
                MotorHandleInner::Hightorque(m) => Arc::clone(m),
                _ => panic!("expected hightorque motor"),
            };
            std::thread::spawn(move || {
                std::thread::sleep(Duration::from_millis(10));
                m.process_feedback_frame(rx_frame(
                    0x0100,
                    [0x25, 0x03, 0x00, 0x0A, 0, 0, 0, 0],
                    4,
                    true,
                ))
                .expect("merge reply");
            })
        };
        let mut out = [0f32; 1];
        let rc = motor_handle_hightorque_get_register(
            (&mut handle) as *mut MotorHandle,
            0x03,
            1,
            500,
            out.as_mut_ptr(),
        );
        deliver.join().expect("join deliver thread");
        assert_eq!(rc, 0);
        assert!(
            (out[0] - 12.8).abs() < 1e-4,
            "raw 2560 × 0.01 × k(0.5) = 12.8, 实得 {}",
            out[0]
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(&sent[0].data[..2], &[0x15, 0x03]);
    }

    /// F5:get_state 输出 fault_code(表3,状态帧 byte1)。
    #[test]
    fn hightorque_get_state_includes_fault_code() {
        let (_bus, mut handle) = ht_handle();
        let motor = match &*handle.inner.lock().expect("lock") {
            MotorHandleInner::Hightorque(m) => Arc::clone(m),
            _ => panic!("expected hightorque motor"),
        };
        // 控制命令附带状态帧:byte0=运行模式(表2),byte1=故障码(表3)。
        // (27 01 寄存器读回复不带故障码,status/fault 均为 0。)
        motor
            .process_feedback_frame(rx_frame(
                0x0100,
                [0x05, 0x07, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                8,
                false,
            ))
            .expect("process");
        let mut state = MotorState::default();
        let rc = crate::state_ffi::motor_handle_get_state((&mut handle) as *mut MotorHandle, &mut state);
        assert_eq!(rc, 0);
        assert_eq!(state.has_value, 1);
        assert_eq!(state.fault_code, 0x07);
        assert_eq!(state.status_code, 0x05);
    }
}
