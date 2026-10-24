use super::*;

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_ensure_mode(
    motor: *mut MotorHandle,
    mode: u32,
    timeout_ms: u32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                let dm_mode = to_damiao_mode(mode).map_err(|e| e.to_string())?;
                m.ensure_control_mode(dm_mode, Duration::from_millis(timeout_ms as u64))
                    .map_err(|e| e.to_string())
            }
            MotorHandleInner::Hexfellow(m) => {
                let timeout = Duration::from_millis(timeout_ms as u64);
                match mode {
                    1 => m.ensure_mode_enabled(5, timeout).map_err(|e| e.to_string()),
                    2 => m.ensure_mode_enabled(1, timeout).map_err(|e| e.to_string()),
                    _ => Err("Hexfellow mode must be 1(MIT) / 2(POS_VEL)".to_string()),
                }
            }
            MotorHandleInner::MyActuator(_m) => {
                let _ = to_myactuator_mode(mode).map_err(|e| e.to_string())?;
                Ok(())
            }
            MotorHandleInner::Robstride(m) => {
                let rs_mode = to_robstride_mode(mode).map_err(|e| e.to_string())?;
                m.ensure_control_mode(rs_mode, Duration::from_millis(timeout_ms as u64))
                    .map_err(|e| e.to_string())
            }
            MotorHandleInner::Hightorque(m) => m
                .ensure_control_mode(mode, Duration::from_millis(timeout_ms as u64))
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_mit(
    motor: *mut MotorHandle,
    target_position: f32,
    target_velocity: f32,
    stiffness: f32,
    damping: f32,
    feedforward_torque: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => {
                let kp = stiffness.clamp(0.0, u16::MAX as f32).round() as u16;
                let kd = damping.clamp(0.0, u16::MAX as f32).round() as u16;
                m.command_mit(
                    HexfellowMitTarget {
                        position_rev: target_position / (2.0 * PI),
                        velocity_rev_s: target_velocity / (2.0 * PI),
                        torque_nm: feedforward_torque,
                        kp,
                        kd,
                        limit_permille: 1000,
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("send_mit is not supported for MyActuator; use pos-vel or vel".to_string())
            }
            MotorHandleInner::Robstride(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_mit(
                    target_position,
                    target_velocity,
                    stiffness,
                    damping,
                    feedforward_torque,
                )
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_pos_vel(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_pos_vel(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .command_pos_vel(
                    HexfellowPosVelTarget {
                        position_rev: target_position / (2.0 * PI),
                        velocity_rev_s: velocity_limit / (2.0 * PI),
                    },
                    Duration::from_millis(300),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m
                .send_position_absolute_setpoint(
                    target_position * (180.0 / PI),
                    velocity_limit * (180.0 / PI),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => {
                // Map unified POS_VEL -> RobStride native position mode:
                // run_mode=Position(1), optional limit_spd(0x7017), then loc_ref(0x7016).
                let mut rc = m
                    .set_mode(RobstrideControlMode::Position)
                    .map_err(|e| e.to_string());
                let vlim = velocity_limit.abs();
                if vlim.is_finite() && vlim > 0.0 {
                    rc = rc.and_then(|_| {
                        m.write_parameter(0x7017, ParameterValue::F32(vlim))
                            .map_err(|e| e.to_string())
                    });
                }
                rc.and_then(|_| {
                    m.write_parameter(0x7016, ParameterValue::F32(target_position))
                        .map_err(|e| e.to_string())
                })
            }
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_pos_vel(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_send_pos_vel_csp(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Robstride(m) => m
                .send_cmd_pos_vel_csp(target_position, velocity_limit)
                .map_err(|e| e.to_string()),
            _ => Err("robstride_send_pos_vel_csp requires a RobStride motor".to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_robstride_send_pos_vel_pp(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_max: f32,
    acceleration: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Robstride(m) => m
                .send_cmd_pos_vel_pp(target_position, velocity_max, acceleration)
                .map_err(|e| e.to_string()),
            _ => Err("robstride_send_pos_vel_pp requires a RobStride motor".to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_vel(motor: *mut MotorHandle, target_velocity: f32) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => {
                m.send_cmd_vel(target_velocity).map_err(|e| e.to_string())
            }
            MotorHandleInner::Hexfellow(_) => {
                Err("send_vel is not supported for Hexfellow; use MIT or POS_VEL".to_string())
            }
            MotorHandleInner::MyActuator(m) => m
                .send_velocity_setpoint(target_velocity * (180.0 / PI))
                .map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m
                .set_velocity_target(target_velocity)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => {
                m.send_cmd_vel(target_velocity).map_err(|e| e.to_string())
            }
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_send_force_pos(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
    torque_limit_ratio: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m
                .send_cmd_force_pos(target_position, velocity_limit, torque_limit_ratio)
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("send_force_pos is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("send_force_pos is not supported for MyActuator".to_string())
            }
            MotorHandleInner::Robstride(_) => {
                Err("send_force_pos is not supported for RobStride".to_string())
            }
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_force_pos(target_position, velocity_limit, torque_limit_ratio)
                .map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_store_parameters(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.store_parameters().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(_) => {
                Err("store_parameters is not supported for Hexfellow".to_string())
            }
            MotorHandleInner::MyActuator(_) => {
                Err("store_parameters is not supported for MyActuator".to_string())
            }
            MotorHandleInner::Robstride(m) => m.save_parameters().map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.store_parameters().map_err(|e| e.to_string()),
        }
    })
}

#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_request_feedback(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.request_motor_feedback().map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .query_status(Duration::from_millis(300))
                .map(|_| ())
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.request_status().map_err(|e| e.to_string()),
            // RobStride has no single-shot "request status frame" command in the private
            // protocol. Do not turn this unified feedback request into a blocking ping:
            // ping does not synthesize MotorState and can stall control loops. Use
            // robstride_ping() for connectivity checks, active report for streaming state,
            // or typed parameter reads for fresh position/velocity values.
            MotorHandleInner::Robstride(_) => Ok(()),
            // 2026-10-23:对齐 DM 语义(与网关 request_feedback op、Damiao/
            // MyActuator 分支一致)——发 17 01 查询帧(dlc 2)即返回,不等
            // 回帧,回帧由后台收帧线程入缓存。原 request_fresh_state(500ms)
            // 的阻塞语义(B1 时因 record_play.py 保留,该文件现已不存在)
            // 由调用方自行组合:发同帧并阻塞等回可用
            // motor_handle_hightorque_get_register(0x01, 3)。
            MotorHandleInner::Hightorque(m) => m.request_motor_feedback().map_err(|e| e.to_string()),
        }
    })
}

/// 受控停止：保留电机使能，按当前运行模式选择零化策略。
///
/// 与 `motor_handle_disable`（失能、取消力矩）语义不同。
///
/// 各厂商行为：
/// - Damiao: 发送零速度 MIT 帧
/// - Hexfellow: 发送零 MIT 命令
/// - MyActuator: 调用 `stop_motor()`
/// - Robstride: 模式感知受控停止（MIT 保持位置、PP 写 vel_max=0、Velocity 写 0、CSP 锁位置）
/// - Hightorque: 发送原始 stop 帧（与 disable 同实现）
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_stop(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Damiao(m) => m.send_cmd_vel(0.0).map_err(|e| e.to_string()),
            MotorHandleInner::Hexfellow(m) => m
                .command_mit(
                    HexfellowMitTarget {
                        position_rev: 0.0,
                        velocity_rev_s: 0.0,
                        torque_nm: 0.0,
                        kp: 0,
                        kd: 0,
                        limit_permille: 1000,
                    },
                    Duration::from_millis(200),
                )
                .map_err(|e| e.to_string()),
            MotorHandleInner::MyActuator(m) => m.stop_motor().map_err(|e| e.to_string()),
            MotorHandleInner::Robstride(m) => m
                .controlled_stop(Duration::from_millis(300))
                .map(|_| ())
                .map_err(|e| e.to_string()),
            MotorHandleInner::Hightorque(m) => m.disable().map_err(|e| e.to_string()),
        }
    })
}

/// HighTorque 梯形控制(v2.0.0 `07 2d`,固件 v4.6.0+):目标位置 + 速度上限 +
/// 加速度上限。vendor `send_cmd_pos_vel_acc`,仅 HighTorque 电机支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_send_pos_vel_acc(
    motor: *mut MotorHandle,
    target_position: f32,
    velocity_limit: f32,
    acceleration_limit: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_pos_vel_acc(target_position, velocity_limit, acceleration_limit)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_send_pos_vel_acc requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 普通位置模式(§1.2.1 `07 07`):目标位置 + 力矩上限(Nm)。
/// `torque_limit_nm` 传 NaN 表示力矩无限制(vendor `send_cmd_pos_classic`
/// 的 `None` 语义,线上编码为 `0x8000` 哨兵);力矩按型号自适应补偿。
/// 仅 HighTorque 电机支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_send_pos_classic(
    motor: *mut MotorHandle,
    target_position: f32,
    torque_limit_nm: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_pos_classic(
                    target_position,
                    if torque_limit_nm.is_nan() {
                        None
                    } else {
                        Some(torque_limit_nm)
                    },
                )
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_send_pos_classic requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 力矩模式(协议命令 5 `05 13`):直接下发目标力矩(Nm,物理量)。
/// 0.01 N·m/LSB 基准 + 型号力矩系数补偿;不需要预先切模式。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_send_torque(
    motor: *mut MotorHandle,
    torque_nm: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_torque(torque_nm)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_send_torque requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque DQ 电压模式(协议命令 6 `01 00 08 05 1B`):模式写 + Q 轴电压
/// 写拼一帧,voltage_v 单位 V(0.1 V/LSB)。协议只定义 Q 分量。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_send_dq_voltage(
    motor: *mut MotorHandle,
    voltage_v: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_dq_voltage(voltage_v)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_send_dq_voltage requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque DQ 电流模式(协议命令 6 `01 00 09 05 1C`):模式写 + Q 轴电流
/// 写拼一帧,current_a 单位 A(0.1 A/LSB)。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_send_dq_current(
    motor: *mut MotorHandle,
    current_a: f32,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .send_cmd_dq_current(current_a)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_send_dq_current requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 刹车(表2 模式 15 `01 00 0F`):三相短接阻尼刹车。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_brake(motor: *mut MotorHandle) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m.brake().map_err(|e| e.to_string()),
            _ => Err("hightorque_brake requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 运行模式切换(表2,`01 00 <mode>` 写模式寄存器 0x000)。
/// mode 取 0-17(0=停止并清错,8=DQ 电压,9=DQ 电流,10=位置,15=刹车),
/// 超范围返回 InvalidArgument。与 `motor_handle_ensure_mode`(仅校验)不同,
/// 此函数真发帧。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_set_run_mode(motor: *mut MotorHandle, mode: u8) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m.send_run_mode(mode).map_err(|e| e.to_string()),
            _ => Err("hightorque_set_run_mode requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 通用寄存器写(协议命令 9 派生 `05 <addr> <val16>`)。
/// `addr` 寄存器地址;`value` int16 值;`config_class` 非 0 表示配置类
/// 寄存器(走扩展 0x8000|id 帧),0 表示控制目标类(走控制通道)。
/// 模式寄存器 0x000 不适用本函数,请用 set_run_mode。仅 HighTorque 支持。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_write_register(
    motor: *mut MotorHandle,
    addr: u8,
    value: i16,
    config_class: u8,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .write_register(addr, value, config_class != 0)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_write_register requires a HighTorque motor".to_string()),
        }
    })
}

/// HighTorque 周期上报配置(协议命令 10 `05 B4 02 <t_ms16>`,扩展帧)。
/// `period_ms` 为上报周期(毫秒,最小 1);传 0 停止周期上报。
/// 开启后电机按周期主动发 `27 01` 状态帧,由被动反馈路径入状态缓存。
#[unsafe(no_mangle)]
pub extern "C" fn motor_handle_hightorque_set_periodic_report(
    motor: *mut MotorHandle,
    period_ms: u16,
) -> i32 {
    ffi_wrap_motor!(motor, |motor: &MotorHandleInner| {
        match motor {
            MotorHandleInner::Hightorque(m) => m
                .set_periodic_report(period_ms)
                .map_err(|e| e.to_string()),
            _ => Err("hightorque_set_periodic_report requires a HighTorque motor".to_string()),
        }
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::{CanBus, CanFrame};
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

    fn sent_frame(bus: &MockBus) -> CanFrame {
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        sent[0]
    }

    /// 2026-10-23 回归:request_feedback HT 分支**非阻塞**(DM 语义,
    /// 对齐网关 request_feedback op)——Mock 总线无任何回复也必须 rc=0
    /// 立即返回(旧 request_fresh_state(500ms) 实现在此会 Timeout →
    /// rc=-1)。
    /// 2026-10-24:发出**两帧** dlc 2 规范查询(`17 01` pos/vel/tq +
    /// `16 0E` 温度/错误码;真机实测固件一帧一对,拼帧被静默丢弃)。
    #[test]
    fn hightorque_request_feedback_nonblocking() {
        let (bus, mut handle) = ht_handle();
        let start = std::time::Instant::now();
        let rc = motor_handle_request_feedback((&mut handle) as *mut MotorHandle);
        assert_eq!(rc, 0, "无回复也应立即成功(发完即走)");
        assert!(
            start.elapsed() < std::time::Duration::from_millis(100),
            "不应阻塞等待回帧"
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2, "两帧:17 01 + 16 0E");
        for f in sent.iter() {
            assert_eq!(f.arbitration_id, 0x8000 | 1);
            assert!(f.is_extended);
            assert_eq!(f.dlc, 2);
        }
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01], "读/Int16/3 个,从 0x01 起");
        assert_eq!(&sent[1].data[..2], &[0x16, 0x0E], "读/Int16/2 个,从 0x0E 起");
    }

    /// F1:梯形控制走 `07 2d` 帧(control_reply 默认关 → 标准 11 位裸 id,
    /// 对齐参考固件;2026-10-09 真机证伪扩展写法后退回)。
    #[test]
    fn hightorque_send_pos_vel_acc_sends_07_2d_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_pos_vel_acc(
            (&mut handle) as *mut MotorHandle,
            1.0,
            2.0,
            3.0,
        );
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(f.arbitration_id, 1);
        assert!(!f.is_extended);
        assert_eq!(f.dlc, 8);
        assert_eq!(&f.data[..2], &[0x07, 0x2d]);
    }

    /// F2:pos_classic 的 NaN 力矩上限映射为无限制哨兵 `0x8000`(i16::MIN)。
    #[test]
    fn hightorque_send_pos_classic_nan_means_unlimited_torque() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_pos_classic(
            (&mut handle) as *mut MotorHandle,
            1.0,
            f32::NAN,
        );
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(&f.data[..2], &[0x07, 0x07]);
        // bytes 6-7 = 0x8000 无限制;bytes 4-5 固定 0
        assert_eq!(&f.data[4..8], &[0x00, 0x00, 0x00, 0x80]);
    }

    /// F2:有限力矩上限走型号系数补偿(ht 占位 → General k=0.5,
    /// raw = ((tau - d)/k)*100),不落 0x8000 哨兵。
    #[test]
    fn hightorque_send_pos_classic_finite_torque_uses_coeff() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_pos_classic(
            (&mut handle) as *mut MotorHandle,
            1.0,
            1.0,
        );
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        let tqe_raw = i16::from_le_bytes([f.data[6], f.data[7]]);
        assert_ne!(tqe_raw, i16::MIN);
    }

    /// A7:力矩模式 `05 13`,1.1 Nm × General(k=0.5) → raw 220。
    #[test]
    fn hightorque_send_torque_sends_05_13_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_torque((&mut handle) as *mut MotorHandle, 1.1);
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(f.dlc, 4);
        assert_eq!(&f.data[..4], &[0x05, 0x13, 0xDC, 0x00]);
    }

    /// A8/A9:DQ 电压/电流拼一帧(模式写+寄存器写)。
    #[test]
    fn hightorque_send_dq_voltage_sends_combined_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_dq_voltage((&mut handle) as *mut MotorHandle, 1.0);
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(f.dlc, 7);
        assert_eq!(&f.data[..7], &[0x01, 0x00, 0x08, 0x05, 0x1B, 0x0A, 0x00]);
    }

    #[test]
    fn hightorque_send_dq_current_sends_combined_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_send_dq_current((&mut handle) as *mut MotorHandle, 0.5);
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(f.dlc, 7);
        assert_eq!(&f.data[..7], &[0x01, 0x00, 0x09, 0x05, 0x1C, 0x05, 0x00]);
    }

    /// A10:刹车 = 模式 15。
    #[test]
    fn hightorque_brake_sends_mode_15_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_brake((&mut handle) as *mut MotorHandle);
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(&f.data[..3], &[0x01, 0x00, 0x0F]);
    }

    /// A11:set_run_mode 真发帧;超范围(>17)拒绝且不发帧。
    #[test]
    fn hightorque_set_run_mode_sends_frame_and_validates() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_set_run_mode((&mut handle) as *mut MotorHandle, 8);
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(&f.data[..3], &[0x01, 0x00, 0x08]);

        let rc_bad =
            motor_handle_hightorque_set_run_mode((&mut handle) as *mut MotorHandle, 18);
        assert_ne!(rc_bad, 0, "mode 18 应被拒绝");
        // 拒绝时不追加发帧
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
    }

    /// A14:write_register 两种帧类(config_class=1 恒扩展帧)。
    #[test]
    fn hightorque_write_register_frame_classes() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_write_register(
            (&mut handle) as *mut MotorHandle,
            0x22,
            500,
            0,
        );
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(&f.data[..4], &[0x05, 0x22, 0xF4, 0x01]);
    }

    /// A15:周期上报 `05 B4 02 <t_ms16>` 扩展帧。
    #[test]
    fn hightorque_set_periodic_report_sends_b4_frame() {
        let (bus, mut handle) = ht_handle();
        let rc = motor_handle_hightorque_set_periodic_report(
            (&mut handle) as *mut MotorHandle,
            10,
        );
        assert_eq!(rc, 0);
        let f = sent_frame(&bus);
        assert_eq!(f.dlc, 5);
        assert!(f.is_extended);
        assert_eq!(f.arbitration_id, 0x8001);
        assert_eq!(&f.data[..5], &[0x05, 0xB4, 0x02, 0x0A, 0x00]);
    }
}
