use crate::protocol::{
    amps_to_raw, decode_feedback, decode_read_reply, decode_register_reply, decode_version_reply,
    encode_mit_frame, encode_read, is_setting_ack, rad_to_pos_raw, radps2_to_acc_raw,
    radps_to_vel_raw, reply_motor_id, tqe_adjust_to_raw, volts_to_raw, FirmwareVersion,
    HightorqueFeedbackState, MotorModel, ReadCmd, RegisterValue, RunMode, TorqueCoeff,
    MIT_ID_PREFIX,
};
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use std::collections::HashMap;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

/// `read_registers` 阻塞等待的缓存轮询间隔(对齐 DM 的 2ms)。
const REGISTER_POLL_INTERVAL_MS: u64 = 2;

pub struct HightorqueMotor {
    pub motor_id: u16,
    feedback_id: u16,
    model: String,
    /// 力矩 raw↔Nm 补偿系数(由 `model` 解析,参考固件 `motor_tqe_adj[]`,见 P2-15)。
    torque_coeff: TorqueCoeff,
    bus: Arc<dyn CanBus>,
    state: Mutex<Option<HightorqueFeedbackState>>,
    /// 状态缓存序号:每次缓存写入(主动 `wait_status` 或被动
    /// `process_feedback_frame`)递增。`wait_status` 以调用前的序号为基准,
    /// 序号前进即视为回复已到达——否则与 CoreController 后台收帧线程
    /// (PollingMode::Background)在同一总线上抢 `bus.recv`,回帧被后台
    /// 线程吃掉时主动查询会空转到超时(数据其实已入缓存)。与 `ack_seq`
    /// 同款"序号推进即成功"的模式。
    state_seq: AtomicU64,
    /// 设置 ACK 序号:每收到一帧 ACK 由 `process_feedback_frame` 递增,
    /// `send_with_ack` 轮询该序号判定 ACK 是否到达(参照 robstride crate)。
    ack_seq: AtomicU64,
    /// 寄存器缓存(被动路径喂):addr → (最近值, 回复入缓时刻)。
    /// 回帧由 CoreController 后台收帧线程经 `process_feedback_frame` →
    /// `decode_register_reply` 解码后按地址入缓;主动读取
    /// `read_registers` 以 `ts >= request_at` 时间戳新鲜度校验认领
    /// "本次请求之后的回复", 不再从总线上 `bus.recv` 收帧——与 DM 的
    /// register_cache / RS 的 param_state 同构。
    register_cache: Mutex<HashMap<u8, (RegisterValue, Instant)>>,
    /// 控制帧帧型开关(2026-10-09 起默认关):控制类命令(pos_vel/vel/stop 等)
    /// 是否以扩展 29 位帧 `0x8000|motor_id` 发送并置 bit15 回复总开关。
    /// 默认(关):**标准 11 位裸 id 帧**——v2.0.0 参考固件主机原行为
    /// (`can_send(hfdcanx, id, ...)`,协议 §1.3 控制类行)。
    /// 开:PDF §1.4 示例的 `0x8000|id` 扩展写法。2026-09-29 曾默认开,
    /// 动机是"每个控制帧带回 §3.2 状态反馈帧";2026-10-09 candump 真机
    /// 证伪:bit15 置于非查询指令(如 05 13),电机只回一帧 DLC=0 空帧
    /// (协议 PDF §1.2 原文即如此),**没有**状态数据——开的唯一效果是
    /// 每个控制帧换一个无用空帧的总线开销,故退回标准帧。开关保留仅作
    /// 诊断/复现实验用(未接到 ABI/Python)。
    control_reply_enabled: AtomicBool,
}

impl HightorqueMotor {
    pub(crate) fn new(motor_id: u16, feedback_id: u16, model: &str, bus: Arc<dyn CanBus>) -> Self {
        let torque_coeff = MotorModel::from_hint(model)
            .unwrap_or(MotorModel::General)
            .coeff();
        Self {
            motor_id,
            feedback_id,
            model: model.to_string(),
            torque_coeff,
            bus,
            state: Mutex::new(None),
            state_seq: AtomicU64::new(0),
            ack_seq: AtomicU64::new(0),
            register_cache: Mutex::new(HashMap::new()),
            control_reply_enabled: AtomicBool::new(false),
        }
    }

    /// 当前力矩补偿系数(供上层/测试诊断)。
    pub fn torque_coeff(&self) -> TorqueCoeff {
        self.torque_coeff
    }

    /// 切换控制帧帧型(见 `control_reply_enabled` 字段注释)。
    /// 默认关:标准 11 位裸 id(参考固件主机原行为)。置 true 切到扩展
    /// `0x8000|id` + bit15——注意电机对非查询指令只回一帧 DLC=0 空帧
    /// (2026-10-09 真机证实),无状态数据,仅诊断/复现实验用。
    pub fn set_control_reply(&self, enabled: bool) {
        self.control_reply_enabled.store(enabled, Ordering::Release);
    }

    pub fn latest_state(&self) -> Option<HightorqueFeedbackState> {
        self.state.lock().ok().and_then(|g| *g)
    }

    /// 统一 enable 契约在 HT 上的映射:**语义空操作(方案 B,2026-10-08
    /// 用户决策),不发任何帧**。HT 协议无 separate 使能帧,控制命令的
    /// cmd 字节即决定行为、电机收到即执行,enable 契约的 postcondition
    /// ("后续控制命令会被执行")恒成立,故直接 Ok。
    ///
    /// 动机:core `apply_bulk_device_op` 遇错中止——enable 返回 Err 会
    /// 毒化混总线 `enable_all()`(HT 之后的电机永远使能不到);统一
    /// enable→command→disable 流程也要求 enable 可调用。清错/停机仍由
    /// `clear_error()`/`disable()` 的 `01 00 00` 显式完成,不借 enable
    /// 顺带发送,保持本方法零副作用。
    pub fn enable(&self) -> Result<()> {
        Ok(())
    }

    pub fn disable(&self) -> Result<()> {
        self.send_stop()
    }

    /// 清错 ≡ 停止(G3):表2 模式 0 的名称即"停止，清除错误",写模式寄存器
    /// 0x000=0 的 `0x01 0x00 0x00` 帧一帧同时完成两个动作,与 `disable`
    /// 同帧(协议层没有独立的 clear_error 命令)。
    pub fn clear_error(&self) -> Result<()> {
        self.send_stop()
    }

    pub fn set_zero_position(&self) -> Result<()> {
        // 设置当前位置为零点(`0x40 0x01 0x04 0x64 0x20 0x63 0x0A`,参考固件
        // `libelybot_can.c::rezero_pos`)。该指令只在 RAM 中改位置偏置,无机
        // 械动作(固件注释"此指令只是在 RAM 中修改");fire-and-forget,不
        // 等 ACK、不 sleep,随即 store_parameters 落盘——对齐参考固件,该固
        // 件 rezero_pos/conf_write 本就是 fire-and-forget(0x41 ACK 时序不可
        // 靠,参考固件也从不主动等它)。
        self.send_query(&[0x40, 0x01, 0x04, 0x64, 0x20, 0x63, 0x0A], 7)?;
        // RAM 偏置写入与随后 conf_write 之间留一短停顿,避免背靠背扩展帧。
        std::thread::sleep(Duration::from_millis(50));
        self.store_parameters()
    }

    pub fn ensure_control_mode(&self, mode: u32, _timeout: Duration) -> Result<()> {
        // 表2 运行模式:0-15 已定义,16/17 保留;超出范围拒绝。
        if mode <= 17 {
            Ok(())
        } else {
            Err(MotorError::InvalidArgument(format!(
                "HighTorque run mode must be 0-17 (表2), got {mode}"
            )))
        }
    }

    /// MIT 模式(电机固件 v4.6.0+):`0x18000 | id` 扩展帧,位打包
    /// pos(16)/vel(12)/tqe(12)/kp(12)/kd(12)。输出力矩 =
    /// (目标位置 - 当前位置)·kp + (目标速度 - 当前速度)·kd + 前馈力矩。
    ///
    /// 量程:pos ±3.2768 圈、vel ±2.0 圈/s、tqe ±10 Nm、kp ±400、kd ±100。
    /// 力矩的型号自适应(`tqe_adjust`)与 PID 的型号自适应(`pid_adjust`)
    /// 见 P2-15,当前未修正。
    pub fn send_cmd_mit(
        &self,
        target_position: f32,
        target_velocity: f32,
        stiffness: f32,
        damping: f32,
        feedforward_torque: f32,
    ) -> Result<()> {
        let data = encode_mit_frame(
            target_position,
            target_velocity,
            feedforward_torque,
            stiffness,
            damping,
        );
        self.send_raw(MIT_ID_PREFIX | u32::from(self.motor_id), &data, 8, true)
    }

    /// 梯形模式(电机固件 v4.6.0+):`0x07 0x2d`,`[pos@2-3, vel@4-5, acc@6-7]`
    /// int16 小端。以目标速度、目标加速度运动到目标位置。`0x8000` 表示无限制。
    pub fn send_cmd_pos_vel_acc(
        &self,
        target_position: f32,
        velocity_limit: f32,
        acceleration_limit: f32,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        let acc_raw = radps2_to_acc_raw(acceleration_limit);
        let mut data = [0x07, 0x2d, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&acc_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_pos_vel(&self, target_position: f32, velocity_limit: f32) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let vel_raw = radps_to_vel_raw(velocity_limit);
        self.send_cmd_pos_vel_tqe(pos_raw, vel_raw, i16::MIN)
    }

    /// 普通位置模式(§1.2.1 / 参考固件 `motor_control_pos`):`0x07 0x07`,
    /// `[pos@2-3, 0x0000@4-5, tqe@6-7]`,8 字节,小端。区别于协同模式 `0x07 0x35`
    /// (P1-2 路径),此模式只给目标位置 + 最大力矩,速度字段固定 0。
    ///
    /// `torque_limit_nm = None` 表示力矩无限制(`0x8000`)。帧走标准 11 位、
    /// ID = motor_id(P2-12/13)。力矩按型号自适应补偿(P2-15,`tqe_adjust`:
    /// `raw = ((tau - d) / k) * 100`),型号由 `model` 解析,未知型号走 General(k=0.5)。
    pub fn send_cmd_pos_classic(
        &self,
        target_position: f32,
        torque_limit_nm: Option<f32>,
    ) -> Result<()> {
        let pos_raw = rad_to_pos_raw(target_position);
        let tqe_raw = match torque_limit_nm {
            Some(nm) => tqe_adjust_to_raw(nm, self.torque_coeff),
            None => i16::MIN, // 0x8000 = 无限制
        };
        let mut data = [0x07, 0x07, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&pos_raw.to_le_bytes());
        // bytes 4-5 固定 0x0000(参考固件 `motor_control_pos` 模板,该字段在纯位置模式下不用)
        data[6..8].copy_from_slice(&tqe_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_vel(&self, target_velocity: f32) -> Result<()> {
        let vel_raw = radps_to_vel_raw(target_velocity);
        let mut data = [0x07, 0x07, 0x00, 0x80, 0x20, 0x00, 0x80, 0x00];
        data[4..6].copy_from_slice(&vel_raw.to_le_bytes());
        data[6..8].copy_from_slice(&i16::MIN.to_le_bytes());
        self.send_control(&data, 8)
    }

    pub fn send_cmd_force_pos(
        &self,
        _target_position: f32,
        _velocity_limit: f32,
        _torque_limit_ratio: f32,
    ) -> Result<()> {
        Err(MotorError::InvalidArgument(
            "send_force_pos is not supported for HighTorque ABI; use send_mit/send_pos_vel"
                .to_string(),
        ))
    }

    /// 力矩模式(协议 §1 通信命令 5):`0x05 0x13 <tqe16>`,4 字节。
    /// 物理量入参,raw = `tqe_adjust_to_raw(nm, coeff)`(0.01 N·m/LSB 基准 +
    /// 型号力矩系数补偿,与 pos_classic 的力矩限幅同一套换算)。
    /// 不需要预先切模式——该命令自身即为力矩目标下发。
    pub fn send_cmd_torque(&self, torque_nm: f32) -> Result<()> {
        let tqe_raw = tqe_adjust_to_raw(torque_nm, self.torque_coeff);
        let mut data = [0x05, 0x13, 0, 0];
        data[2..4].copy_from_slice(&tqe_raw.to_le_bytes());
        self.send_control(&data, 4)
    }

    /// 运行模式切换(协议 §1 通信命令 6 / 表2):写模式寄存器 0x000,
    /// `0x01 0x00 <mode>`,3 字节。mode 取 0-17(0=停止并清错,8=DQ 电压,
    /// 9=DQ 电流,10=位置,15=刹车)。区别于 `ensure_control_mode`(仅校验,
    /// MotorDevice trait 语义保持不变),此方法真发帧。
    pub fn send_run_mode(&self, mode: u8) -> Result<()> {
        if mode > 17 {
            return Err(MotorError::InvalidArgument(format!(
                "HighTorque run mode must be 0-17 (表2), got {mode}"
            )));
        }
        self.send_control(&[0x01, 0x00, mode], 3)
    }

    /// 刹车(表2 模式 15):`0x01 0x00 0x0F`,三相短接阻尼刹车。
    pub fn brake(&self) -> Result<()> {
        self.send_run_mode(RunMode::Brake as u8)
    }

    /// DQ 电压模式(协议 §1 通信命令 6):模式写 + 寄存器写拼一帧,
    /// `0x01 0x00 0x08 0x05 0x1B <volt16>`,7 字节,0.1 V/LSB。
    /// 协议只定义 Q 分量(0x1B),无 D 寄存器。
    pub fn send_cmd_dq_voltage(&self, voltage_v: f32) -> Result<()> {
        let volt_raw = volts_to_raw(voltage_v);
        let mut data = [0x01, 0x00, 0x08, 0x05, 0x1B, 0, 0];
        data[5..7].copy_from_slice(&volt_raw.to_le_bytes());
        self.send_control(&data, 7)
    }

    /// DQ 电流模式(协议 §1 通信命令 6):模式写 + 寄存器写拼一帧,
    /// `0x01 0x00 0x09 0x05 0x1C <cur16>`,7 字节,0.1 A/LSB。
    pub fn send_cmd_dq_current(&self, current_a: f32) -> Result<()> {
        let cur_raw = amps_to_raw(current_a);
        let mut data = [0x01, 0x00, 0x09, 0x05, 0x1C, 0, 0];
        data[5..7].copy_from_slice(&cur_raw.to_le_bytes());
        self.send_control(&data, 7)
    }

    /// 通用寄存器写(协议 §1 通信命令 9 派生):`0x05 <addr> <val16>`,4 字节。
    /// 帧型按寄存器用途二选一:控制目标类(0x13/0x1B/0x1C 等热路径寄存器)
    /// 走控制通道;配置类(存储、上报配置等)走扩展 `0x8000|id` 查询通道
    /// (参考固件 `can_send(hfdcanx, 0x8000 | id, ...)`)。
    /// 模式寄存器 0x000 不走此族,用 `send_run_mode` 的专用帧。
    pub fn write_register(&self, addr: u8, value: i16, config_class: bool) -> Result<()> {
        let mut data = [0x05, 0, 0, 0];
        data[1] = addr;
        data[2..4].copy_from_slice(&value.to_le_bytes());
        if config_class {
            self.send_query(&data, 4)
        } else {
            self.send_control(&data, 4)
        }
    }

    /// 周期上报配置(协议 §1 通信命令 10):`0x05 0xB4 0x02 <t_ms16>`,
    /// 5 字节,扩展 `0x8000|id` 帧。period_ms 最小 1;传 0 停止上报,
    /// 之后电机不再主动发 `0x27 0x01` 状态帧。
    pub fn set_periodic_report(&self, period_ms: u16) -> Result<()> {
        let mut data = [0x05, 0xB4, 0x02, 0, 0];
        data[3..5].copy_from_slice(&period_ms.to_le_bytes());
        self.send_query(&data, 5)
    }

    pub fn store_parameters(&self) -> Result<()> {
        // 落盘到 flash(`0x05 0xB3 0x02 0x00 0x00`,参考固件 `conf_write`)。
        // fire-and-forget:参考固件本就如此,0x41 ACK 时序不可靠(见
        // set_zero_position 注释);固件建议 conf_write 后重新上电使配置生效。
        self.send_query(&[0x05, 0xB3, 0x02, 0x00, 0x00], 5)
    }

    /// 发送设置类命令并等待 ACK(7 字节 `0x41 0x01 0x04 OK\r\n`)。
    /// ACK 帧由被动反馈路径(`process_feedback_frame`)递增 `ack_seq`,此处轮询该序号。
    /// 超时未收到 ACK 时返回 `MotorError::Timeout`(不重试)。
    pub fn send_with_ack(&self, payload: &[u8], dlc: u8, timeout: Duration) -> Result<()> {
        let start_seq = self.ack_seq.load(Ordering::Acquire);
        self.send_query(payload, dlc)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            if self.ack_seq.load(Ordering::Acquire) > start_seq {
                return Ok(());
            }
            std::thread::sleep(Duration::from_millis(4));
        }
        Err(MotorError::Timeout(format!(
            "HighTorque setting ACK timeout on motor id {}",
            self.motor_id
        )))
    }

    /// 查询固件版本(`0x15 0xB5 0x02`,参考固件 `send_read_motor_version`)。
    /// 回复 5 字节,签名 `data[1]==0xB5 && data[2]==0x02`,版本号按半位解包。
    pub fn request_firmware_version(&self, timeout: Duration) -> Result<FirmwareVersion> {
        self.send_query(&[0x15, 0xB5, 0x02], 3)?;
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            let left = deadline.saturating_duration_since(Instant::now());
            if let Some(frame) = self.bus.recv(left.min(Duration::from_millis(20)))? {
                if reply_motor_id(&frame) != Some(self.motor_id as u8) {
                    continue;
                }
                if let Some(v) = decode_version_reply(&frame) {
                    return Ok(v);
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "HighTorque firmware version timeout on motor id {}",
            self.motor_id
        )))
    }

    pub fn request_motor_feedback(&self, timeout: Duration) -> Result<()> {
        self.send_query(&[0x17, 0x01, 0, 0, 0, 0, 0, 0], 8)?;
        self.wait_status(timeout)
    }

    // 非阻塞状态查询的"发完即走"角色由 `request_registers` 承担:
    // 命令 8(读状态)是命令 7(通用寄存器读)的特例——
    // `request_registers(&[ReadCmd { addr: 0x01, ty: Int16, count: 3 }])`
    // 即同一 `17 01` 查询(协议文档帧型: 数据 `17 01`, dlc 2), 回复
    // `0x27 0x01` 经被动路径同时写入状态缓存与寄存器缓存。

    /// 通用寄存器读取(协议 §1.3)——DM 风格的"请求即发送":只把一组
    /// `(cmd, addr)` 读请求帧发到总线上,立即返回不等回帧。
    ///
    /// 回复由被动路径(CoreController 后台收帧线程
    /// `process_feedback_frame` → `decode_register_reply`)按地址写入
    /// `register_cache`(带入缓时刻)。之后:
    /// - 阻塞等值:`read_registers`(发送 + 轮询新鲜缓存);
    /// - 非阻塞轮询:`cached_register`(读最近值)。
    pub fn request_registers(&self, cmds: &[ReadCmd]) -> Result<()> {
        let payload = encode_read(cmds).ok_or_else(|| {
            MotorError::InvalidArgument(
                "request_registers: cmd list too long or count out of range".to_string(),
            )
        })?;
        self.send_query(&payload, payload.len() as u8)
    }

    /// 读寄存器缓存的最近值(不发帧不等待)。返回 `None` 表示从未收到过
    /// 该地址的回复。注意:无法区分"新鲜"与"陈旧"(电机掉线后停留在
    /// 最后一值);需要新鲜度保证时用 `read_registers` 或先
    /// `request_registers` 发查询、隔一拍回读并自行判断。
    pub fn cached_register(&self, addr: u8) -> Option<RegisterValue> {
        self.register_cache
            .lock()
            .ok()
            .and_then(|c| c.get(&addr).map(|(v, _)| *v))
    }

    /// 通用寄存器读取(协议 §1.3)的阻塞组合:`request_registers` 发帧 +
    /// 轮询 `register_cache`,以 `ts >= request_at` 时间戳新鲜度校验认领
    /// "本次请求之后到达的回复"。
    ///
    /// 与旧实现(直接 `bus.recv` 收帧)相比:不再与 CoreController 后台
    /// 收帧线程在同一总线队列上抢帧;时间戳即事务号,并发其它寄存器类
    /// 查询(如 `17 01` 状态查询)的回复按地址入各自的缓存键,不会误认。
    pub fn read_registers(
        &self,
        cmds: &[ReadCmd],
        timeout: Duration,
    ) -> Result<Vec<(u8, RegisterValue)>> {
        let request_at = Instant::now();
        self.request_registers(cmds)?;
        let deadline = request_at + timeout;
        loop {
            if let Some(vals) = self.fresh_registers(cmds, request_at) {
                return Ok(vals);
            }
            if Instant::now() >= deadline {
                let addrs = cmds
                    .iter()
                    .map(|c| format!("0x{:02X}", c.addr))
                    .collect::<Vec<_>>()
                    .join(",");
                return Err(MotorError::Timeout(format!(
                    "HighTorque register read timeout on motor id {} (addr {})",
                    self.motor_id, addrs
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    /// 从缓存取出覆盖全部请求地址且入缓时刻 >= `request_at` 的值;
    /// 任一地址缺失或陈旧则返回 `None`(继续等)。
    fn fresh_registers(
        &self,
        cmds: &[ReadCmd],
        request_at: Instant,
    ) -> Option<Vec<(u8, RegisterValue)>> {
        let cache = self.register_cache.lock().ok()?;
        let mut out = Vec::new();
        for c in cmds {
            for i in 0..u16::from(c.count) {
                let addr = u8::try_from(u16::from(c.addr) + i).ok()?;
                match cache.get(&addr) {
                    Some((v, ts)) if *ts >= request_at => out.push((addr, *v)),
                    _ => return None,
                }
            }
        }
        Some(out)
    }

    fn send_stop(&self) -> Result<()> {
        self.send_control(&[0x01, 0x00, 0x00], 3)
    }

    fn send_cmd_pos_vel_tqe(&self, pos_raw: i16, vel_raw: i16, tqe_raw: i16) -> Result<()> {
        let mut data = [0x07, 0x35, 0, 0, 0, 0, 0, 0];
        data[2..4].copy_from_slice(&vel_raw.to_le_bytes());
        data[4..6].copy_from_slice(&tqe_raw.to_le_bytes());
        data[6..8].copy_from_slice(&pos_raw.to_le_bytes());
        self.send_control(&data, 8)
    }

    /// 控制类命令(vel/pos_vel/pos_vel_acc/pos_classic/stop/brake)。
    ///
    /// 默认**标准 11 位帧**、ID = 裸 `motor_id`——对齐 v2.0.0 参考固件
    /// `can_send(hfdcanx, id, ...)`(协议 §1.3 控制类行)。2026-09-29 曾默认
    /// 扩展 `0x8000|id` + bit15,以期每个控制帧带回 §3.2 状态反馈帧;
    /// 2026-10-09 candump 真机证伪:非查询指令下电机只回一帧 DLC=0 空帧
    /// (PDF §1.2 原文行为),无状态数据,故退回标准裸 id 帧。
    /// `set_control_reply(true)` 可切回扩展写法(仅诊断/复现用)。
    fn send_control(&self, payload: &[u8], dlc: u8) -> Result<()> {
        if self.control_reply_enabled.load(Ordering::Acquire) {
            self.send_raw(u32::from(0x8000u16 | self.motor_id), payload, dlc, true)
        } else {
            self.send_raw(u32::from(self.motor_id), payload, dlc, false)
        }
    }

    /// 查询/配置类命令(read/rezero/store/conf_write/version):**扩展 29 位帧**,
    /// ID = `0x8000 | motor_id`,置回复总开关 bit15(参考固件这些路径均
    /// `can_send(hfdcanx, 0x8000 | id, ...)`,见 P2-12/P2-13)。
    fn send_query(&self, payload: &[u8], dlc: u8) -> Result<()> {
        self.send_raw(u32::from(0x8000u16 | self.motor_id), payload, dlc, true)
    }

    fn send_raw(
        &self,
        arbitration_id: u32,
        payload: &[u8],
        dlc: u8,
        is_extended: bool,
    ) -> Result<()> {
        let mut data = [0u8; 8];
        data[..payload.len().min(8)].copy_from_slice(&payload[..payload.len().min(8)]);
        self.bus.send(CanFrame {
            arbitration_id,
            data,
            dlc: dlc.min(8),
            is_extended,
            is_rx: false,
        })
    }

    fn wait_status(&self, timeout: Duration) -> Result<()> {
        let start_seq = self.state_seq.load(Ordering::Acquire);
        let deadline = Instant::now() + timeout;
        while Instant::now() < deadline {
            // 后台收帧线程可能已把回帧解码进缓存:序号前进即成功,
            // 不再依赖本线程自己从 bus 收到该帧。
            if self.state_seq.load(Ordering::Acquire) > start_seq {
                return Ok(());
            }
            let left = deadline.saturating_duration_since(Instant::now());
            if let Some(frame) = self.bus.recv(left.min(Duration::from_millis(20)))? {
                if let Some(state) = decode_read_reply(frame, self.torque_coeff) {
                    if state.can_id as u16 == self.motor_id {
                        if let Ok(mut g) = self.state.lock() {
                            *g = Some(state);
                        }
                        self.state_seq.fetch_add(1, Ordering::Release);
                        return Ok(());
                    }
                }
            }
        }
        Err(MotorError::Timeout(format!(
            "HighTorque read timeout on motor id {}",
            self.motor_id
        )))
    }
}

impl MotorDevice for HightorqueMotor {
    fn vendor(&self) -> &'static str {
        "hightorque"
    }

    fn model(&self) -> &str {
        &self.model
    }

    fn motor_id(&self) -> u16 {
        self.motor_id
    }

    fn feedback_id(&self) -> u16 {
        self.feedback_id
    }

    fn enable(&self) -> Result<()> {
        HightorqueMotor::enable(self)
    }

    fn disable(&self) -> Result<()> {
        HightorqueMotor::disable(self)
    }

    fn accepts_frame(&self, frame: &CanFrame) -> bool {
        if !frame.is_rx {
            return false;
        }
        if reply_motor_id(frame) != Some(self.motor_id as u8) {
            return false;
        }
        // 8 字节:寄存器读回复(0x27...)或控制命令附带的状态反馈(byte0=mode)。
        // 7 字节:设置 ACK(0x41...)。5 字节:固件版本回复(0xB5 0x02)。
        // 其余长度:可解析的寄存器读回复(int16 count1=4 字节、float=6 字节
        // 等)——入寄存器缓存;不匹配则不归被动反馈,帧被丢弃。
        match frame.dlc {
            8 => true,
            7 => is_setting_ack(frame),
            5 => decode_version_reply(frame).is_some(),
            _ => decode_register_reply(&frame.data[..frame.dlc as usize]).is_some(),
        }
    }

    fn process_feedback_frame(&self, frame: CanFrame) -> Result<()> {
        if reply_motor_id(&frame) != Some(self.motor_id as u8) {
            return Ok(());
        }
        if is_setting_ack(&frame) {
            self.ack_seq.fetch_add(1, Ordering::Release);
            return Ok(());
        }
        // 寄存器读回复按地址入缓存(带时刻):0x27 0x01 状态回复的
        // 0x01-0x03 也入(状态查询本身就是寄存器读), 短帧(4/6 字节)
        // 只走这里。时间戳供 read_registers 的新鲜度校验认领。
        if let Some(vals) = decode_register_reply(&frame.data[..frame.dlc as usize]) {
            if let Ok(mut cache) = self.register_cache.lock() {
                let now = Instant::now();
                for (addr, v) in vals {
                    cache.insert(addr, (v, now));
                }
            }
        }
        if let Some(state) = decode_feedback(frame, self.torque_coeff) {
            if state.can_id as u16 == self.motor_id {
                self.state
                    .lock()
                    .map_err(|_| MotorError::Io("state lock poisoned".to_string()))?
                    .replace(state);
                // 推进缓存序号,让正在 wait_status 主动等待的一方也能
                // 观察到"回复已到达"。
                self.state_seq.fetch_add(1, Ordering::Release);
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use motor_core::bus::CanFrame;
    use motor_core::device::MotorDevice;
    use motor_core::test_support::MockBus;
    use std::sync::Arc;
    use std::time::Duration;

    fn make_motor(id: u16) -> (Arc<MockBus>, Arc<HightorqueMotor>) {
        let bus: Arc<MockBus> = Arc::new(MockBus::new());
        let motor = Arc::new(HightorqueMotor::new(id, 0, "ht-test", bus.clone()));
        (bus, motor)
    }

    fn ack_frame(id: u16) -> CanFrame {
        CanFrame {
            arbitration_id: u32::from(id) << 8,
            data: [0x41, 0x01, 0x04, 0x4F, 0x4B, 0x0D, 0x0A, 0],
            dlc: 7,
            is_extended: true,
            is_rx: true,
        }
    }

    fn version_frame(id: u16, b3: u8, b4: u8) -> CanFrame {
        CanFrame {
            arbitration_id: u32::from(id) << 8,
            data: [0x15, 0xB5, 0x02, b3, b4, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        }
    }

    #[test]
    fn process_feedback_frame_bumps_ack_seq_on_setting_ack() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        motor.process_feedback_frame(ack_frame(1)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before + 1);
    }

    #[test]
    fn process_feedback_frame_ignores_ack_for_other_motor() {
        let (_bus, motor) = make_motor(1);
        let before = motor.ack_seq.load(Ordering::Acquire);
        // id=2 的 ACK 不应被 id=1 的电机消费
        motor.process_feedback_frame(ack_frame(2)).expect("process");
        assert_eq!(motor.ack_seq.load(Ordering::Acquire), before);
    }

    #[test]
    fn send_with_ack_returns_ok_when_ack_arrives() {
        let (bus, motor) = make_motor(1);
        let m = motor.clone();
        // 模拟被动反馈路径在命令发出后递补 ACK 序号
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(15));
            m.process_feedback_frame(ack_frame(1)).expect("process");
        });
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(500));
        handle.join().expect("thread join");
        assert!(r.is_ok(), "send_with_ack should succeed on ACK: {:?}", r);
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert_eq!(&sent[0].data[..5], &[0x05, 0xB3, 0x02, 0, 0]);
    }

    #[test]
    fn send_with_ack_times_out_without_ack() {
        let (_bus, motor) = make_motor(1);
        let r = motor.send_with_ack(&[0x05, 0xB3, 0x02, 0, 0], 5, Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "expected timeout, got {r:?}"
        );
    }

    /// 命令 8(读状态)是命令 7(通用寄存器读)的特例: 寄存器读原语发
    /// 出的即协议文档的规范 `17 01` 查询帧(扩展 29 位 0x8000|id, dlc 2),
    /// 无任何回帧时也立即 Ok——request_motor_feedback_async 删除后,
    /// "发完即走"的非阻塞状态查询由这条路径承担。
    #[test]
    fn state_read_via_register_request_sends_canonical_frame() {
        let (bus, motor) = make_motor(1);
        // 不 push 任何回帧: 阻塞版 read_registers 在此会 Timeout,
        // request(只发送)应立即 Ok
        motor
            .request_registers(&[ReadCmd {
                addr: 0x01,
                ty: crate::protocol::RegisterType::Int16,
                count: 3,
            }])
            .expect("state read request send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1);
        assert!(sent[0].is_extended, "查询帧应为扩展 29 位");
        assert_eq!(sent[0].dlc, 2);
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
    }

    /// 发完即走 + 后台被动反馈路径的完整闭环: 寄存器读请求(即 17 01
    /// 状态查询)发出后, 回帧经 process_feedback_frame(后台轮询线程的
    /// 入口)同时写入状态缓存与寄存器缓存——latest_state 与
    /// cached_register 读到的是同一帧。
    #[test]
    fn state_read_request_reply_feeds_both_caches_via_passive_path() {
        let (_bus, motor) = make_motor(1);
        assert!(motor.latest_state().is_none());
        motor
            .request_registers(&[ReadCmd {
                addr: 0x01,
                ty: crate::protocol::RegisterType::Int16,
                count: 3,
            }])
            .expect("state read request send");
        // 模拟 CoreController 后台线程收到 27 01 回复(id<<8 仲裁 ID)
        let reply = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(reply).expect("process");
        let st = motor.latest_state().expect("state should be cached");
        assert_eq!(st.can_id, 1);
        // pos_raw=0x1234=4660 → 4660*0.0001*2π
        assert!((st.pos - 0.4660 * std::f32::consts::TAU).abs() < 1e-4);
        // 同一帧按地址入寄存器缓存: 0x01/0x02/0x03 各一个 int16
        assert!(matches!(motor.cached_register(0x01), Some(RegisterValue::I16(0x1234))));
        assert!(matches!(motor.cached_register(0x02), Some(RegisterValue::I16(0x5678))));
        assert!(matches!(motor.cached_register(0x03), Some(RegisterValue::I16(0x0A00))));
    }

    /// 收敛(2026-09-30)回归:CoreController 后台收帧线程把 27 01 回帧
    /// 直接送进 process_feedback_frame(总线帧被它消费,主动查询方的
    /// bus.recv 拿不到)时,阻塞式 request_motor_feedback 也必须成功返回。
    #[test]
    fn request_motor_feedback_ok_when_worker_consumes_reply() {
        let (_bus, motor) = make_motor(1);
        let motor2 = Arc::clone(&motor);
        let worker = std::thread::spawn(move || {
            // 模拟后台线程在主动查询等待期间收到回帧(回帧不进 MockBus
            // 的 rx 队列,主动方的 bus.recv 永远收不到它)。
            std::thread::sleep(Duration::from_millis(30));
            let reply = CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            };
            motor2.process_feedback_frame(reply).expect("process");
        });
        motor
            .request_motor_feedback(Duration::from_millis(500))
            .expect("worker-consumed reply should still satisfy wait_status");
        worker.join().expect("worker join");
        assert!(motor.latest_state().is_some());
    }

    #[test]
    fn request_firmware_version_decodes_reply() {
        let (bus, motor) = make_motor(1);
        // data[3]=0x12, data[4]=0x34 → major=3, minor=5, patch=2
        bus.push_rx(version_frame(1, 0x12, 0x34));
        let v = motor
            .request_firmware_version(Duration::from_millis(200))
            .expect("version");
        assert_eq!(v.as_tuple(), (3, 5, 2));
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1);
        assert_eq!(&sent[0].data[..3], &[0x15, 0xB5, 0x02]);
    }

    #[test]
    fn request_firmware_version_times_out_without_reply() {
        let (_bus, motor) = make_motor(1);
        let r = motor.request_firmware_version(Duration::from_millis(20));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "expected timeout, got {r:?}"
        );
    }

    #[test]
    fn accepts_frame_admits_status_register_ack_and_version() {
        let (_bus, motor) = make_motor(1);
        // 8 字节状态帧
        let status = CanFrame {
            arbitration_id: 0x0100,
            data: [0x0A, 0x00, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
            dlc: 8,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&status));
        // 7 字节 ACK
        assert!(motor.accepts_frame(&ack_frame(1)));
        // 5 字节版本回复
        assert!(motor.accepts_frame(&version_frame(1, 0x12, 0x34)));
        // 其它电机的帧不接受
        assert!(!motor.accepts_frame(&ack_frame(2)));
        // 3 字节短帧不接受
        let short = CanFrame {
            arbitration_id: 0x0100,
            data: [0x01, 0x00, 0x00, 0, 0, 0, 0, 0],
            dlc: 3,
            is_extended: true,
            is_rx: true,
        };
        assert!(!motor.accepts_frame(&short));
    }

    #[test]
    fn ensure_control_mode_accepts_0_through_17() {
        let (_bus, motor) = make_motor(1);
        for mode in 0..=17 {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_ok(),
                "mode {mode} should be accepted"
            );
        }
    }

    #[test]
    fn ensure_control_mode_rejects_above_17() {
        let (_bus, motor) = make_motor(1);
        for mode in [18, 19, 100, u32::MAX] {
            assert!(
                motor.ensure_control_mode(mode, Duration::ZERO).is_err(),
                "mode {mode} should be rejected"
            );
        }
    }

    /// 2026-10-09:控制命令默认走**标准 11 位裸 id 帧**(v2.0.0 参考固件
    /// 原行为)。2026-09-29 的扩展 `0x8000|id` + bit15 已被 candump 真机
    /// 证伪退回——bit15 于非查询指令只换来一帧 DLC=0 空帧,无状态数据。
    #[test]
    fn control_commands_default_to_standard_bare_id_frames() {
        let (bus, motor) = make_motor(1);
        // 速度命令(0x07 0x07)
        motor.send_cmd_vel(1.0).expect("vel");
        // 协同位置-速度(0x07 0x35)
        motor.send_cmd_pos_vel(0.5, 1.0).expect("pos_vel");
        // 梯形(0x07 0x2d)
        motor
            .send_cmd_pos_vel_acc(0.5, 1.0, 0.5)
            .expect("pos_vel_acc");
        // 停止(0x01 0x00 0x00)
        motor.send_stop().expect("stop");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 4);
        for f in sent.iter() {
            assert!(!f.is_extended, "控制帧默认应为标准 11 位");
            assert_eq!(f.arbitration_id, 1, "控制帧 ID 应为裸 motor_id");
            assert!(!f.is_rx);
        }
    }

    /// set_control_reply(true):切到 PDF §1.4 示例写法——扩展 29 位
    /// `0x8000|id` + bit15。真机行为:电机对非查询指令回一帧 DLC=0
    /// 空帧(§1.2),无状态数据,仅诊断/复现用。
    #[test]
    fn control_commands_reply_mode_uses_extended_frames() {
        let (bus, motor) = make_motor(1);
        motor.set_control_reply(true);
        motor.send_cmd_pos_vel(0.5, 1.0).expect("pos_vel");
        motor.send_stop().expect("stop");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        for f in sent.iter() {
            assert!(f.is_extended, "reply 控制帧应为扩展 29 位");
            assert_eq!(f.arbitration_id, 0x8001, "reply 控制帧 ID 应为 0x8000|motor_id");
            assert!(!f.is_rx);
        }
    }

    /// G3:clear_error 与 disable 同帧(`0x01 0x00 0x00`,表2 模式 0
    /// "停止，清除错误"),协议层没有独立的清错命令。
    #[test]
    fn clear_error_sends_stop_frame() {
        let (bus, motor) = make_motor(1);
        motor.clear_error().expect("clear_error");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert!(!f.is_extended, "清错帧默认应为标准 11 位(参考固件原行为)");
        assert_eq!(f.arbitration_id, 1, "清错帧 ID 应为裸 motor_id");
        assert_eq!(f.dlc, 3);
        assert_eq!(&f.data[..3], &[0x01, 0x00, 0x00]);
    }

    /// P2-12/13:查询/配置命令走扩展 29 位帧、ID=0x8000|motor_id,置回复位。
    #[test]
    fn query_commands_use_extended_frame_with_reply_bit() {
        let (bus, motor) = make_motor(1);
        // 寄存器读取(0x17 0x01 派生的 read 命令)
        motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x01,
                    ty: crate::protocol::RegisterType::Int16,
                    count: 3,
                }],
                Duration::from_millis(10),
            )
            .ok();
        // 固件版本查询(0x15 0xB5 0x02)
        motor
            .request_firmware_version(Duration::from_millis(10))
            .ok();
        let sent = bus.sent.lock().expect("sent");
        assert!(sent.len() >= 2);
        for f in sent.iter() {
            assert!(f.is_extended, "查询帧应为扩展 29 位");
            assert_eq!(f.arbitration_id, 0x8001, "查询帧 ID 应为 0x8000|motor_id");
            assert!(!f.is_rx);
        }
    }

    /// 回归:read_registers 走"发送 + 轮询新鲜缓存"路径,回复由被动
    /// 路径(后台线程入口 process_feedback_frame)入缓存。等值期间
    /// 到达其它事务的寄存器回复(并发 `17 01` 状态查询的 `0x27 0x01`,
    /// 地址 0x01-0x03)按地址入各自的缓存键,不满足 0x0E 请求;等到
    /// 0x0E 的回复入缓后才返回其值。
    #[test]
    fn read_registers_waits_for_fresh_reply_at_requested_address() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        // 模拟后台轮询线程: 请求发出 10ms 后先送来别人的状态回复,
        // 再送来本事务请求的 0x0E 温度回复
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge state reply");
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge temp reply");
        });
        let vals = motor
            .read_registers(
                &[ReadCmd {
                    addr: 0x0E,
                    ty: crate::protocol::RegisterType::Int16,
                    count: 1,
                }],
                Duration::from_millis(500),
            )
            .expect("应等到 0x0E 的新鲜回复");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0E, "返回的应是地址 0x0E 的值");
        assert!(matches!(
            vals[0].1,
            crate::protocol::RegisterValue::I16(0x64)
        ));
    }

    /// 请求之前入缓存的旧值不满足新鲜度(ts < request_at),不能被
    /// 本次 read_registers 认领——时间戳就是事务号。
    #[test]
    fn read_registers_rejects_stale_cached_value() {
        let (_bus, motor) = make_motor(1);
        // 先入缓一个 0x0E 旧值, 再发起读: 旧值早于 request_at
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge old reply");
        let r = motor.read_registers(
            &[ReadCmd {
                addr: 0x0E,
                ty: crate::protocol::RegisterType::Int16,
                count: 1,
            }],
            Duration::from_millis(30),
        );
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "旧缓存值不应被认领, 应超时: {r:?}"
        );
    }

    /// 非阻塞路径:request_registers 发完即走(帧格式/扩展 id 正确),
    /// cached_register 在回复入缓前为 None、入缓后可读;accepts_frame
    /// 认领 4 字节的寄存器回复(此前该长度被拒收丢弃)。
    #[test]
    fn request_registers_then_cached_register_via_passive_path() {
        let (bus, motor) = make_motor(1);
        motor
            .request_registers(&[ReadCmd {
                addr: 0x0E,
                ty: crate::protocol::RegisterType::Int16,
                count: 1,
            }])
            .expect("send query");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 1, "request_registers 只发一帧不等回帧");
        assert_eq!(sent[0].arbitration_id, 0x8000 | 1, "查询帧应为扩展 29 位");
        assert_eq!(sent[0].dlc, 2, "int16 count1 读请求载荷 2 字节");
        assert_eq!(&sent[0].data[..2], &[0x15, 0x0E], "cmd=0x15(int16 count1) addr=0x0E");
        drop(sent);

        let reply = CanFrame {
            arbitration_id: 0x0100,
            data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
            dlc: 4,
            is_extended: true,
            is_rx: true,
        };
        assert!(
            MotorDevice::accepts_frame(&*motor, &reply),
            "4 字节寄存器回复应被被动路径认领"
        );
        assert!(motor.cached_register(0x0E).is_none(), "入缓前应无值");
        motor.process_feedback_frame(reply).expect("merge");
        assert!(matches!(
            motor.cached_register(0x0E),
            Some(crate::protocol::RegisterValue::I16(0x64))
        ));
    }

    /// 0x27 0x01 状态回复同时入状态缓存与寄存器缓存(状态查询本身就是
    /// 0x01-0x03 的寄存器读),latest_state 与 cached_register 均可读。
    #[test]
    fn state_reply_feeds_both_state_and_register_cache() {
        let (_bus, motor) = make_motor(1);
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x09, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge");
        let st = motor.latest_state().expect("state cache");
        assert!((st.pos - (0x1234 as f32 * 0.0001 * std::f32::consts::TAU)).abs() < 1e-6);
        assert!(matches!(
            motor.cached_register(0x01),
            Some(crate::protocol::RegisterValue::I16(0x1234))
        ));
        assert!(matches!(
            motor.cached_register(0x03),
            Some(crate::protocol::RegisterValue::I16(0x0000))
        ));
    }

    /// P2-14:普通位置模式 `0x07 0x07` 字节布局,力矩 None → 0x8000(无限制),
    /// 标准 11 位帧。对照参考固件 `motor_control_pos` 模板 `[0x07,0x07,pos@2-3,
    /// 0x00,0x00,tqe@6-7]`。
    #[test]
    fn send_cmd_pos_classic_byte_layout() {
        let (bus, motor) = make_motor(1);
        // 目标位置 0.5 圈 → raw = 0.5 * 10000 = 5000 = 0x1388
        motor
            .send_cmd_pos_classic(0.5 * std::f32::consts::TAU, None)
            .expect("pos_classic");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert!(!f.is_extended, "普通位置帧默认应为标准 11 位");
        assert_eq!(f.arbitration_id, 1);
        assert_eq!(&f.data[0..2], &[0x07, 0x07]);
        // pos = 5000 LE
        assert_eq!(i16::from_le_bytes([f.data[2], f.data[3]]), 5000);
        // bytes 4-5 固定 0x0000
        assert_eq!(&f.data[4..6], &[0x00, 0x00]);
        // tqe = 0x8000(无限制)
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), i16::MIN);
    }

    /// P2-14 + P2-15:给定力矩时,classic pos 的 tqe 字段 = tqe_adjust_to_raw(nm, coeff)。
    /// 测试电机 model="ht-test" → General(k=0.5),故 1.0 Nm → ((1.0/0.5)*100) = 200。
    #[test]
    fn send_cmd_pos_classic_with_torque_limit() {
        let (bus, motor) = make_motor(1);
        assert_eq!(motor.torque_coeff().k, 0.5, "测试电机应为 General k=0.5");
        // 1.0 Nm → raw = 200 = 0x00C8
        motor
            .send_cmd_pos_classic(0.0, Some(1.0))
            .expect("pos_classic");
        let f = &bus.sent.lock().expect("sent")[0];
        assert_eq!(i16::from_le_bytes([f.data[6], f.data[7]]), 200);
    }

    /// 力矩模式(协议命令 5):`05 13 <tqe16>` 4 字节;1.1 Nm × General(k=0.5)
    /// → ((1.1/0.5)*100) = 220 = 0x00DC。
    #[test]
    fn send_cmd_torque_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_torque(1.1).expect("torque");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 4);
        assert_eq!(&f.data[..4], &[0x05, 0x13, 0xDC, 0x00]);
        assert!(!f.is_extended, "力矩帧默认走控制通道(标准裸 id)");
        assert_eq!(f.arbitration_id, 1);
    }

    /// enable 为语义空操作(方案 B,2026-10-08):返回 Ok 且不向总线发
    /// 任何帧。协议无使能帧、命令即执行,postcondition 恒成立;清错/
    /// 停机由 clear_error()/disable() 显式完成,enable 保持零副作用。
    /// 此契约同时保证混总线 enable_all 不再被 Err 中止毒化。
    #[test]
    fn enable_is_frameless_noop() {
        let (bus, motor) = make_motor(1);
        motor.enable().expect("enable is a no-op success");
        assert!(
            bus.sent.lock().expect("sent").is_empty(),
            "enable must not send any frame"
        );
    }

    /// 运行模式切换(协议命令 6):`01 00 <mode>` 3 字节,与 send_stop 同款。
    #[test]
    fn send_run_mode_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_run_mode(5).expect("run mode");
        motor.send_run_mode(17).expect("run mode 17 保留值仍合法");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        assert_eq!(&sent[0].data[..3], &[0x01, 0x00, 0x05]);
        assert_eq!(&sent[1].data[..3], &[0x01, 0x00, 0x11]);
        assert_eq!(sent[0].arbitration_id, 1);
        assert!(!sent[0].is_extended, "run_mode 走控制通道(标准裸 id)");
    }

    #[test]
    fn send_run_mode_rejects_above_17() {
        let (bus, motor) = make_motor(1);
        for mode in [18, 100, 255] {
            assert!(
                motor.send_run_mode(mode).is_err(),
                "mode {mode} should be rejected"
            );
        }
        // 拒绝时不发帧
        assert!(bus.sent.lock().expect("sent").is_empty());
    }

    /// 刹车 = 模式 15(表2)。
    #[test]
    fn brake_sends_mode_15_frame() {
        let (bus, motor) = make_motor(1);
        motor.brake().expect("brake");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        assert_eq!(&sent[0].data[..3], &[0x01, 0x00, 0x0F]);
        assert_eq!(sent[0].dlc, 3);
    }

    /// DQ 电压(协议命令 6):模式写 + 寄存器写拼一帧 7 字节,
    /// `01 00 08 05 1B <volt16>`;1.0 V → 0.1 V/LSB → raw 10。
    #[test]
    fn send_cmd_dq_voltage_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_dq_voltage(1.0).expect("dq voltage");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 7);
        assert_eq!(
            &f.data[..7],
            &[0x01, 0x00, 0x08, 0x05, 0x1B, 0x0A, 0x00]
        );
    }

    /// DQ 电流(协议命令 6):`01 00 09 05 1C <cur16>`;0.5 A → raw 5。
    #[test]
    fn send_cmd_dq_current_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.send_cmd_dq_current(0.5).expect("dq current");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
        let f = &sent[0];
        assert_eq!(f.dlc, 7);
        assert_eq!(&f.data[..7], &[0x01, 0x00, 0x09, 0x05, 0x1C, 0x05, 0x00]);
    }

    /// 通用寄存器写(协议命令 9 派生):`05 <addr> <val16>` 4 字节。
    /// 控制类走控制通道(默认标准 11 位裸 id)、配置类恒走扩展 `0x8000|id`
    /// 查询通道——config 类不随 control_reply 开关切换帧型。
    #[test]
    fn write_register_both_frame_classes() {
        let (bus, motor) = make_motor(1);
        // 控制类(如前馈扭矩寄存器 0x22):默认标准裸 id
        motor.write_register(0x22, 500, false).expect("control write");
        // 配置类:仍走 send_query 扩展帧,即使 control_reply 开
        motor.set_control_reply(true);
        motor.write_register(0x30, 100, true).expect("config write");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        assert_eq!(&sent[0].data[..4], &[0x05, 0x22, 0xF4, 0x01]);
        assert!(!sent[0].is_extended);
        assert_eq!(sent[0].arbitration_id, 1);
        assert_eq!(&sent[1].data[..4], &[0x05, 0x30, 0x64, 0x00]);
        assert!(sent[1].is_extended, "config 类写必须恒为扩展帧");
        assert_eq!(sent[1].arbitration_id, 0x8000 | 1);
    }

    /// 周期上报(协议命令 10):`05 B4 02 <t_ms16>` 5 字节扩展帧;0=停止。
    #[test]
    fn set_periodic_report_byte_layout() {
        let (bus, motor) = make_motor(1);
        motor.set_periodic_report(10).expect("periodic on");
        motor.set_periodic_report(0).expect("periodic off");
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 2);
        for f in sent.iter() {
            assert_eq!(f.dlc, 5);
            assert!(f.is_extended, "周期上报配置帧应为扩展 0x8000|id");
            assert_eq!(f.arbitration_id, 0x8001);
        }
        assert_eq!(&sent[0].data[..5], &[0x05, 0xB4, 0x02, 0x0A, 0x00]);
        assert_eq!(&sent[1].data[..5], &[0x05, 0xB4, 0x02, 0x00, 0x00]);
    }
}
