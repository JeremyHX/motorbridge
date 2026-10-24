use crate::protocol::{
    amps_to_raw, decode_feedback, decode_register_reply, encode_mit_frame,
    encode_read, is_setting_ack, rad_to_pos_raw, radps2_to_acc_raw, radps_to_vel_raw,
    reply_motor_id, tqe_adjust_to_raw, volts_to_raw, HightorqueFeedbackState, MotorModel, ReadCmd,
    RegisterType, RegisterValue, RunMode, TorqueCoeff, MIT_ID_PREFIX,
};
use crate::registers;
use motor_core::bus::{CanBus, CanFrame};
use motor_core::device::MotorDevice;
use motor_core::error::{MotorError, Result};
use motor_core::feedback::{FeedbackCache, FeedbackSnapshot};
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
    /// 状态缓存(motor_core 跨厂商 FeedbackCache):后台收帧线程
    /// 经 `process_feedback_frame` 被动写入,自带写入序号与入缓时刻
    /// (`sequence`/`age`),`request_fresh_state` 以发帧前时刻为基准
    /// 判定回复到达——不再与 CoreController 后台收帧线程在同一总线上
    /// 抢 `bus.recv`。替代原 `Mutex<Option<…>>` + 手工 `state_seq` 方案
    /// (序号推进即成功的模式由 FeedbackCache 内建)。
    state: FeedbackCache<HightorqueFeedbackState>,
    /// 设置 ACK 序号:每收到一帧 ACK 由 `process_feedback_frame` 递增,
    /// `send_with_ack` 轮询该序号判定 ACK 是否到达(参照 robstride crate)。
    ack_seq: AtomicU64,
    /// 寄存器缓存(被动路径喂):`(addr, ty)` → (最近值, 回复入缓时刻),
    /// 键 `(addr, ty)`(2026-10-21 A3 修复:同地址不同类型各归各条目,
    /// 显式 Int8 读不会被被动 27 01 流的 Int16 值污染)。回帧由
    /// CoreController 后台收帧线程经 `process_feedback_frame` →
    /// `decode_register_reply` 解码后入缓;主动读取 `read_registers`
    /// 以 `ts >= request_at` 时间戳新鲜度校验认领"本次请求之后的回复",
    /// 不再从总线上 `bus.recv` 收帧——与 DM 的寄存器读回复缓存、RS 的
    /// param_state 设计同构(被动喂缓存 + 主动侧新鲜度认领)。
    register_cache: Mutex<HashMap<(u8, RegisterType), (RegisterValue, Instant)>>,
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
            state: FeedbackCache::new(),
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
        self.latest_state_snapshot().map(|snapshot| snapshot.state)
    }

    pub fn latest_state_snapshot(&self) -> Option<FeedbackSnapshot<HightorqueFeedbackState>> {
        self.state.latest().ok().flatten()
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
        // 2026-10-22 追补决策:缺号/表外地址拒绝访问(写比读更危险,
        // 语义未知的寄存器不写)。0xB3/0xB4 是命令地址不算寄存器,经
        // store_parameters/set_periodic_report 专用方法下发。
        registers::ensure_known_span(addr, 1).map_err(MotorError::InvalidArgument)?;
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

    /// 非阻塞状态查询(DM 对齐命名;2026-10-21 B1 修复时此名按 DM 语义
    /// 归还"发完即走",阻塞式旧方法连同其 8 字节零填充帧一并删除)。
    ///
    /// 2026-10-24 设计(真机实测驱动):**连发两帧** 2 字节规范读——
    /// - `17 01`:0x01-0x03 位置/速度/力矩(命令 8 是命令 7 的特例);
    /// - `16 0E`:0x0E-0x0F 温度+错误码(两地址连续,一帧一个回帧;
    /// 实测单 socket 连发两帧总线流水线占用约 1ms,回帧延迟 ~0.5ms)。
    /// 模式(0x00)不在此帧组——它与 0x01 之间隔着缺号地址,且独立读
    /// `15 00` 也会经同一接收桥接补丁 status_code。
    ///
    /// 回复由被动路径(后台收帧线程)处理:`27 01` 锚定帧写状态缓存
    /// (序号 +1,即一次状态计数);`26 0E` 经 2026-10-24 桥接就地补丁
    /// `t_mos`/`fault_code`(不推进序号)——**两回帧合为一次状态计数**,
    /// `request_fresh_state` 的新鲜度仍只由锚定帧定义。阻塞等新鲜状态用
    /// `request_fresh_state`,非阻塞回读用 `latest_state`/`cached_register`。
    /// 与 DM 的同名方法同构(发完即走)。
    pub fn request_motor_feedback(&self) -> Result<()> {
        self.request_registers(&[
            ReadCmd {
                addr: 0x01,
                count: 3,
            },
            ReadCmd {
                addr: 0x0E,
                count: 2,
            },
        ])
    }

    /// 阻塞式状态查询组合(DM 对齐命名,对应 DM 的同名方法):
    /// `request_motor_feedback` 发帧 + 轮询状态缓存,等到"本次请求之后
    /// 到达"(`received_at >= request_at`)的状态即返回;超时返回
    /// `MotorError::Timeout`。`request_at` 在发帧**前**记录,不存在
    /// "回复先到、基准后记"的漏认窗口。取代已删除的阻塞式
    /// `request_motor_feedback(timeout)`(其 wait_status 的 bus.recv
    /// 抢帧分支一并移除——回复一律归后台收帧线程,寄存器缓存不再被
    /// 旁路漏喂)。
    ///
    /// 2026-10-24 两帧版注:`received_at` 只被锚定帧(`27 01`)推进,
    /// `26 0E` 补丁帧不刷新它——本方法等到的恒是"新鲜 pos/vel/tq"。
    /// 真机回帧按请求顺序串行(锚定帧先到,补丁帧间隔 ~250µs),远小于
    /// 2ms 轮询周期,返回时温度/错误码补丁基本已落地;极端竞态下
    /// (返回瞬间补丁未到)t_mos/fault_code 为上一周期值,缺席不覆盖。
    pub fn request_fresh_state(&self, timeout: Duration) -> Result<HightorqueFeedbackState> {
        let request_at = Instant::now();
        self.request_motor_feedback()?;
        let deadline = request_at + timeout;
        loop {
            if let Some(snapshot) = self.latest_state_snapshot() {
                if snapshot.received_at >= request_at {
                    return Ok(snapshot.state);
                }
            }
            if Instant::now() >= deadline {
                return Err(MotorError::Timeout(format!(
                    "HighTorque read timeout on motor id {}",
                    self.motor_id
                )));
            }
            std::thread::sleep(Duration::from_millis(REGISTER_POLL_INTERVAL_MS));
        }
    }

    /// 通用寄存器读取(协议 §1.3)——DM 风格的"请求即发送":只把一组
    /// `(cmd, addr)` 读请求帧发到总线上,立即返回不等回帧。
    ///
    /// **多条请求逐帧连发**(2026-10-24 真机实测:固件只解析帧内第一对
    /// `(cmd, addr)`,非零尾随字节的多对拼帧被整帧静默丢弃——一帧一对,
    /// 每帧 2 字节规范 dlc)。整组地址先校验后发送,任一越界整组拒绝、
    /// 零帧发出;发送中途总线出错则返回该错误(此时前序帧已上线,不回滚)。
    ///
    /// 回复由被动路径(CoreController 后台收帧线程
    /// `process_feedback_frame` → `decode_register_reply`)按 `(addr, ty)`
    /// 写入 `register_cache`(带入缓时刻)。之后:
    /// - 阻塞等值:`read_registers`(发送 + 轮询新鲜缓存);
    /// - 非阻塞轮询:`cached_register`(读最近值)。
    pub fn request_registers(&self, cmds: &[ReadCmd]) -> Result<()> {
        // 2026-10-22 追补决策:缺号/表外地址拒绝访问——count 覆盖的每个
        // 地址都必须在寄存器表内,任一越界整段报错,不发任何帧。
        for c in cmds {
            registers::ensure_known_span(c.addr, c.count)
                .map_err(MotorError::InvalidArgument)?;
        }
        for c in cmds {
            let payload = encode_read(*c).ok_or_else(|| {
                MotorError::InvalidArgument(
                    "request_registers: count out of range (1..=3)".to_string(),
                )
            })?;
            self.send_query(&payload, 2)?;
        }
        Ok(())
    }

    /// 读寄存器缓存的最近值(不发帧不等待):该地址**跨类型**取入缓时刻
    /// 最新的条目(A3 修复后缓存按 `(addr, ty)` 分键,本接口无 ty 参数,
    /// 故取最近一条),再按该条目的实际类型经 `register_physical_value`
    /// 换算成物理量返回(2026-10-22 决策:读一律给换算后物理值;无换算
    /// 关系的寄存器 1:1)。返回 `None` 表示从未收到过该地址的回复,
    /// **或地址不在寄存器表内**(缺号/表外拒绝访问,2026-10-22 追补)。
    /// 注意:无法区分"新鲜"与"陈旧"(电机掉线后停留在最后一值);需要
    /// 新鲜度保证时用 `read_registers` 或先 `request_registers` 发查询、
    /// 隔一拍回读并自行判断。
    pub fn cached_register(&self, addr: u8) -> Option<f32> {
        if !registers::is_known_register(addr) {
            return None;
        }
        self.register_cache
            .lock()
            .ok()
            .and_then(|c| {
                c.iter()
                    .filter(|((a, _ty), _)| *a == addr)
                    .max_by_key(|(_k, (_v, ts))| *ts)
                    .map(|(_k, (v, _ts))| *v)
            })
            .map(|v| registers::register_physical_value(addr, v, self.torque_coeff))
    }

    /// 通用寄存器读取(协议 §1.3)的阻塞组合:`request_registers` 发帧 +
    /// 轮询 `register_cache`,以 `ts >= request_at` 时间戳新鲜度校验认领
    /// "本次请求之后到达的回复"。
    ///
    /// 与旧实现(直接 `bus.recv` 收帧)相比:不再与 CoreController 后台
    /// 收帧线程在同一总线队列上抢帧;时间戳即事务号,并发其它寄存器类
    /// 查询(如 `17 01` 状态查询)的回复按 `(addr, ty)` 入各自的缓存键,
    /// 类型不符的条目不会被本请求认领(A3 修复,2026-10-21)。
    ///
    /// 2026-10-22 决策:读取**固定按 Int16**(认领键恒 `(addr, Int16)`,
    /// 外来 Int8/I32 回复条目不会被认领),返回值经
    /// `register_physical_value` 换算为物理量(位置 rad、速度 rad/s、
    /// 力矩 N·m 套型号 k 系数、温度 °C 等;无换算关系的寄存器 1:1)。
    pub fn read_registers(
        &self,
        cmds: &[ReadCmd],
        timeout: Duration,
    ) -> Result<Vec<(u8, f32)>> {
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

    /// 从缓存取出覆盖全部请求地址且入缓时刻 >= `request_at` 的值;任一
    /// 条目缺失、类型不符(Int16 键)或陈旧则返回 `None`(继续等)。
    /// 地址推进与解码侧统一为 wrapping(2026-10-21 B3 修复:0xFF 起连续
    /// 读越过 0x100 回绕,两侧算法一致才能认领)。认领后逐条换算成
    /// 物理量。
    fn fresh_registers(
        &self,
        cmds: &[ReadCmd],
        request_at: Instant,
    ) -> Option<Vec<(u8, f32)>> {
        let cache = self.register_cache.lock().ok()?;
        let mut out = Vec::new();
        for c in cmds {
            for i in 0..c.count {
                let addr = c.addr.wrapping_add(i);
                match cache.get(&(addr, RegisterType::Int16)) {
                    Some((v, ts)) if *ts >= request_at => {
                        let physical = registers::register_physical_value(
                            addr,
                            *v,
                            self.torque_coeff,
                        );
                        out.push((addr, physical));
                    }
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

    /// 查询/配置类命令(read/rezero/store/conf_write):**扩展 29 位帧**,
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
        // 7 字节:设置 ACK(0x41...)。其余长度:可解析的寄存器读回复
        // (int16 count1=4 字节、float=6 字节等)——入寄存器缓存;
        // 不匹配则不归被动反馈,帧被丢弃。
        match frame.dlc {
            8 => true,
            7 => is_setting_ack(frame),
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
        // 寄存器读回复按 `(addr, ty)` 入缓存(带时刻):0x27 0x01 状态回复的
        // 0x01-0x03 也入(状态查询本身就是寄存器读, Int16), 短帧(4/6 字节)
        // 只走这里。类型取自解码出的 RegisterValue 变体,与回复帧 cmd
        // 半字节编码的类型一致。时间戳供 read_registers 的新鲜度校验认领。
        if let Some(vals) = decode_register_reply(&frame.data[..frame.dlc as usize]) {
            if let Ok(mut cache) = self.register_cache.lock() {
                let now = Instant::now();
                for (addr, v) in &vals {
                    cache.insert((*addr, v.register_type()), (*v, now));
                }
            }
            // 2026-10-24 桥接:模式/温度/错误码寄存器回帧就地补丁进状态
            // 缓存——**不推进序号、不刷新 received_at**(FeedbackCache::
            // patch_latest),与锚定帧 `27 01` 合为一次状态计数,计数逻辑
            // 不变。真机时序保证锚定帧先到(request_motor_feedback 先发
            // `17 01`,电机按请求顺序串行回帧,间隔 ~250µs)。映射:
            // - 0x00(模式)→ status_code,1:1;
            // - 0x0E(温度)→ t_mos,×0.1°C(与寄存器读换算同式);
            // - 0x0F(错误码)→ fault_code,1:1(与表3 直接对应,
            //   用户 2026-10-23 确认)。
            // 独立寄存器读(`15 00`/`16 0E`)同样受益——补丁无差别生效。
            // 首帧(锚定)未到时 patch 返回 false,补丁丢弃不报错。前置
            // any 判断:非桥接地址(如 27 01 周期流)不触碰状态锁。
            let has_bridge = vals
                .iter()
                .any(|(addr, _)| matches!(*addr, 0x00 | 0x0E | 0x0F));
            if has_bridge {
                self.state.patch_latest(|state| {
                    for (addr, v) in &vals {
                        match *addr {
                            0x00 => {
                                if let Some(mode) = bridge_u8(*v) {
                                    state.status_code = mode;
                                }
                            }
                            0x0E => {
                                state.t_mos =
                                    registers::register_physical_value(0x0E, *v, self.torque_coeff);
                            }
                            0x0F => {
                                if let Some(fault) = bridge_u8(*v) {
                                    state.fault_code = fault;
                                }
                            }
                            _ => {}
                        }
                    }
                })?;
            }
        }
        if let Some(mut state) = decode_feedback(frame, self.torque_coeff) {
            if state.can_id as u16 == self.motor_id {
                // 2026-10-23 决策: 帧未携带的字段**不覆盖缓存**(旧实现
                // 把缺席字段置 0, 一次 27 01 状态查询就会把控制回帧带来
                // 的故障码冲掉):
                // - `27 01` 读回复只有 pos/vel/tqe → status/fault/温度
                //   保持缓存旧值;
                // - 控制命令状态帧有 mode/fault 但同样没有温度 →
                //   t_mos/t_rotor 保持旧值。
                // 帧里真正有的字段照常更新(含 received_at 新鲜度推进,
                // request_fresh_state 不受影响)。
                let is_read_reply = frame.dlc >= 1 && frame.data[0] == 0x27;
                if let Ok(Some(prev)) = self.state.latest() {
                    if is_read_reply {
                        state.status_code = prev.state.status_code;
                        state.fault_code = prev.state.fault_code;
                    }
                    state.t_mos = prev.state.t_mos;
                    state.t_rotor = prev.state.t_rotor;
                }
                // 写入 FeedbackCache(内部推进序号与入缓时刻,供
                // request_fresh_state 新鲜度判定与 latest_state_snapshot)。
                self.state.update(state)?;
            }
        }
        Ok(())
    }
}

/// 桥接辅助(2026-10-24):寄存器 raw 值 → u8,1:1。整型取值在
/// 0..=255 才有效(运行模式 0-17、表3 错误码均在此域);浮点变体或
/// 越界/负值返回 `None`——该字段保持缓存旧值(缺席不覆盖原则)。
fn bridge_u8(v: RegisterValue) -> Option<u8> {
    let raw = match v {
        RegisterValue::I8(x) => i32::from(x),
        RegisterValue::I16(x) => i32::from(x),
        RegisterValue::I32(x) => x,
        RegisterValue::F32(_) => return None,
    };
    u8::try_from(raw).ok()
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
            .request_registers(&[ReadCmd { addr: 0x01, count: 3 }])
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
            .request_registers(&[ReadCmd { addr: 0x01, count: 3 }])
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
        // 同一帧按地址入寄存器缓存, 读出即换算后的物理量(与状态路径同式)
        assert!((motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU).abs() < 1e-4);
        assert!((motor.cached_register(0x02).unwrap() - 0x5678 as f32 / 4000.0 * std::f32::consts::TAU).abs() < 1e-4);
        // tqe_raw=0x0A00=2560 → 2560*0.01*k(General k=0.5) = 12.8
        assert!((motor.cached_register(0x03).unwrap() - 12.8).abs() < 1e-4);
    }

    /// 收敛(2026-09-30)回归:CoreController 后台收帧线程把 27 01 回帧
    /// 直接送进 process_feedback_frame(总线帧被它消费,主动查询方自己
    /// 收不到)时,阻塞式 request_fresh_state 也必须成功返回。
    #[test]
    fn request_fresh_state_ok_when_worker_consumes_reply() {
        let (_bus, motor) = make_motor(1);
        let motor2 = Arc::clone(&motor);
        let worker = std::thread::spawn(move || {
            // 模拟后台线程在主动查询等待期间收到回帧(回帧不进 MockBus
            // 的 rx 队列,主动方自己永远收不到它)。
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
        let s = motor
            .request_fresh_state(Duration::from_millis(500))
            .expect("worker-consumed reply should still satisfy freshness wait");
        worker.join().expect("worker join");
        assert_eq!(s.can_id, 1);
    }

    /// B1 回归(2026-10-21):request_motor_feedback(DM 对齐的非阻塞
    /// 语义)发出的即协议规范 2 字节帧(扩展 29 位 0x8000|id)——旧阻塞
    /// 方法曾发 8 字节零填充帧,与文档不一致。
    /// 2026-10-24:扩展为**连发两帧**(`17 01` pos/vel/tq + `16 0E`
    /// 温度/错误码)——真机实测固件一帧一对,拼帧被静默丢弃。
    #[test]
    fn request_motor_feedback_sends_canonical_two_byte_frame() {
        let (bus, motor) = make_motor(1);
        motor.request_motor_feedback().expect("send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 2, "两帧:17 01 + 16 0E");
        for f in sent.iter() {
            assert_eq!(f.arbitration_id, 0x8000 | 1);
            assert!(f.is_extended, "查询帧应为扩展 29 位");
            assert_eq!(f.dlc, 2, "每帧 2 字节规范 dlc");
        }
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
        assert_eq!(&sent[1].data[..2], &[0x16, 0x0E]);
    }

    /// 2026-10-24:多条 ReadCmd 逐帧连发(一帧一对),整组地址先校验。
    #[test]
    fn request_registers_sends_one_frame_per_cmd() {
        let (bus, motor) = make_motor(1);
        motor
            .request_registers(&[
                ReadCmd {
                    addr: 0x01,
                    count: 3,
                },
                ReadCmd {
                    addr: 0x0E,
                    count: 2,
                },
            ])
            .expect("send");
        let sent = bus.sent.lock().expect("sent frames");
        assert_eq!(sent.len(), 2, "两条命令两帧,不拼帧");
        assert_eq!(&sent[0].data[..2], &[0x17, 0x01]);
        assert_eq!(&sent[1].data[..2], &[0x16, 0x0E]);
    }

    /// 2026-10-24 两帧状态计数(核心语义):`26 0E` 回帧把温度/错误码
    /// **就地补丁**进状态缓存——序号不推进(一次 request_feedback 周期
    /// 只计一次数),字段值来自补丁帧;寄存器缓存照常入缓(独立键独立
    /// 时间戳,read_registers 新鲜度认领不受影响)。
    #[test]
    fn temp_error_reply_patches_state_without_bumping_count() {
        let (_bus, motor) = make_motor(1);
        // 锚定帧 27 01:pos/vel/tqe + 一次状态计数
        let anchor = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(anchor).expect("process anchor");
        let after_anchor = motor.latest_state_snapshot().expect("snapshot");
        assert_eq!(after_anchor.sequence, 1);
        assert_eq!(after_anchor.state.t_mos, 0.0);
        assert_eq!(after_anchor.state.fault_code, 0);

        // 补丁帧 26 0E:温度 0x0142=322→32.2°C,错误码 5
        let patch = CanFrame {
            arbitration_id: 0x0100,
            data: [0x26, 0x0E, 0x42, 0x01, 0x05, 0x00, 0x00, 0x00],
            dlc: 6,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(patch).expect("process patch");
        let after_patch = motor.latest_state_snapshot().expect("snapshot");
        // 两回帧合为一次状态计数:序号不动
        assert_eq!(after_patch.sequence, 1, "补丁不推进序号");
        assert_eq!(
            after_patch.received_at, after_anchor.received_at,
            "补丁不刷新时间戳(新鲜度由锚定帧定义)"
        );
        assert_eq!(after_patch.state.pos, after_anchor.state.pos);
        assert!((after_patch.state.t_mos - 32.2).abs() < 1e-4, "0x0142 × 0.1°C");
        assert_eq!(after_patch.state.fault_code, 5, "0x0F 1:1 桥接");
        // 寄存器缓存照常:温度/错误码可独立回读(物理量/1:1)
        assert!((motor.cached_register(0x0E).unwrap() - 32.2).abs() < 1e-4);
        assert_eq!(motor.cached_register(0x0F).unwrap(), 5.0);
    }

    /// 2026-10-24:独立模式读(`25 00` 回帧)同样经桥接补丁 status_code,
    /// 不推进序号;锚定帧未到时补丁丢弃(静默,不建快照)。
    #[test]
    fn mode_reply_patches_status_code_without_bumping_count() {
        let (_bus, motor) = make_motor(1);
        // 首帧(锚定)未到:补丁无处落
        let orphan = CanFrame {
            arbitration_id: 0x0100,
            data: [0x25, 0x00, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00],
            dlc: 4,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(orphan).expect("process orphan");
        assert!(motor.latest_state().is_none(), "补丁不创建快照");

        // 锚定帧之后补丁生效
        let anchor = CanFrame {
            arbitration_id: 0x0100,
            data: [0x27, 0x01, 0, 0, 0, 0, 0, 0],
            dlc: 8,
            is_extended: false,
            is_rx: true,
        };
        motor.process_feedback_frame(anchor).expect("process anchor");
        motor.process_feedback_frame(orphan).expect("process mode patch");
        let st = motor.latest_state_snapshot().expect("snapshot");
        assert_eq!(st.sequence, 1, "补丁仍不推进序号");
        assert_eq!(st.state.status_code, 7, "0x00 模式 1:1 桥接");
    }

    #[test]
    fn accepts_frame_admits_status_register_and_ack() {
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
        // 5 字节 Int8×3 寄存器回复(cmd=reply|Int8|count3,2+3=5 字节):
        // 版本查询删除后 5 字节帧改走寄存器解码分支——旧实现
        // `5 => decode_version_reply` 会拒收该帧,导致 Background 模式下
        // Int8×3 连续读的回复永不被认领(回归保护)。
        let int8x3 = CanFrame {
            arbitration_id: 0x0100,
            data: [0x23, 0x22, 0x11, 0x22, 0x33, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        };
        assert!(motor.accepts_frame(&int8x3));
        motor.process_feedback_frame(int8x3).expect("process");
        // 外来 Int8 缓存条目按换算推广读取(2026-10-22):
        // 0x22 前馈扭矩 I8(17) → 17/2×k(0.5) = 4.25 N·m;
        // 0x23 Kp 比例 I8(0x22=34) → 34/32767 归一化。
        assert!((motor.cached_register(0x22).unwrap() - 4.25).abs() < 1e-4);
        assert!((motor.cached_register(0x23).unwrap() - 34.0 / 32767.0).abs() < 1e-6);
        // 已删除的版本回复帧(15 B5 02..,dlc 5):首字节 0x15 非
        // REPLY_NIBBLE,寄存器解码拒绝,被动路径仍不认领。
        assert!(!motor.accepts_frame(&CanFrame {
            arbitration_id: 0x0100,
            data: [0x15, 0xB5, 0x02, 0x12, 0x34, 0, 0, 0],
            dlc: 5,
            is_extended: true,
            is_rx: true,
        }));
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
            .read_registers(&[ReadCmd { addr: 0x01, count: 3 }], Duration::from_millis(10))
            .ok();
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1);
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
            .read_registers(&[ReadCmd { addr: 0x0E, count: 1 }], Duration::from_millis(500))
            .expect("应等到 0x0E 的新鲜回复");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0E, "返回的应是地址 0x0E 的值");
        // I16 raw 100 → 温度 0.1°C/LSB → 10.0°C(换算后物理量)
        assert!((vals[0].1 - 10.0).abs() < 1e-4);
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
        let r = motor.read_registers(&[ReadCmd { addr: 0x0E, count: 1 }], Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::Timeout(_))),
            "旧缓存值不应被认领, 应超时: {r:?}"
        );
    }

    /// 回归(A3, 2026-10-21;2026-10-22 读取固定 Int16 后重写): 同地址
    /// 不同类型的缓存条目互不污染。请求固定按 Int16 键 `(addr, Int16)`
    /// 认领——先到的外来 Int8 回复(别的主机发的 0x21 帧,以 Int8 键
    /// 入缓)虽然更新且同地址,但键不匹配,永不被本事务认领;直到
    /// +40ms 后自己的 Int16 回复(0x25 帧)到达才返回。
    /// 旧实现(键只有地址)会在 Int8 帧到达时即认领它,返回外来路径
    /// 换算值 1.0°C(0x0A=10,/10)而非本事务的 10.0°C。
    #[test]
    fn read_registers_rejects_fresh_reply_of_different_type() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            // 请求发出后先到: 别的主机发的 Int8 读 0x0E 回复(0x21, dlc 3)。
            // 与 Int16 回复间隔 40ms(>> 2ms 轮询间隔),保证读循环至少
            // 轮询一次时缓存里只有 Int8 条目——旧实现(键只有地址)在此
            // 时刻即认领 I8 返回,判别可靠;若两帧背靠背,旧实现"最后
            // 写入者胜"碰巧返回 I16,测试失去判别力。
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x21, 0x0E, 0x0A, 0, 0, 0, 0, 0],
                dlc: 3,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge int8 reply");
            std::thread::sleep(Duration::from_millis(40));
            // 再到: 本事务(固定 Int16)对 0x0E 的回复(0x25, dlc 4)
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0E, 0x64, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge int16 reply");
        });
        let vals = motor
            .read_registers(&[ReadCmd { addr: 0x0E, count: 1 }], Duration::from_millis(500))
            .expect("应等到 Int16 回复, 而非认领外来 Int8 缓存条目");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0E);
        // I16 raw 100 → 10.0°C(温度换算);外来 I8 条目(0x0A=10)则会被
        // 换算成 1.0°C(Int8 温度同为 0.1°C/LSB),判别充分。
        assert!(
            (vals[0].1 - 10.0).abs() < 1e-4,
            "应返回 Int16 路径的 10.0°C, 实得 {:?}(旧实现会错拿外来 Int8 条目)",
            vals[0].1
        );
    }

    /// 回归(B3, 2026-10-21;2026-10-22 追补后改写): 缺号/表外地址拒绝
    /// 访问——读取 span 覆盖的任一地址不在寄存器表内即**整段报错、
    /// 零发帧**。表最高地址 0x45,合法请求永不越过 0xFF,0xFF 起连续读
    /// 的地址回绕(B3 原始 bug)在自家请求路径上已不可达,回绕语义仅
    /// 保留给解码侧处理外来帧。
    #[test]
    fn read_registers_rejects_unknown_addr_span() {
        let (_bus, motor) = make_motor(1);
        // 表外单地址(0xFF)
        let r = motor.read_registers(&[ReadCmd { addr: 0xFF, count: 1 }], Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0xFF")),
            "表外地址应整段拒绝: {r:?}"
        );
        // 缺号地址(0x0A, 0x00-0x45 内的洞)
        let r = motor.read_registers(&[ReadCmd { addr: 0x0A, count: 1 }], Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x0A")),
            "缺号地址应拒绝: {r:?}"
        );
        // 连续读跨入缺号:0x2D(Ki,表内)count2 覆盖 0x2E(缺号)→ 整段拒绝
        let r = motor.read_registers(&[ReadCmd { addr: 0x2D, count: 2 }], Duration::from_millis(30));
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x2E")),
            "span 覆盖缺号应整段拒绝: {r:?}"
        );
        // 表内连续读照常放行(会超时,但帧已发出)
        let (bus, motor) = make_motor(1);
        let _ = motor.read_registers(&[ReadCmd { addr: 0x01, count: 3 }], Duration::from_millis(10));
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 1, "表内地址应正常发帧");
        // 写路径同样拒绝表外
        let (bus, motor) = make_motor(1);
        let r = motor.write_register(0x50, 0, false);
        assert!(
            matches!(r, Err(MotorError::InvalidArgument(ref e)) if e.contains("0x50")),
            "写表外地址应拒绝: {r:?}"
        );
        let sent = bus.sent.lock().expect("sent");
        assert_eq!(sent.len(), 0, "拒绝发生在发帧之前");
    }

    /// 2026-10-22 决策守卫:同一 `27 01` 回帧,`read_registers`(0x01-0x03,
    /// 固定 Int16)返回的换算后 f32 与 `latest_state()` 的 pos/vel/torq
    /// 逐位一致——寄存器读路径与状态路径的换算必须始终对齐(两路径
    /// 各自改 scale/系数时此测试兜底)。
    #[test]
    fn read_registers_pos_matches_state_path() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: false,
                is_rx: true,
            })
            .expect("merge state reply");
        });
        let vals = motor
            .read_registers(&[ReadCmd { addr: 0x01, count: 3 }], Duration::from_millis(500))
            .expect("应等到状态回复");
        handle.join().expect("thread join");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(vals.len(), 3);
        assert!((vals[0].1 - st.pos).abs() < 1e-5, "pos 分歧: {:?} vs {st:?}", vals[0].1);
        assert!((vals[1].1 - st.vel).abs() < 1e-5, "vel 分歧: {:?} vs {st:?}", vals[1].1);
        assert!((vals[2].1 - st.torq).abs() < 1e-5, "torq 分歧: {:?} vs {st:?}", vals[2].1);
    }

    /// 0x0F(错误代码)无换算关系 → 1:1 恒等返回原值。
    #[test]
    fn read_registers_identity_for_0x0f() {
        let (_bus, motor) = make_motor(1);
        let m = motor.clone();
        let handle = std::thread::spawn(move || {
            std::thread::sleep(Duration::from_millis(10));
            m.process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x0F, 0x20, 0x00, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge err reply");
        });
        let vals = motor
            .read_registers(&[ReadCmd { addr: 0x0F, count: 1 }], Duration::from_millis(500))
            .expect("应等到 0x0F 回复");
        handle.join().expect("thread join");
        assert_eq!(vals.len(), 1);
        assert_eq!(vals[0].0, 0x0F);
        assert_eq!(vals[0].1, 32.0, "无换算关系的寄存器应 1:1 返回 raw");
    }

    /// 非阻塞路径:request_registers 发完即走(帧格式/扩展 id 正确),
    /// cached_register 在回复入缓前为 None、入缓后可读;accepts_frame
    /// 认领 4 字节的寄存器回复(此前该长度被拒收丢弃)。
    #[test]
    fn request_registers_then_cached_register_via_passive_path() {
        let (bus, motor) = make_motor(1);
        motor
            .request_registers(&[ReadCmd { addr: 0x0E, count: 1 }])
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
        // I16 raw 100 → 10.0°C(读出即换算后的物理量)
        assert!((motor.cached_register(0x0E).unwrap() - 10.0).abs() < 1e-4);
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
        // 读出即换算后的物理量,与状态路径同式。
        assert!((motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU).abs() < 1e-6);
        // I16(0) → 0×0.01×k+d = 0.0
        assert!((motor.cached_register(0x03).unwrap() - 0.0).abs() < 1e-6);
    }

    /// 2026-10-23 决策回归:帧未携带的字段不覆盖状态缓存——8 字节状态
    /// 帧(byte0=mode, byte1=fault, §3.2 格式)先带来 fault=0x22,随后
    /// 的 `27 01` 读回复只有 pos/vel/tqe,故障码/运行模式应保持旧值
    /// (旧实现置 0,一次状态查询就冲掉)。注:v2.0.0 真机上该状态帧
    /// 无已知生产者(控制命令只回 DLC=0 空帧,见 can_protocol §3.2
    /// 更正/§1.2),此处是构造帧测合并逻辑;帧里有的字段(pos)照常更新。
    #[test]
    fn read_reply_preserves_cached_fault_code() {
        let (_bus, motor) = make_motor(1);
        // 控制命令附带状态帧: byte0=mode(0x0A 位置), byte1=fault(0x22)。
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x0A, 0x22, 0x34, 0x12, 0x78, 0x56, 0x00, 0x0A],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge status frame");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x0A);
        assert_eq!(st.fault_code, 0x22);

        // 随后 27 01 读回复(不含 mode/fault 字节)。
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge read reply");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x0A, "27 01 不携带运行模式, 应保持旧值");
        assert_eq!(st.fault_code, 0x22, "27 01 不携带故障码, 应保持旧值");
        // 帧里有的字段照常更新: pos raw 0x0009 → 9×0.0001 圈×2π。
        assert!((st.pos - (9.0 * 0.0001 * std::f32::consts::TAU)).abs() < 1e-6);

        // 反向:新的控制状态帧照常刷新故障码(mode 0/fault 0 = 停止并
        // 清错后电机的正常回帧)。
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("merge clearing frame");
        let st = motor.latest_state().expect("state cache");
        assert_eq!(st.status_code, 0x00);
        assert_eq!(st.fault_code, 0x00, "控制状态帧携带故障码, 照常刷新");
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

    // 自 origin/main 合入:验证 FeedbackCache 语义——只有可解码的
    // 状态帧才推进快照序号与入缓时刻。帧格式适配 v2.0.0
    // (回帧 id=motor_id<<8;main 原测试的裸 id 布局是旧协议)。
    #[test]
    fn only_decoded_status_advances_the_snapshot() {
        let bus: Arc<dyn CanBus> = Arc::new(MockBus::new());
        let motor = HightorqueMotor::new(1, 1, "hightorque", bus);

        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x27, 0x01, 0, 0, 0, 0, 0, 0],
                dlc: 8,
                is_extended: true,
                is_rx: true,
            })
            .expect("status feedback");
        let first = motor.latest_state_snapshot().expect("first snapshot");
        assert_eq!(first.sequence, 1);

        // v2.0.0 下 8 字节非 0x27 帧均为合法被动状态帧(原 main 测试的
        // [0x01,..] 布局已不成立),改用短帧寄存器回复(cmd 0x25 =
        // reply+Int16+count1,dlc 4):只入寄存器缓存,不推进状态
        // 快照——同时验证两条缓存的分离。
        motor
            .process_feedback_frame(CanFrame {
                arbitration_id: 0x0100,
                data: [0x25, 0x01, 0x34, 0x12, 0, 0, 0, 0],
                dlc: 4,
                is_extended: true,
                is_rx: true,
            })
            .expect("register reply frame");
        let after_non_state = motor.latest_state_snapshot().expect("snapshot remains");
        assert_eq!(after_non_state.sequence, first.sequence);
        assert_eq!(after_non_state.received_at, first.received_at);
        // 短帧仍应入寄存器缓存,读出为换算后物理量(0x01 raw 0x1234 → rad)。
        assert!((motor.cached_register(0x01).unwrap() - 0x1234 as f32 * 0.0001 * std::f32::consts::TAU).abs() < 1e-6);
    }
}
