# 高擎电机（HighTorque）四层接口全链路审计

> 审计日期：2026-09-28（同日完成 G3/G4 修复，见 §8；2026-09-30 全链路收敛后更新，见 §9；同日 vendor 能力补通，见 §10；同日 A 类 vendor 原语补全，见 §11）
> 审计范围：上位机 ws_gateway / Rust CLI motor_cli / Python bindings / C ABI motor_abi × vendor crate
> 基准：
> - vendor 层实现：`motor_vendors/hightorque/`（ht_can v2.0.0 迁移，commit f652d2b）
> - 协议参考固件：`can_h730_v2.0.0.zip` 内 `libelybot_can.c`（参考固件命令集）
> - 协议文档：01-CAN协议解析.pdf（§1.2–§1.8）、表1-寄存器功能、表2-运行模式、表3-报错代码
> - 协议整理稿：[hightorque_can_protocol.md](hightorque_can_protocol.md)（仿 RobStride 手册体例的全命令帧格式与 CAN 帧示例）

---

## 1. 总体结论：四层全部收敛到 vendor（2026-09-29）

```
上位机 ws_gateway ──> motor_vendor_hightorque ──> CAN 总线   ✅ 控制/状态/参数链路收敛（scan 发现仍裸总线）
Rust CLI motor_cli ──> motor_vendor_hightorque ──> CAN 总线   ✅ 同上（scan 与 5 种诊断帧仍裸总线）
Python bindings ──ctypes──> motor_abi(C ABI) ──> motor_vendor_hightorque   ✅
```

- ~~收敛（commit c842d7d，2026-09-29）后，CLI 与 ws_gateway 的控制/状态/参数链路全部经 vendor crate；`decode_read_reply` 的三份手写拷贝消灭。仍保留裸总线的仅两处：scan 发现（CLI 与 gateway 各一份 `17 01`/`27 01` 收发，vendor 无 scan API）与 vendor 无对应命令的 CLI 诊断帧（`tqe`/`volt`/`cur`/`brake`/`timed-read`，经 diag_bus 总线副本直发，与 vendor 收帧线程互不干扰）。~~ **[2026-09-30 更新，见 §11]**：诊断帧路径已删（五类命令全走 vendor 原语），裸总线仅剩 scan 发现一处（CLI 与 gateway 各一份，vendor 无 scan API）。
- ~~ABI/Python 仍是 vendor 能力的子集（§3 B 类缺口不变）。~~ **[2026-09-30 补通，见 §10]**：梯形/位置经典/寄存器读/固件版本/fault_code 五项全部接通四层，B 类仅剩 `send_with_ack`（有意不接）与 `torque_coeff`。**[2026-09-30 再补，见 §11]**：A 类 7 行（力矩/DQ 电压/DQ 电流/刹车/模式切换/寄存器写/周期上报）vendor 原语新增并接通四层，矩阵全绿。
- 四层协议世代：全部 v2.0.0。
- ~~commit f652d2b 提交信息声称给 CLI 增加了 v2.0.0 模式，实际只改 15 行~~（历史结论，收敛后不再成立）。

---

## 2. 全链路能力矩阵（2026-09-30 收敛后）

以参考固件命令集（libelybot_can.c）+ 协议文档为基准。图例：✅ 已实现并路由｜⚠️ 部分实现/语义偏差｜❌ 未实现｜`手写` = 不经过 vendor crate 的独立实现｜`diag` = CLI 诊断帧，经 diag_bus 裸发（2026-09-30 后 diag_bus 已删）｜[收敛] = 2026-09-29 收敛 commit c842d7d 接通｜[补] = 2026-09-30 vendor 能力补通（见 §10）｜[新] = 2026-09-30 vendor 原语新增（见 §11）｜[B] = 2026-10-08 enable 方案 B 决策：语义空操作（见 §12）。

| # | 功能 | 线上帧 | vendor | C ABI | Python | CLI | ws_gateway |
|---|---|---|---|---|---|---|---|
| 1 | 停止 / 清错（模式 0） | `01 00 00` | ✅ disable | ✅（stop=disable 别名） | ✅ | ✅ [收敛] vendor disable | ✅ [收敛] vendor disable |
| 2 | MIT 位打包（v4.6.0+） | `0x18000\|id` | ✅ | ✅ | ✅ | ✅ [收敛] vendor send_cmd_mit | ✅ [收敛] vendor send_cmd_mit |
| 3 | 协同控制（pos+vel+tqe） | `07 35` | ✅ | ✅ | ✅ | ✅ [收敛] vendor send_cmd_pos_vel（pos-vel-tqe 入口，力矩恒无限制） | ✅ [收敛] vendor send_cmd_pos_vel（此前被拒） |
| 4 | 梯形控制（v4.6.0+） | `07 2d` | ✅ | ✅ [补] pos_vel_acc | ✅ | ✅ [补] pos-vel-acc 模式 | ✅ [补] pos_vel_acc op（含 continuous tick） |
| 5 | 普通模式-位置+力矩 | `07 07` | ✅ | ✅ [补] pos_classic（NaN=无限制） | ✅ | ✅ [收敛] vendor send_cmd_pos_classic | ✅ [补] pos_classic op |
| 6 | 普通模式-速度+力矩 | `07 07` | ✅ | ✅ | ✅ | ✅ [收敛] vendor send_cmd_vel | ✅ [收敛] vendor send_cmd_vel（无限制哨兵由 vendor 填） |
| 7 | 力矩模式 | `05 13`（写寄存器 0x13） | ✅ [新] send_cmd_torque（物理量 Nm+coeff） | ✅ [新] send_torque | ✅ [新] hightorque_send_torque | ✅ [新] tqe 模式（--tau Nm，删 raw-tqe） | ✅ [新] torque op（含 continuous tick） |
| 8 | DQ 电压（模式 8+写 0x1b） | `01 00 08` + `05 1b`（拼一帧） | ✅ [新] send_cmd_dq_voltage（0.1 V/LSB） | ✅ [新] send_dq_voltage | ✅ [新] hightorque_send_dq_voltage | ✅ [新] volt 模式（--volt V，删 raw-vol） | ✅ [新] dq_voltage op |
| 9 | DQ 电流（模式 9+写 0x1c） | `01 00 09` + `05 1c`（拼一帧） | ✅ [新] send_cmd_dq_current（0.1 A/LSB） | ✅ [新] send_dq_current | ✅ [新] hightorque_send_dq_current | ✅ [新] cur 模式（--cur A，删 raw-cur） | ✅ [新] dq_current op |
| 10 | 刹车（模式 15） | `01 00 0F` | ✅ [新] brake | ✅ [新] brake | ✅ [新] hightorque_brake | ✅ [新] brake 模式（vendor 调用） | ✅ [新] brake op |
| 11 | 运行模式切换（写 0x000） | `01 00 <mode>` | ✅ [新] send_run_mode（0-17 校验+真发帧；ensure_control_mode 保持仅校验语义不变） | ✅ [新] set_run_mode | ✅ [新] hightorque_set_run_mode | ✅ [新] set-mode 模式（--run-mode 0-17） | ✅ [新] run_mode op |
| 12 | 通用寄存器读（0x17 族） | `17 <cmd/addr>` | ✅ read_registers | ✅ [补] get_register（ty+count，raw 值） | ✅ | ✅ [补] read-reg 模式（--addr/--count/--ty，名称取寄存器表） | ✅ [补] read_register op |
| 13 | 读状态（= #12 特例） | `17 01` | ✅ request_motor_feedback | ✅（500ms 硬编码） | ✅ | ✅ [收敛] vendor request_motor_feedback（state_seq 后轮询模式下不再假超时） | ✅ [收敛] vendor（read/state_once/status/verify） |
| 14 | 通用寄存器写 | `05 <reg> <val>` | ✅ [新] write_register（控制类走控制通道，配置类走扩展帧） | ✅ [新] write_register（addr+val16+config_class） | ✅ [新] hightorque_write_register | ✅ [新] write-reg 模式（--addr/--value/--config） | ✅ [新] write_register op |
| 15 | 周期上报（0x05 B4） | `05 B4 02 <t>` | ✅ [新] set_periodic_report（0=停止） | ✅ [新] set_periodic_report | ✅ [新] hightorque_set_periodic_report | ✅ [新] periodic-report 模式（--period-ms；timed-read 保留别名） | ✅ [新] periodic_report op |
| 16 | 零位设置 | `40 01 04 64 20 63 0A` | ✅ | ✅ | ✅ | ✅ [收敛] vendor set_zero_position（自动落盘） | ✅ [收敛] vendor（此前直接拒绝） |
| 17 | 参数保存 conf_write | `05 B3 02 00 00` | ✅ | ✅ | ✅ | ✅ [收敛] vendor store_parameters | ✅ [收敛] vendor（此前直接拒绝） |
| 18 | 固件版本查询 | `15 B5 02` | ✅ | ✅ [补] request_firmware_version（out major/minor/patch） | ✅ | ✅ [补] version 模式 | ✅ [补] version op（解除 myactuator-only） |
| 19 | 故障码解码（0–47，表3） | 状态帧 byte1 | ✅ decode_fault | ✅ [补] MotorState.fault_code（末尾追加，不改布局） | ✅ | ✅ [收敛] read/ping 打印 fault_code | ✅ [收敛] read/state_once 输出 fault_code |
| 20 | 运行模式显示（0–17，表2） | 状态帧 byte0 | ✅ RunMode | ⚠️ 原始字节进 status_code | ❌ | ✅ [收敛] read/ping 打印 status | ✅ [收敛] status_code 真值（此前恒 0） |
| 21 | 力矩型号补偿（P2-15） | — | ✅ model→TorqueCoeff（add_motor 时解析） | — | ✅（demo 传 6056_36） | ✅ [收敛] --model 生效 | ✅ [收敛] --model 生效 |
| 22 | ACK 确认机制 | `41 01 04 OK\r\n` | ✅ send_with_ack | ❌ | ❌ | ❌ | ❌ |
| 23 | enable | 协议无此命令 | ✅ [B] 语义空操作 Ok（不发帧） | ✅ [B] 透传 Ok | ✅ [B] 正常返回（不抛异常） | —（无 enable 模式，不变） | ✅ [B] 接受（no-op 成功） |
| 24 | clear_error | ≡ 停止帧（模式 0） | ✅ =send_stop 同帧 | ✅ 透传 | ✅ | —（stop 模式同帧，无独立入口） | ✅ [收敛] vendor clear_error |

---

## 3. 缺失分三类

### A 类：vendor 层缺失（任何上层不可能有）

**[2026-09-30 全部消灭，见 §11]**：原 4 个原语全部落地——寄存器写（`write_register`，含力矩/DQ 目标下发的共享帧族）、真实模式切换帧（`send_run_mode`）、周期上报（`set_periodic_report`）。力矩型号补偿（#21）本就在 vendor 内（`tqe_adjust_to_raw`），#7 力矩原语直接复用。A 类缺口清零。

~~按**实现原语**计数（非功能行数）：~~

| 原语 | 影响 | 说明 |
|---|---|---|
| ~~寄存器写 encode_write~~ | ~~#7 力矩、#8 DQ电压、#9 DQ电流（`05 <reg> <val>` 一族多用途）~~ | ~~表1：0x1b=Q电压 R/W、0x1c=Q电流 R/W、0x13=力矩指令~~ **[新] `send_cmd_torque`/`send_cmd_dq_*`/`write_register` 落地** |
| ~~真实的模式切换帧~~ | ~~#8/#9/#10/#11~~ | ~~`01 00 <mode>` 写寄存器 0x000~~ **[新] `send_run_mode` 落地（ensure_control_mode 的仅校验语义按 trait 契约保留）** |
| ~~周期上报~~ | ~~#15~~ | ~~`05 B4 02 + t_ms`~~ **[新] `set_periodic_report` 落地** |
| ~~力矩型号补偿~~ | ~~#21~~ | 本就实现（`tqe_adjust_to_raw`），#7 复用之 |

~~功能缺口仍为 7 项（力矩/DQ电压/DQ电流/刹车/模式切换/周期上报/补偿），但实现工作量收敛为上表 4 个原语；补 2 个原语（寄存器写+模式切换）即点亮 4 个功能行。~~ **[2026-09-30 A 类清零，见 §11]**：7 项功能全部点亮，实现即按此收敛路径（寄存器写 + 模式切换两原语，外加周期上报独立帧）。

### B 类：vendor 已实现、ABI 未暴露（Python 随之缺失）

1. ~~梯形模式 `send_cmd_pos_vel_acc`（#4）— ABI 无函数；协议文档自己都推荐用梯形替代普通位置模式（§2.3 注意）~~ **[补通消灭，见 §10]**
2. ~~`send_cmd_pos_classic`（#5）~~ **[补通消灭，见 §10]**
3. ~~`read_registers` + PARAMETER_TABLE 0x00–0x45（#12）— `get_register_*` 对 hightorque 报 Damiao-only~~ **[补通消灭，见 §10]**
4. ~~`request_firmware_version`（#18）— ABI 无对应 C 函数~~ **[补通消灭，见 §10]**
5. `send_with_ack`（#22）— 底层原语，无独立用户语义，上层应调语义化命令而非裸帧+ACK，不补
6. `torque_coeff`
7. ~~fault_code（#19）：`MotorState` 无 fault 字段，`HightorqueFeedbackState.fault_code`（protocol.rs）被静默丢弃；RunMode（#20）不解码~~ **[补通消灭（fault_code 部分），见 §10]**；RunMode 字符串解码（#20）属显示层美化，不补（status_code 原始字节已透传）
8. ~~幽灵符号：.so 导出 10 个 `motor_handle_hightorque_{get,write}_param_*`，全是 "not supported yet" stub，头文件未声明、Python 未绑定，但 capabilities JSON 宣称支持 `param_i8…f32`（违反仓库“不支持的能力不得返回成功”规则）~~ **[补通消灭，见 §10]**：10 个 stub 符号与 `vendor_params/hightorque.rs`、`param_ffi/hightorque/` 整体删除；capabilities JSON 的 `hightorque` 数组改为真实能力清单（mit/vel/pos_vel/pos_vel_acc/pos_classic/request_feedback_async/firmware_version/get_register/fault_code）

### C 类：上层手写实现与 vendor 分叉（正确性风险）

1. ~~**帧格式分歧（潜在 bug）**：CLI 与 ws_gateway 把控制命令发成扩展帧 `0x8000|id`；vendor（与参考固件一致）控制类用标准 11 位裸 id，仅查询/配置类用 `0x8000|id`~~ **[收敛消灭]**：两层控制命令全部经 vendor `send_control`/`send_query`，帧型由 vendor 决定；~~仅剩 CLI 诊断帧（配置类 `05 xx`/`01 00`）以 `0x8000|id` 裸发，与 vendor 配置类帧型一致~~ **[2026-09-30 诊断帧路径删除（见 §11），分叉来源不复存在]**
2. ~~CLI `mit` 模式实际发 `07 35`~~ **[已修复]**（§8 G4）；**[收敛]**：CLI `mit` 整链改走 vendor `send_cmd_mit`，不再本地调 `encode_mit_frame`
3. ~~ws_gateway `mit` 发 `07 35` 且丢 kp/kd~~ **[已修复]**（§8 G4）；**[收敛]**：gateway `mit` 处理器与 runtime 连续 tick 改走 vendor `send_cmd_mit`，`send_hightorque_mit` 随 hightorque_ws.rs 一并删除
4. ~~ws_gateway `enable`/`disable` 假成功 no-op~~ **[收敛消灭]**：disable 走 vendor 停止帧；capabilities 矩阵同步（modes: mit/pos_vel/vel；ops_unified 补 disable/stop，ops_vendor_native 补 set_zero_position/store_parameters）。enable 当时收敛为透传 vendor Err，**2026-10-08 按方案 B 改为语义空操作**（行 23，见 §12）：Err 会中止混总线 `enable_all()` 使后续电机使能不到，统一 enable 流程需要 no-op 兼容
5. ~~ws_gateway `vel` 力矩字段写 0~~ **[收敛消灭]**：vendor `send_cmd_vel` 填 0x8000 无限制哨兵
6. ~~`set_zero_position`/`store_parameters` ws_gateway 直接拒绝~~ **[收敛消灭]**：走 vendor
7. 文档漂移：~~README 称 force_pos 映射 pos+vel+tqe、称 MIT 忽略 kp/kd~~ **[已修复]**（README/PROTOCOL/Python README/CLI help 更新）；PROTOCOL.zh-CN.md 是准确的一份
8. ~~CLI `--model`/`--feedback-id` 对 hightorque 是死参数~~ **[收敛消灭]**：`--model` 注册时校验并决定力矩系数；scan 强制 send_count=1 忽略 `--loop` 仍存在
9. ~~CLI/mit 与 pos-vel-tqe 字节级相同~~ **[已修复]**：mit 走 `0x18000|id`，pos-vel-tqe 走 `07 35`，两入口分道；robstride_cli.rs 内 5 处 hightorque compat 分支为死代码

---

## 4. 重复能力接口（协议层归并）

用“功能 / 命令 / 原子”三层定性。**真重复只有 2 处；其余是命令原子复用或特例关系。**

| 组 | 矩阵行 | 命令层 | 功能层 | 定性 |
|---|---|---|---|---|
| G3 | 停止 ≡ clear_error | 同一帧 `01 00 00` | 同一行为（表2 模式 0 名称即“停止，清除错误”） | **真重复**：一个命令一个动作，两个 API 名；clear_error 是伪缺口。**[已修复]**：vendor `clear_error()`=`send_stop()`（带单测），ABI/Python 透传自动生效，gateway clear_error 分支发 `01 00 00` |
| G4a | CLI `mit` ≡ CLI `pos-vel-tqe` | 同一帧、字节级相同 | 同一行为（协同控制） | **真重复**：一份代码两个入口名。**[已修复]**：mit 改走 v2.0.0 `0x18000\|id` 位打包帧（kp/kd 生效），两入口分道 |
| G4b | CLI/gateway 的"MIT" vs 真 MIT | ~~`07 35` vs `0x18000\|id`~~ | ~~协同 ≠ MIT（有无 kp/kd）~~ | **名不副实**，非重复。**[已修复]**：两层的 mit 均改发真 MIT 帧 |
| G1 | #5 位置+力矩 / #6 速度+力矩 | 同一 `07 07` 帧 | 不同（靠 0x8000 填充区分） | **原子复用**（协议 §1.2.1“位置和速度不能同时控制”） |
| G5 | #7 力矩 / #8 DQ电压 / #9 DQ电流 / #10 刹车 / #11 模式切换 | 共享“写模式寄存器+写目标寄存器”两原语 | **不同**（各自独立运行模式，向上暴露的功能接口） | **原子复用**，功能不重复 |
| G2 | #13 读状态 / #12 通用寄存器读 | 同一 0x17 命令族 | 特例 vs 泛化 | **特例关系**：`17 01` = `read_registers(&[addr:1,int16,count:3}])` |

### G2 特例的意义与问题

线上无独立意义；价值全在包装层：① 物理量换算+torque_coeff 补偿（decode_read_reply 带 coeff）；② 写 state 缓存（`motor.rs:322-324`）对接 `latest_state`/MotorDevice 被动生态（周期上报、ACK 序号）；③ 同步语义（ABI request_feedback / gateway state_once）。

**问题**：特例未建在通用读之上，而是平行实现——两个几乎相同的等待循环（`motor.rs:253-264` vs `315-334`）、两个解码器（decode_register_reply vs decode_read_reply）、硬编码帧而非调 encode_read。正确结构应为 `request_motor_feedback = read_registers(...) + 换算 + 写缓存`。

### 协议层归并后的真实命令集（~11 条）

`07 07`（普通）｜`07 35`（协同）｜`07 2d`（梯形）｜`0x18000|id`（MIT）｜`17 <cmd/addr>`（寄存器读）｜`01 00 <mode>`（模式写）｜`05 <reg>`（寄存器写）｜`05 B3`（保存）｜`05 B4`（周期上报）｜`40 01 …`（零位）｜`15 B5`（版本）

矩阵 24 行按此归并后：**功能行数不虚（每行独立功能），虚的是把功能缺口数当工作量**。

---

## 5. 其它厂商上位机数据解读模式（对照）

`build_state_snapshot`（`runtime.rs:156-298`）五分支对比：

| | Damiao | RobStride | MyActuator | Hexfellow | **HighTorque** |
|---|---|---|---|---|---|
| 链 vendor crate | ✅ | ✅ | ✅ | ✅ | ✅ [收敛] |
| Motor 句柄 | vendor 对象 | vendor 对象 | vendor 对象 | vendor 对象 | vendor 对象（Arc<HightorqueMotor>，此前裸 u16） |
| 状态数据 | 物理量+status_name | 物理量+12 故障布尔位 | 物理量（deg） | 物理量（rev/permille） | 物理量+status/fault/温度（此前 raw 三元组） |
| gateway 剩余换算 | 无 | mode→字符串 | deg→rad | rev→rad、permille→Nm | 无（仅 scan 发现仍 raw） |

寄存器/参数模式：帧编解码+verify 在 vendor，地址语义由客户端传参（Damiao rid 直通 `get_register_f32`，RobStride param_id 直通类型化读写，HighTorque `read_registers` 直通 addr/ty/count——2026-09-30 补通）；唯一 gateway 侧表知识是 Damiao scan 硬编码 rid 21/22/23，但型号识别仍调 vendor `match_models_by_limits`。

~~HighTorque 是五家中唯一"上位机内置换算逻辑"的，且已做错~~ **[收敛消灭，2026-09-29]**：与其余四家同构；仅 scan 发现（`17 01`/`27 01` 广播收发）保留 gateway 本地裸总线实现。

---

## 6. 修复优先级建议

1. **vendor 层补 2 原语**：encode_write（寄存器写）+ 真实模式切换帧 → 解锁力矩/DQ电压/DQ电流/刹车 4 行；~~顺手把 clear_error 接到停止帧（消除 G3 伪缺口）~~ **[已做，见 §8]**
2. ~~**ABI 补暴露**：梯形模式、read_registers（照 Damiao rid 直通模式）、固件版本、fault_code 进 MotorState → Python 自动受益；删除或实现 10 个幽灵 param 符号并对齐 capabilities JSON~~ **[已完成，2026-09-30，见 §10]**
3. ~~**ws_gateway 迁移**：照抄其它四家模式~~ **[已完成，commit c842d7d，2026-09-29]**：`MotorHandle::Hightorque` 换 `Arc<HightorqueMotor>`，状态走 latest_state，删 hightorque_ws.rs 129 行手写代码；enable/disable 假成功与 capabilities 虚报已修
4. ~~**CLI 迁移**：改为调用 vendor crate~~ **[已完成，commit c842d7d]**：控制/状态/参数全链走 vendor，帧格式分歧消除；scan 与诊断帧按设计保留裸发
5. **文档对齐**：README 的 force_pos/kp-kd 表述、CLI 的 "v1.5.5-compat" 标签、demo 双写 flash（set_zero_position 内部已调 store）
6. **测试**：CLI/ws_gateway 的 hightorque 路径当前零测试

---

## 7. 证据索引（关键 file:line，2026-09-30 收敛后）

- vendor 能力：`motor_vendors/hightorque/src/motor.rs`（clear_error:89=send_stop / state_seq:28 / wait_status:359 / mit / 梯形 / pos_vel 协同 / pos_classic / vel / store / send_with_ack / version / feedback / read_registers / send_control vs send_query）、`protocol.rs`（encode_mit_frame 已 pub 供上层复用）
- ABI 路由：`motor_abi/src/motor_control_ffi.rs`（hightorque_send_pos_vel_acc / hightorque_send_pos_classic）、`motor_register_ffi.rs`（hightorque_request_firmware_version / hightorque_get_register）、`state_ffi.rs`（MotorState.fault_code）、`lib.rs`（capabilities JSON 真实能力清单）；10 个幽灵 param 符号已随 `vendor_params/hightorque.rs`、`param_ffi/hightorque/` 删除（2026-09-30）
- CLI：`motor_cli/src/hightorque_cli.rs`（mit:299 走 vendor send_cmd_mit / diag_bus:245 诊断帧副本 / scan 裸总线收发 / 模式分发）、`Cargo.toml`（motor_vendor_hightorque 依赖）
- ws_gateway：`src/commands/scan.rs`（HtScanHit:352 / cmd_scan_hightorque:413，仅存的手写 HT 代码，只做发现）、`src/model.rs`（ControllerHandle::Hightorque 持有 vendor controller）、`src/session/connect.rs`（add_motor 注册）、`src/router/handlers/control.rs`（mit:93 / pos_vel:206 / vel:272 全走 vendor）、`src/router/handlers/register.rs`（clear_error 走 vendor）、`src/session/runtime.rs`（mit tick / build_state_snapshot 走 latest_state）；`src/vendors/hightorque_ws.rs` 已删除（commit c842d7d）
- Python：`bindings/python/src/motorbridge/abi.py`（4 个新 ctypes 声明）、`core.py`（hightorque_send_pos_vel_acc / hightorque_send_pos_classic / hightorque_request_firmware_version / hightorque_get_register）、`models.py`（MotorState.fault_code）、`examples/hightorque_all_interfaces_demo.py`
- 协议事实：表1（0x000 模式 R/W、0x1b Q电压、0x1c Q电流）、表2（模式 0=停止，清除错误、8=DQ电压、9=DQ电流、15=刹车）、协议 PDF §1.2.1（07 07 一帧两用）、§1.3.3（0x17 逐位=通用读）、参考固件 libelybot_can.c（DQ 命令=模式切换+寄存器写组合帧）

---

## 8. 修复记录（G3/G4，2026-09-28）

原则：以 ht_can v2.0.0 协议为准，去掉旧协议路径；编码只留 vendor 一份，上层复用而非抄写。

### G3：clear_error ≡ 停止帧（模式 0）

| 层 | 变更 | 文件 |
|---|---|---|
| vendor | `clear_error()` 由返回 `Err` 改为 `send_stop()`（同帧 `01 00 00`），新增单测 `clear_error_sends_stop_frame` | `motor_vendors/hightorque/src/motor.rs:65` |
| ABI / Python | 无代码变更：透传 vendor 结果，Err→Ok 自动生效 | — |
| ws_gateway | `clear_error` 的 Hightorque 分支由“直接拒绝”改为发 `01 00 00`（与 stop op 同帧） | `router/handlers/register.rs:52-56` |

> **G3 协议证据链（`01 00 <mode>` = 写模式寄存器 0x000）**：表1 寄存器 `0x000 模式 R/W`；表2 模式 0 名称即“停止，清除错误”（8=DQ电压、9=DQ电流、15=刹车）。参考库帧族第三字节恰为表2 模式号（停止 `01 00 00`、刹车 `01 00 0F`），DQ 例程一帧拼“模式写+寄存器写”`{01 00 08, 05 1b <volt>}`，证明 `01 00` 即模式写入专用命令（非通用寄存器写族 `05 <reg> <val>`）；状态帧 byte0 持续回读当前模式（持久状态量）。注意：协议 PDF §1.4 示例写 `0x8000|id` 扩展帧，与 v2.0.0 参考库（控制类裸 id 标准帧）矛盾，以参考库为准（vendor `send_control` 一致）。电机端固件源码不在工作区，“固件内部写 0x000”为外部证据推断。

### G4：MIT 改真 v2.0.0 位打包帧（`0x18000|id`，固件 v4.6.0+）

| 层 | 变更 | 文件 |
|---|---|---|
| vendor | `encode_mit_frame` 由 `pub(crate)` 改 `pub` 并从 lib.rs 导出（CLI/gateway 复用，避免第 4 份拷贝） | `motor_vendors/hightorque/src/protocol.rs:574` |
| CLI | `mit` 模式由发 `07 35` 改为发位打包帧（pos 16/vel 12/tqe 12/kp 12/kd 12，超界饱和）；新增 `send_ext_with_id`（任意 29 位 ID）与 `pos-deg`/`vel-deg-s` 参数换算；删除 kp/kd 忽略说明；`pos-vel-tqe` 入口保留（`07 35` 协同为合法模式，不再与 mit 同帧） | `motor_cli/src/hightorque_cli.rs` |
| ws_gateway | 新增 `send_hightorque_mit`（编码调 vendor）；`mit` 处理器与 runtime 连续 tick 均改走它，kp/kd 打包进帧；删除旧路径死代码 `pos_raw_from_rad`/`tqe_raw_from_tau` | `vendors/hightorque_ws.rs:65`、`router/handlers/control.rs:96-108`、`session/runtime.rs` |

### 文档对齐

- ws_gateway：PROTOCOL.zh-CN.md（§9.1 MIT 表、§10.1 clear_error 适用范围）、README.md / README.zh-CN.md（厂商映射表、模式参数差异）
- Python：README.md（send_mit 注释、模式参数表）、`cli/main.py` help（去掉 "v1.5.5 忽略 kp/kd"）
- CLI：args.rs help、README.md（"v1.5.5-compat" 标签清除）
- demo：`hightorque_all_interfaces_demo.py`（clear_error 语义注释：清错即停机）

### 验证

`cargo test -p motor_vendor_hightorque`（74 通过，含新增 clear_error 单测）、`cargo test -p motor_cli`（8 通过）、`cargo test -p ws_gateway`（6 通过）、`cargo check` 无警告。真机 MIT 路径需固件 v4.6.0+。

---

## 9. 修复记录（全链路收敛，2026-09-29/30）

三个 commit：`188e9d8`（vendor 修复）、`c842d7d`（CLI+ws_gateway 收敛）、`e81f0ec`（文档）。原则不变：编码只留 vendor 一份，上层复用而非抄写。

### vendor（188e9d8）

- **state_seq 状态缓存序号**：`wait_status` 以调用前的序号为基准，序号前进即视为回复已到达（与 ack_seq 同款模式）；`process_feedback_frame` 每次写缓存递增。修复：CoreController 后台收帧线程（PollingMode::Background）在同一总线抢 `bus.recv` 吃掉回帧时，阻塞式 `request_motor_feedback` 不再空转到假性超时。回归测试 `request_motor_feedback_ok_when_worker_consumes_reply`。
- MIT 常量 / `encode_mit_frame` 导出（见 §8 G4，收敛后 CLI/gateway 已不直接调用，改走 `send_cmd_mit`）。

### ws_gateway（c842d7d）

- 删除 `vendors/hightorque_ws.rs`（-129 行，含 `send_hightorque_mit`）；`ControllerHandle::Hightorque` 持有 `HightorqueController`、`MotorHandle::Hightorque` 持有 `Arc<HightorqueMotor>`，电机经 `add_motor` 注册。
- MIT/vel/pos_vel/停止/清错/零位/落盘/读状态全部经 vendor；pos_vel 从拒绝变为支持（与 ABI/Python 能力对齐）；enable/disable 诚实报错（enable 后于 2026-10-08 按方案 B 改回语义空操作，见行 23 与 §12）；`read`/`state_once`/`status`/`verify` 输出物理量（消灭力矩平移 ÷100、status_code 恒 0 两处错误）；scan 逻辑迁入 `commands/scan.rs` 自持。

### motor_cli（c842d7d）

- 控制/状态/参数（mit/pos/vel/pos-vel-tqe/stop/rezero/conf-write/read/ping）全链走 `HightorqueController`；`--model` 注册时校验并决定力矩系数；`raw-tqe` 拒绝（力矩统一物理量 `--tau`）；scan 发现与 `tqe`/`volt`/`cur`/`brake`/`timed-read` 诊断帧经 `diag_bus` 总线副本裸发。

### 文档（e81f0ec）

- 新增 [hightorque_can_protocol.md](hightorque_can_protocol.md)（逐帧协议参考）、本审计文档；Python demo 更新（clear_error 语义、state_seq 说明、`HT_EXAMPLE_WRITE_CONFIG` 跳过闪存写入）；devices/cli/各 README 的 "v2.0.0 迁移中" 措辞清除。

### 验证

`cargo test -p motor_vendor_hightorque`（78 通过，含 state_seq 回归测试）、`cargo test -p motor_cli`（8 通过）、`cargo test -p ws_gateway`（6 通过）。

### 遗留（A 类补全后仍开放）

- ~~§3 A 类 vendor 原语缺口（寄存器写 encode_write、模式切换帧、周期上报、力矩/DQ/刹车）不变——收敛与 §10 补通均只做“vendor 已有能力向上接通”，不新增 vendor 原语。~~ **[2026-09-30 A 类清零，见 §11]**：全部 4 原语落地。B 类 ABI 缺口此前已清零（`send_with_ack` 为有意不接、`torque_coeff` 为内部补偿系数）。
- scan 的 `17 01`/`27 01` 收发在 CLI 与 gateway 仍是两份小拷贝（vendor 无 scan API，属可接受重复）。
- CLI scan 强制 send_count=1、忽略 `--loop`（§3 C8 残留）。
- RunMode 字符串解码（#20）与 send_with_ack（#22）为有意不接（见 §10）。
- **`apply_bulk_device_op` 首错短路 → 批量操作部分应用**（2026-10-08 诊断，未修；motor_core `controller.rs:139`，跨厂商问题非 HT 专属）：循环内 `op(device)?` 遇单台 Err 即 return，该 id 之后的电机连尝试都没有。直接入口 `enable_all`/`disable_all`（Rust 五家 vendor controller / Python / ABI 全部汇入此一处循环），**间接入口 `Controller::shutdown()` 清尾 `disable_all`**（`close_inner` `let _` 吞错，但"失败点之后的电机没收到停止帧"行为原样保留——含 Python `with` 块退出路径）。触发与传输无关：RobStride/Hexfellow enable 逐台 SDO 确认超时（电机掉电即 Err，socketcan 下同样触发）是最高频源；修复前 HT enable Err 为混总线**必然**触发源（§12 方案 B 已拆该源，但循环缺陷本身仍在）。修复方向（参照同文件后台轮询线程 :220-227 的既有容错写法）：`Unsupported` 静默跳过（能力差异非故障）、其余错误记录 id 继续、循环结束聚合报错；全部成功路径行为不变，唯一可见差异是错误返回从"第一个错误原样"变为含 id 的聚合信息（项目内无人做错误文本匹配）。配套单测：三台假电机、中间台故意报错、断言第三台仍被调用。

---

## 10. 修复记录（vendor 能力补通，2026-09-30）

目标：补全“vendor ✅ 而上层 ❌”的部分（§3 B 类），不新增 vendor 原语。五项功能 + 一项数据字段：

| # | vendor 原语 | C ABI | Python | CLI | ws_gateway |
|---|---|---|---|---|---|
| F1 梯形 07 2d | `send_cmd_pos_vel_acc`（已有） | `motor_handle_hightorque_send_pos_vel_acc` | `hightorque_send_pos_vel_acc` | `--mode pos-vel-acc` | `pos_vel_acc` op（ActiveCommand::PosVelAcc，continuous tick 复用） |
| F2 经典位置 07 07 | `send_cmd_pos_classic`（已有） | `motor_handle_hightorque_send_pos_classic`（NaN=无限制） | `hightorque_send_pos_classic`（tqe=None） | `--mode pos`（已有） | `pos_classic` op（一次性，tqe 可缺省） |
| F3 固件版本 15 B5 02 | `request_firmware_version`（已有） | `motor_handle_hightorque_request_firmware_version`（out major/minor/patch） | `hightorque_request_firmware_version` | `--mode version` | `version` op（解除 myactuator-only） |
| F4 寄存器读 17 族 | `read_registers`（已有） | `motor_handle_hightorque_get_register`（addr/ty/count → raw f32 数组） | `hightorque_get_register` | `--mode read-reg`（--addr/--count/--ty，名称取 PARAMETER_TABLE） | `read_register` op（ty 缺省回落寄存器表，无则要求显式） |
| F5 fault_code | `HightorqueFeedbackState.fault_code`（已有） | `MotorState.fault_code`（结构体末尾追加，旧布局不变；其余厂商恒 0） | `MotorState.fault_code` | 已有 | 已有 |

**有意不接**：`send_with_ack`（#22，裸帧+ACK 原语无独立用户语义，上层应调语义化命令）；RunMode 字符串解码（#20，显示层美化，status_code 原始字节已透传，对照表见 [hightorque_can_protocol.md](hightorque_can_protocol.md)）。

附带清理：删除 10 个幽灵 `motor_handle_hightorque_{get,write}_param_*` stub（`vendor_params/hightorque.rs`、`param_ffi/hightorque/` 整体移除）；capabilities JSON 的 `hightorque` 数组改为真实能力清单；头文件 `motor_abi.h` 同步 4 个新函数声明与 fault_code 字段。

### 验证

`cargo test -p motor_abi`（12 通过，含 07 2d 帧/NaN 哨兵/版本解码/寄存器读/fault_code 新增单测）、`cargo test -p motor_cli`（12 通过，含 read-reg --ty 解析 4 个新单测）、`cargo test -p ws_gateway`（6 通过）、`cargo test -p motor_vendor_hightorque`（78 通过）；Python `py_compile` + ctypes 结构体冒烟（CState 40 字节，fault_code 往返）。真机梯形/固件版本路径需固件 v4.6.0+。

---

## 11. 修复记录（A 类 vendor 原语补全，2026-09-30）

目标：补全“vendor ❌”的 §3 A 类缺口（矩阵 #7-11/14/15 七行），新增 vendor 原语并四层级联。用户决定：CLI tqe/volt/cur/brake 从 raw 诊断帧改为 vendor 物理量调用（删 `--raw-tqe/--raw-vol/--raw-cur`）；顺带补 #14 寄存器写与 #15 周期上报。

| # | vendor 原语（新增） | C ABI | Python | CLI | ws_gateway |
|---|---|---|---|---|---|
| A7 力矩 `05 13` | `send_cmd_torque(nm)`（复用 `tqe_adjust_to_raw` 补偿） | `motor_handle_hightorque_send_torque` | `hightorque_send_torque` | `--mode tqe --tau Nm` | `torque` op（ActiveCommand::Torque，continuous tick 复用） |
| A8 DQ 电压 `01 00 08 05 1B` | `send_cmd_dq_voltage(v)`（拼一帧，0.1 V/LSB） | `motor_handle_hightorque_send_dq_voltage` | `hightorque_send_dq_voltage` | `--mode volt --volt V` | `dq_voltage` op |
| A9 DQ 电流 `01 00 09 05 1C` | `send_cmd_dq_current(a)`（0.1 A/LSB） | `motor_handle_hightorque_send_dq_current` | `hightorque_send_dq_current` | `--mode cur --cur A` | `dq_current` op |
| A10 刹车 `01 00 0F` | `brake()` | `motor_handle_hightorque_brake` | `hightorque_brake` | `--mode brake` | `brake` op |
| A11 模式切换 `01 00 <mode>` | `send_run_mode(mode)`（0-17 校验+真发帧） | `motor_handle_hightorque_set_run_mode` | `hightorque_set_run_mode` | `--mode set-mode --run-mode N` | `run_mode` op |
| A14 寄存器写 `05 <reg> <val16>` | `write_register(addr, val, config_class)`（控制类走 send_control，配置类恒扩展帧） | `motor_handle_hightorque_write_register` | `hightorque_write_register` | `--mode write-reg --addr --value [--config]` | `write_register` op |
| A15 周期上报 `05 B4 02 <t>` | `set_periodic_report(period_ms)`（0=停止） | `motor_handle_hightorque_set_periodic_report` | `hightorque_set_periodic_report` | `--mode periodic-report --period-ms`（timed-read 保留别名） | `periodic_report` op |

设计要点：

- `ensure_control_mode`（MotorDevice trait 契约、ABI ensure_mode 依赖）**保持仅校验语义不变**；真发帧走独立的新原语 `send_run_mode`。
- DQ 电压/电流的模式写+寄存器写按协议拼一帧（7 字节），非两帧先后发送；协议只定义 Q 分量，无 D 寄存器入参。
- `write_register` 的帧类由调用方按寄存器用途指定：控制目标类（0x13/0x1B/0x1C 等热路径）走控制通道（默认标准 11 位裸 id；`control_reply` 开关仅诊断用，见 §13），配置类恒走扩展 `0x8000|id` 帧。
- CLI 裸总线路径（`diag_bus`/`send_diag_ext`）整体删除——五类命令全走 vendor 后，HT 裸总线代码只剩 scan（vendor 无 scan API）。
- gateway capabilities：modes 增 `torque`；ops_vendor_native 增 dq_voltage/dq_current/brake/run_mode/write_register/periodic_report。ABI capabilities JSON 的 `hightorque` 数组同步 7 项。

### 验证

`cargo test -p motor_vendor_hightorque`（86 通过，+8 帧字节级新单测：力矩 1.1Nm→`05 13 DC 00`、DQ 电压 1.0V、DQ 电流 0.5A、brake、run_mode 5/17/18 拒绝、寄存器写两种帧型、周期上报开/停）、`cargo test -p motor_abi`（19 通过，+7 MockBus 单测）、`cargo test -p motor_cli`（12 通过）、`cargo test -p ws_gateway`（6 通过）；`cargo check --workspace` 零警告；release .so 冒烟（7 个新符号导出 + capabilities JSON 含 7 项新能力）+ Python `py_compile`。真机力矩/DQ/模式切换路径需实机验证（`set-mode --run-mode 0` 停止清错最安全）。

---

## 12. enable 方案 B：语义空操作（2026-10-08）

**决策**（用户裁定，三案选 B）：`HightorqueMotor::enable()` 返回 `Ok(())`，**不发送任何帧**。A 案（enable≡`01 00 00` 复位帧）被否——enable 不应借机清错/停机，保持零副作用；C 案（软使能状态机 gate 控制命令）被否——破坏 HT"命令即执行"语义且 host 标志与电机真实状态易 desync。

**动机**（替换 2026-09-29 收敛的"透传 Err"）：
- core `apply_bulk_device_op` 遇错即中止（`op(device)?` 短路）——混总线上 HT 电机的 enable Err 使 `enable_all()` 中断，**排在后面的电机永远使能不到**；
- 统一 enable→command→disable 流程（gateway enable op、通用脚本）要求 enable 可调用；
- enable 契约的 postcondition"后续控制命令会被执行"对 HT **恒成立**（协议无 separate 使能帧，控制命令 cmd 字节即决定行为），no-op 是该 postcondition 的忠实实现。

**与 2026-09-29"删假成功 no-op"收敛的关系**：那次删的是**无文档的假成功**（gateway 手写层不知情地返回成功）；本次是**有意的语义映射**，带动机、边界与单测锁定——区别不在行为而在设计知情权。

**边界**：清错/停机仍由 `clear_error()`/`disable()` 的 `01 00 00` 显式完成；enable 不顺带任何总线动作。latched fault 场景请显式调 `clear_error`。

**改动**：vendor `motor.rs` enable 实现（trait impl 委托固有方法自动生效）+ 单测 `enable_is_frameless_noop`（Ok 且 MockBus 零帧）；ABI/gateway/CLI 代码零改动（透传路径自然变绿）；demo `_switch_mode` 补 `motor.enable()` 调用对齐统一流程；文档同步（本节、行 23、§3 第 4 条、PROTOCOL.zh-CN.md enable 表、gateway README×2、demo docstring）。

**验证**：`cargo test -p motor_vendor_hightorque`（+1 至 87）等四 crate 全绿 + `cargo check --workspace` 零警告；详见当日验证记录。

---

## 13. send_control 退回标准 11 位裸 id 帧（2026-10-09）

**背景**：2026-09-29 曾把 `send_control` 默认改为扩展 29 位 `0x8000|id` + bit15（动机：预期 v2.0.0 固件对每个控制帧回发 §3.2 状态反馈帧，控制流期间 `get_state` 持续新鲜）。

**证伪**（用户 candump 真机实测）：

```
can0  00008001   [4]  05 13 1C 00    ← 发送：扩展帧 0x8001（bit15 置位）+ 力矩指令
can0       100   [0]                  ← 回复：标准帧 id<<8，DLC=0 空帧，仅此一帧
```

bit15 置于非查询指令，电机只回一帧数据段长度为 0 的帧——与协议 PDF §1.2 原文完全一致（"置 1 但不发查询指令时，电机会返回一帧数据段长度为 0 的帧"），**没有任何状态数据**。§3.2 状态反馈帧与"控制命令附带"无关，产生路径只有 `17 01` 查询与 `05 B4` 周期上报两条（与 2026-10-08"运动段读数冻结"的观察互相印证——控制流本身确实不带回帧）。

**改动**：
- vendor `motor.rs`：`control_reply_enabled` 默认 true→false——`send_control` 默认标准 11 位裸 id，对齐 §10 参考固件 `can_send(hfdcanx, id, ...)`；`set_control_reply(true)` 保留为诊断/复现开关（vendor 层，未接 ABI/Python）。字段/方法/`send_control` 文档注释改写并记录证伪证据。
- 单测：默认行为测试翻转为 `control_commands_default_to_standard_bare_id_frames`；原 legacy 测试翻转为 `control_commands_reply_mode_uses_extended_frames`（两分支均锁定）；clear_error/pos_classic/torque/run_mode 断言同步翻转；`write_register_both_frame_classes` 改为 reply 开 + config 恒扩展（双向锁定"config 类不随开关切换帧型"）。ABI `hightorque_send_pos_vel_acc` 测试断言翻转。查询/配置路径（17 01、15 B5、B4、B3、0x40）帧型不变，相关测试零改动。
- 文档：`hightorque_can_protocol.md` 七处更正——§1.2 bit15 行补真机证实、§1.3 退回记录（原 2026-09-29 更新注记重写）、通信命令 1/4"应答帧"行、§3.2 标题与导语（改为"周期上报/状态编码格式"+更正注）、§8.3 状态刷新两条路、§9 导语。CLI/gateway/Python README 中 `--config`/`config_class` 的扩展帧描述针对配置类，本就正确，未动。

**影响面**：控制类帧（07 07 / 07 35 / 07 2d / 05 13 / 01 00 族 / DQ）仲裁 ID 从扩展 `0x8000|id` 变为裸 id 标准 11 位——与官方参考固件逐帧一致；消除每控制帧一个 DLC=0 空帧的总线开销（100Hz 控制流 = 每秒 100 个废帧）。接收路径（`accepts_frame` 判 `id<<8`）与状态刷新机制（周期上报/主动查询）不受影响。

**验证**：`cargo test -p motor_vendor_hightorque`（87 通过）+ `-p motor_abi`（19）+ `-p motor_cli`（12）+ `-p ws_gateway`（6）全绿。
