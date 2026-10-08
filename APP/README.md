# APP —— 应用层

本目录放**业务逻辑**，是唯一允许写「这个设备要干什么」的地方。

## 现状

- `main.c` / `main.h` —— 从 `framework/Core/Src/` 迁入。初始化调用链与主循环，
  另外带一个 CAN 帧日志旁路（`CO_CANrxCaptureHook` + `can_log_dump()`），
  每秒把收到的帧打到串口，方便对着 PCAN-View 之类工具交叉验证。
- **`cia402/`** —— CiA 402 驱动 profile 的使能状态机 + 运动阶段，见下。

主循环里每轮依次调两个函数，顺序有意义：

```c
canopen_app_process();   /* 协议栈非实时部分：NMT / SDO 服务器 / 心跳 */
motor_ctrl_process();    /* 应用侧状态机：内部再分发到 co_init / cia402 */
test_mode_switch(tick);  /* 测试脚本，见 test/test.h */
```

SDO 事务不再需要单独的 `co_master_poll()`：推进、归属判断、取结果都收在
`co_master_step_gate()` 里，由各个状态机在自己的 tick 中调用（见
`framework/canopen/canopen_master.h` 的说明）。

### `cia402/`

对 DM556-CAN 这类 CiA 402 步进/伺服驱动器，用 NMT + SDO 把它推到
**Operation Enabled**，再按脚本下发运动指令让它真的转起来。两步：

1. **链路自检**（使能之前）：读 0x1000 设备类型 + 0x2001 每转脉冲。
   一眼看出 SDO 通道 / 节点号 / 波特率对不对，不用等使能超时。
2. **使能**：「读 0x6041 状态字 → 解码 → 选控制字 → 写 0x6040 → 再读」的闭环，
   不是盲发一串控制字。驱动器中途报警或掉使能都能自己纠正。
3. **运动**：先走一段**有界**的位置（相对 1 圈），再切速度模式连续转。
   写成一张脚本表 `s_motion[]` + 一个小解释器，加一条改一条都只动表。

- 目标节点号在 `cia402.h` 的 `CIA402_DEFAULT_NODE_ID`，必须和驱动器
  SW1~SW5 拨码一致；波特率两边都要 500 kbit/s（驱动器 SW6=on / SW7=off）。
- 要调运动参数只改 `cia402.h` 里那一段 `CIA402_*_MILLI_RPS` 宏，
  单位是「千分之一圈/秒」，不用浮点（本工程挂了 `-specs=nano.specs`，
  printf 的 `%f` 默认不可用）。
- **`0x60FF` 的单位手册没写**（只写了 `0x606C` 是 p/s），所以下发后会读
  `0x606C` 对账，实测超目标 3 倍或一直为 0 就当场停车。详见
  `Doc/CiA402主站实现.md` 10.4。
- 全程非阻塞。SDO 的收发在 CAN 中断里完成，但状态推进要靠主循环反复调用，
  所以这里不能写阻塞等待 —— 一阻塞主循环，`canopen_app_process()` 就停了。

> ⚠️ **速度模式下调试器 halt 住 CPU，电机照样转。** `0x60FF` 是持续指令，
> 主站停了驱动器不会停。默认的 15 秒有界运行是 halt 之后唯一还能自动停车的东西。

## 后续要放什么

- `cia402/` 往上长：回零模式（0x6060=6 / 0x6098）、
  RPDO/TPDO 周期映射，替代现在每步一次 SDO 的做法。
- **`ros/`** —— ROS 接口引出，把运动指令/状态通过 ROS 消息或串口协议暴露出去。
- 多轴管理逻辑：轴号分配、NMT 状态编排、心跳超时处理。

**除 `cia402/` 的使能与运动外，其余尚未实现。**

## 分层原则

- 应用层只调用 `framework/canopen` 暴露的对象字典与 PDO 接口，
  不直接调 HAL，也不直接操作 CAN 外设
- 具体硬件差异封装在 `framework/BSP`，应用层不感知

SDO 客户端事务这层「发起 → 轮询 → 取结果」的通用主站能力属于协议栈，
不属于业务，所以放在 `framework/canopen/canopen_master.c`，
应用层通过 `canopen_master.h` 调用，不直接碰 CANopenNode 内部。
