# MYCode_v4 开发代理说明

本文件供后续接手本工程的 AI/自动化开发代理使用。修改前必须先阅读本文件，并以当前工程源码、`empty.syscfg` 和 SysConfig 生成宏为准，不得根据其他模板猜测引脚、外设实例或 DriverLib API。

## 1. 工程定位

- 工程路径：`D:/CCS/workspace_v12/MYCode_v4`。
- MCU：TI MSPM0G3507，天猛星开发板。
- IDE/构建：Code Composer Studio 12.8.1，TI ArmClang 4.0.0.LTS。
- SDK：`D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04`。
- SysConfig：`D:/ti/CCS_SDK/sysconfig_1.26.2`。
- 当前入口：`empty.c`。
- 当前主要功能：八路数字灰度巡线、PD 差速控制、直角弯 pivot、OLED 菜单调参、Flash 参数保存。
- 当前运行模式是纯巡线；`empty.c` 保存参数时将 MPU 模式写为 0。除非用户明确要求，不要擅自把 MPU6050 yaw 接入巡线控制。

## 2. 文件职责

- `empty.c`：模式、按键、OLED 菜单和巡线主循环。不要把底层驱动或复杂控制算法堆到这里。
- `empty.syscfg`：时钟、引脚、PWM、I2C、UART、Timer 和中断配置的唯一源文件。
- `BSP/src/line.c`：八路灰度读取、巡线 PD、丢线处理和直角弯状态机。
- `BSP/src/bsp_tb6612.c`：TB6612FNG 电机底层驱动。
- `BSP/src/encoder.c`：左右编码器计数。
- `BSP/src/mpu6050.c`：MPU6050 I2C 驱动和姿态解算；修改直角巡线时不要顺带改姿态解算。
- `BSP/src/param_store.c`：Flash 参数保存与读取。
- `Debug/ti_msp_dl_config.c`、`Debug/ti_msp_dl_config.h`、`Debug/device_linker.cmd`：SysConfig 生成文件，只能重新生成，禁止手工修改。
- `Debug/*.o`、`Debug/*.out`、`Debug/*.map`：构建产物，禁止作为源代码修改。

## 3. 当前硬件映射

所有宏名最终必须以 `Debug/ti_msp_dl_config.h` 为准。

| 功能 | 器件/模块 | MSPM0G3507 引脚 | 当前外设/用途 |
|---|---|---:|---|
| 右轮 PWM | TB6612 A/PWMA | PB15 | TIMG7 CCP0 |
| 右轮方向 | TB6612 AIN1/AIN2 | PA13/PA12 | GPIO |
| 左轮 PWM | TB6612 B/PWMB | PB16 | TIMG7 CCP1 |
| 左轮方向 | TB6612 BIN1/BIN2 | PB0/PB1 | GPIO |
| 灰度 L1 | 八路数字灰度 | PB25 | GPIO |
| 灰度 L2 | 八路数字灰度 | PB24 | GPIO |
| 灰度 L3 | 八路数字灰度 | PB20 | GPIO |
| 灰度 L4 | 八路数字灰度 | PA14 | GPIO |
| 灰度 R1 | 八路数字灰度 | PB18 | GPIO |
| 灰度 R2 | 八路数字灰度 | PB19 | GPIO |
| 灰度 R3 | 八路数字灰度 | PB10 | GPIO |
| 灰度 R4 | 八路数字灰度 | PA7 | GPIO |
| OLED | SSD1306 | PA28/PA31 | I2C0 SDA/SCL，当前 100 kHz |
| MPU6050 | MPU6050/兼容 MPU6500 | PA10/PA11 | I2C1 SDA/SCL，当前 100 kHz |
| K230 通信 | UART3 | PB2/PB3 | TX/RX，115200 8N1 |
| 左编码器 | MG310 | PA17/PA24 | A/B 相 |
| 右编码器 | MG310 | PA15/PA16 | A/B 相 |
| 舵机 PWM（保留） | TIMA0 | PB8/PB9 | CCP0/CCP1 |
| SWD | 调试接口 | PA19/PA20 | SWDIO/SWCLK，禁止占用 |

注意：当前 `Line_ReadSensors()` 按“GPIO 高电平表示检测到黑线”生成内部位图。不要仅凭其他灰度模块的常见电平逻辑将其反相；若实物显示相反，应先在 OLED/调试页确认八路原始电平，再统一修改并实车验证。

## 4. TB6612 与左右轮规则

- 当前硬件按 A 通道接右轮、B 通道接左轮。
- `bsp_tb6612.c` 中 A/B 通道映射不可凭函数名猜测，修改前同时核对 `AO_Control()`、`BO_Control()` 和实车旋转方向。
- `line.c` 的 `Car_SetMotors()` 现有调用约定必须保持一致：正值表示该逻辑通道前进，负值表示后退。
- 当前直角右转命令为 `Car_SetMotors(-reverse, fast)`，左转命令为 `Car_SetMotors(fast, -reverse)`。如果实车方向相反，先确认左右轮接线和 A/B 映射，不要通过反转灰度误差符号掩盖硬件映射问题。
- 第一次测试电机方向、直角转弯或高 PWM 时必须架空车轮。

## 5. 巡线与直角弯逻辑

八路传感器顺序固定为：

```text
LEFT1 LEFT2 LEFT3 LEFT4 RIGHT1 RIGHT2 RIGHT3 RIGHT4
  -4    -3    -2    -1      1      2      3      4
```

正常巡线：

```text
传感器位图 -> 加权位置 -> 中心死区 -> EMA -> PD -> 左右差速
```

当前直角弯状态机位于 `BSP/src/line.c`，必须保持以下顺序：

```text
最外侧传感器连续确认
        -> 记录转向方向
        -> 内侧轮小幅反转、外侧轮前进
        -> 确认离开旧线路
        -> 等待 PivTime 最小转向时间
        -> 中央 LEFT4/RIGHT1 连续重捕获
        -> 退出 pivot 并保留最近转向方向
```

关键参数：

- `PIVOT_ENTRY_CONFIRM_MS`：入口去抖，当前 2 次主循环。
- `PIVOT_EXIT_CONFIRM_MS`：中心重捕获确认，当前 5 次主循环。
- `PIVOT_CENTER_MASK`：当前 `0x18`，对应 LEFT4 和 RIGHT1。
- `PIVOT_TIMEOUT_MS`：异常超时保护，当前 900 次主循环。
- `PIVOT_REVERSE_MIN/MAX`：内侧轮反转力度。
- 菜单 `PivTime`：直角弯最小保持时间，当前默认 100，实车建议从 120～160 调试。

禁止恢复以下旧问题：

- 不得在检测到中心传感器的同一次调用里立即退出 pivot。
- 不得定义 `pivot_timer` 后不递增或不使用。
- 左右最外侧同时触发时不得固定优先左转或右转，应结合候选方向、左右半区数量和最近有效方向消歧。
- pivot 结束后不得把最后方向无条件清零，否则短暂丢线会向错误方向搜索。
- 上电尚未获得任何有效灰度样本时，不得默认向右旋转搜索。

## 6. 参数调整原则

- 先确认传感器位图和左右轮方向，再调 PD；硬件方向错误不能靠 PID 修正。
- 直道摆动：先降低 `KP`，再适当提高 `KD`。
- 入弯迟钝：提高 `KP` 或降低滤波强度，不要首先增大基础速度。
- 直角转不够：增加菜单 `PivTime`，或小幅增加 pivot 快轮/反转轮力度。
- 直角转过头：减少 `PivTime` 或减小反转力度。
- 直角误触发：把 `PIVOT_ENTRY_CONFIRM_MS` 从 2 增加到 3～5。
- 已经压中新线但退出慢：把 `PIVOT_EXIT_CONFIRM_MS` 从 5 减少到 3，但不得取消稳定确认。
- 调试顺序：低速架空测试 -> 地面低速单弯测试 -> 连续左右直角 -> 最后提高基础速度。

## 7. 编码与 DriverLib 规则

- 使用当前 MSPM0 SDK 2.10.00.04 中真实存在的 API，新增 API 前必须在本机 SDK 头文件中检索确认。
- Timer 类型必须匹配：TIMA0 使用 `DL_TimerA_*`，TIMGx 使用 `DL_TimerG_*`。
- 不要手工猜 SysConfig 宏名；读取 `Debug/ti_msp_dl_config.h`。
- 中断中只做计数、搬运或置标志，不执行 OLED、Flash、浮点姿态解算或长延时。
- 保留现有中文注释，并说明控制逻辑的原因，而不只描述代码表面行为。
- 修改范围应尽可能小，保留用户现有菜单、参数、Flash 数据布局和无关模块。

## 8. 修改与验证流程

每次修改 MSPM0 源码后至少完成以下检查：

1. 检查 `empty.syscfg`、`Debug/ti_msp_dl_config.h` 和代码宏名是否一致。
2. 如果修改了外设、引脚、时钟或中断，运行 SysConfig：

```powershell
& 'D:\ti\CCS_SDK\sysconfig_1.26.2\sysconfig_cli.bat' `
  --script 'D:\CCS\workspace_v12\MYCode_v4\empty.syscfg' `
  -o 'D:\CCS\workspace_v12\MYCode_v4\Debug' `
  -s 'D:\ti\CCS_SDK\mspm0_sdk_2_10_00_04\.metadata\product.json' `
  --compiler ticlang
```

3. 在 `Debug` 目录编译并链接：

```powershell
& 'D:\ti\ccs1281\ccs\utils\bin\gmake.exe' -j4 all
```

4. 只有 SysConfig 无错误、TI ArmClang 编译无错误、链接成功后，才能说明“源码和构建验证通过”。
5. 未连接实车时必须明确写明“未完成硬件验证”，不得把编译通过描述为直角弯实车已经可靠。

SysConfig 关于 Flash 状态位和 STOP/STANDBY 寄存器保持的 `info` 提示应单独报告，但它们不等于生成失败。

## 9. 禁止事项

- 禁止直接编辑 `Debug/ti_msp_dl_config.c/h`、链接脚本或构建产物。
- 禁止擅自更换 MCU、SDK、编译器、CCS 版本、调试器或板卡型号。
- 禁止擅自占用 PA19/PA20；PA10/PA11 已用于 MPU6050 I2C1，也不得同时配置成调试 UART。
- 禁止在没有确认实物接线时改变 TB6612 A/B 通道、编码器左右轮或灰度顺序。
- 禁止为修复直角转弯而改动 MPU6050 姿态解算、K230 通信、云台或 OLED 底层。
- 禁止执行 `git reset --hard`、覆盖用户无关改动或删除整个 `Debug`/工程目录。
- 禁止未经用户明确要求直接烧录或启动电机。

## 10. 当前验证状态

- 2026-07-22：直角弯状态机已优化。
- SysConfig 1.26.2 生成通过，无引脚冲突。
- `BSP/src/line.c` 已由 TI ArmClang 4.0.0.LTS 编译通过。
- 工程已链接生成 `Debug/MYCode_v4.out`。
- 尚未完成实车直角弯验证；后续调参必须以真实灰度位图、转向方向和赛道表现为准。

## 11. MPU6050 相对航向与操作规则（2026-07-22）

- MPU6050 接在 I2C1：PA10=SDA、PA11=SCL、地址 0x68；当前安装按参考工程处理为车体水平放置、Z 轴垂直车体平面，偏航使用 `gyro_z`。若实测平放旋转时主要变化的不是 `gyro_z`，必须先确认模块实际安装方向，不能靠 PID 符号掩盖轴向错误。
- 上电会初始化 MPU6050，并在车辆静止时采集 200 点陀螺零偏。校准阶段必须保持车辆和传感器完全静止；初始化失败时巡线自动退回纯八路灰度模式。
- 姿态解算禁止固定假设 `dt=0.001s`。`Board_TimebaseInit()` 建立 1ms SysTick，`MPU6050_QuaternionUpdate()` 使用相邻更新的真实毫秒差积分，并把异常暂停后的单步周期限制在 50ms。
- 当前 I2C1 控制器接收 FIFO 为8字节，禁止在事务结束后一次读取14字节数据；这会在校准阶段卡住。必须沿用 `Test_MPU6050` 已验证方式：加速度6字节、陀螺仪6字节、温度2字节分段读取。姿态更新后的角速度仍由 `MPU6050_GetLatestData()` 复用，巡线层不要额外重复读取。
- 六轴 MPU6050 没有磁力计，yaw 是短期相对航向，不是绝对航向，长期必然会缓慢漂移。禁止用 `atan2(accel_y, accel_x)` 修正 yaw；加速度计只能约束 roll/pitch。
- 首页按 K4 进入巡线菜单，第一页固定为 MPU 状态页。状态为 `MPU ON`、`MPU OFF` 或 `MPU FAIL`；在此页短按 K2 切换 ON/OFF，状态保存到 Flash。
- `MPU OFF`：`Line_Run()` 完全使用八路灰度位置 PD、丢线搜索和直角 pivot 状态机。
- `MPU ON`：灰度 PD 仍为主控制；直道使用小权重相对 yaw 辅助，普通弯使用 `gyro_z` 阻尼，直角 pivot 阶段释放 yaw 锁定，禁止 MPU 与 pivot 抢控制权。
- 首页短按 K3 进入 MPU 测试页。测试页不运行巡线、不驱动电机，只实时显示相对 `Yaw`；短按 K3 归零，短按 K4 返回首页。
- 每次开启 MPU 辅助、开始巡线或进入测试页都会重置相对 yaw 和时间基准，避免把菜单等待期间的漂移或暂停时间带入控制。
- 本规则的源码修改完成后必须重新运行 SysConfig CLI 与 CCS `gmake -j4 all`。编译通过只代表软件构建验证，90°角度比例、正负方向和实车辅助效果仍必须在硬件上验证。
- 2026-07-22 本次验证：SysConfig 1.26.2 生成成功；`empty.c`、`Board/board.c`、`BSP/src/mpu6050.c`、`BSP/src/line.c` 均由 TI ArmClang 4.0.0.LTS 编译通过；工程链接成功并生成 `Debug/MYCode_v4.out`。尚未完成 MPU 90°比例、方向、静态漂移和巡线辅助的实车验证。
- MPU 上电初始化必须复用 `Test_MPU6050` 的时序：OLED 初始化后至少等待 200ms 再访问 I2C1。当前 `MPU_BootDiagnosticInit()` 等待250ms并重试3次；成功时不再显示多页总线诊断，失败页只保留简洁的 WHO_AM_I 和返回值。
- 上电OLED只保留一页 `MPU INIT / KEEP CAR STILL` 提醒；初始化成功后直接进入首页，失败时短暂显示 `MPU FAIL`，随后仍允许纯灰度巡线。
- 2026-07-22 实测连续手动旋转中，初始角、逆时针90°、顺时针回转读数依次出现 `0, 5.2, -0.6, 7.5, -0.4, 5.6, -2.4, 3.3, -3.5, 0.6`。方向基本正确但比例严重偏小，根因是测试页每轮 `OLED_Clear()+OLED_Refresh()` 产生两次100kHz整屏I2C刷新，而姿态 `dt` 又限幅50ms，显示阻塞期间的旋转时间被丢弃。
- MPU测试页必须保持“高频姿态采样、低频局部显示”：姿态循环约2ms运行，OLED每100ms只刷新Yaw数字所在的第3~4页。禁止在姿态采样循环内调用 `OLED_Clear()` 或每轮整屏刷新。
- OLED刷新采用不超过当前8字节TX FIFO的批量事务：1字节控制字节+最多7字节显存数据。`OLED_RefreshPages()` 用于局部刷新，禁止恢复每个显存字节单独发起一次I2C事务的旧实现。
- v5的USB-TTL调试输出固定使用 `UART3_TX=PB2`，115200 baud、8数据位、无校验、1停止位。接线为 `PB2 -> USB-TTL RX`，并且必须连接两端GND；USB-TTL应使用3.3V逻辑。
- UART3实例由SysConfig生成为 `UART Main`，发送底层使用 `DL_UART_Main_transmitDataBlocking()`。禁止混用不明确的通用非阻塞发送后立即连续写入。
- 首页按K3进入MPU测试页后，以50Hz发送一列纯ASCII数字帧，例如 `90.0\r\n` 或 `-5.6\r\n`。为兼容逐飞助手的文本曲线解析，禁止发送启动握手、`Y,` 字段名、正号、单位或其他非数字说明；负号和小数点除外。
