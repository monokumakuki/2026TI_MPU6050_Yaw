# MYCode_v6_MPUMode 开发说明

本文供后续接手本工程的开发者或 AI 使用。修改前必须先检查本文件、当前源码、`empty.syscfg` 和 SysConfig 生成宏，不得根据其他工程猜测引脚或外设实例。

## 1. 工程信息

- 工程路径：`D:/CCS/workspace_v12/MYCode_v6_MPUMode`
- MCU：TI MSPM0G3507
- IDE：Code Composer Studio 12.8.1
- SDK：`D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04`
- SysConfig：`D:/ti/CCS_SDK/sysconfig_1.26.2`
- 主入口：`empty.c`
- 主要功能：八路数字灰度巡线、TB6612FNG 差速控制、直角弯 pivot、可选 MPU6050 辅助、OLED 菜单、Flash 参数保存、MPU yaw 测试串口输出。

## 2. 硬件映射

以下映射已经由现有工程和用户实物确认，不得擅自修改。

| 功能 | 器件 | MSPM0G3507 引脚 | 说明 |
|---|---|---|---|
| 右轮 PWM | TB6612 A/PWMA | PB15 | A 通道为右轮 |
| 右轮方向 | TB6612 AIN1/AIN2 | PA13/PA12 | GPIO |
| 左轮 PWM | TB6612 B/PWMB | PB16 | B 通道为左轮 |
| 左轮方向 | TB6612 BIN1/BIN2 | PB0/PB1 | GPIO |
| 灰度 L1/L2/L3/L4 | 八路灰度 | PB25/PB24/PB20/PA14 | 顺序不可交换 |
| 灰度 R1/R2/R3/R4 | 八路灰度 | PB18/PB19/PB10/PA7 | 顺序不可交换 |
| OLED | SSD1306 | PA28/PA31 | I2C0 |
| MPU6050 | MPU6050/兼容 MPU6500 | PA10=SDA、PA11=SCL | I2C1，0x68，100 kHz |
| 调试串口 | UART3 | PB2=TX、PB3=RX | 当前 115200 8N1 |
| SWD | 调试接口 | PA19/PA20 | SWDIO/SWCLK，禁止占用 |

当前 `Line_ReadSensors()` 按“GPIO 高电平表示检测到黑线”生成内部位图。没有实测证据时不得反相。

## 3. 菜单与 Flash 参数

巡线菜单共 9 项：

1. `MPU`：MPU 辅助开关
2. `Laps`
3. `Start`
4. `KP`
5. `KD`
6. `LostGain`
7. `Speed`
8. `PivTime`
9. `TurnSpd`：直角弯 pivot 外侧轮速度，范围 80～250，步进 5，默认 200

`TurnSpd` 复用 Flash 参数结构中的 `pivot_speed` 字段，掉电保存。旧版曾把该字段固定写成 0，读取到 0 或非法值时必须回退到 200。不要改变现有 Flash 地址、Magic 或结构布局，否则会破坏旧参数兼容。

## 4. MPU6050 与 yaw 规则

- 初始化量程：加速度计 ±2g，陀螺仪 ±2000°/s。
- DLPF 保持 CFG=3（约 44 Hz），不要直接照搬参考工程的 5 Hz 配置。
- I2C1 接收 FIFO 只有 8 字节，必须继续分段读取：加速度 6 字节、陀螺仪 6 字节、温度 2 字节。禁止恢复一次读取 14 字节。
- 上电成功后静止校准 200 点；校准期间车辆和 MPU 必须保持静止。
- 静止时仅在加速度模长接近 1g 且三轴角速度均小于 0.5°/s 时，缓慢修正 Z 轴浮点零偏。
- roll/pitch 继续使用 Mahony；yaw 不使用加速度计修正，而是对校准后的 `gyro_z` 按真实 `dt` 做梯形积分。
- 六轴 MPU6050 无磁力计，yaw 只能作为短期相对航向，长期漂移无法彻底消除。
- 当前实测方向：逆时针 yaw/gyro_z 为正，顺时针为负。
- `MPU6050_QuaternionReset()` 必须同时清零相对 yaw、上一帧角速度、四元数积分状态和时间基准。
- 上电 OLED 只保留一页 `MPU INIT / KEEP CAR STILL`；失败后纯灰度巡线仍可使用。
- 首页 K3 进入 yaw 测试；K3 短按归零，K4 短按返回。PB2 输出一列 ASCII 数值。

## 5. MPU 辅助巡线逻辑

灰度传感器始终是主控，MPU 只辅助，不得让 yaw 在 pivot 阶段抢控制权。

- MPU OFF：保持原八路灰度 PD、丢线搜索和 pivot 状态机。
- MPU ON：
  - 每轮先更新 MPU，复用最近一次的 yaw 与 gyro_z。
  - 灰度居中连续达到 `YAW_LOCK_MS` 后才锁定当前 yaw。
  - 直道航向控制使用角度 P + 实测角速度阻尼，不使用按循环次数累积的 I/D。
  - 直道融合为灰度 PD 70% + yaw 辅助 30%。
  - 一旦灰度出现明确转向误差，立即释放直道 yaw 锁。
  - 普通弯道使用 gyro_z 对灰度 PD 做小幅动态阻尼。
  - pivot 期间禁用 yaw 锁定，由最外侧灰度和中心重捕获决定退出。
  - 丢线时优先保持已锁定航向，并叠加小幅搜索；此前未锁定时才以丢线瞬间航向为目标。

符号约定：

- `pid_output > 0` 对应顺时针/右转修正。
- 逆时针 yaw 为正，因此航向误差必须使用 `current - target`。
- 普通弯道阻尼为 `line_pd + gyro_damp`；不要改回减号。

## 6. 直角弯 pivot

- `TurnSpd` 直接决定外侧轮速度。
- 内侧轮反转速度由 `TurnSpd / 2` 计算，并受 `PIVOT_REVERSE_MIN/MAX` 限制。
- `PivTime` 是最小保持时间，不是固定转弯时间。
- 退出条件仍需同时满足：已离开旧线、达到最小保持时间、中心两路连续重捕获。
- pivot 超时只用于传感器异常保护。
- 第一次测试方向、快速转弯或高 PWM 时必须架空车轮。

## 7. 文件职责与禁止事项

- `empty.c`：模式、按键、OLED 菜单、参数装载/保存、主循环。
- `BSP/src/line.c`：灰度位置、PD、MPU 辅助、丢线与 pivot 状态机。
- `BSP/src/mpu6050.c`：I2C 驱动、校准、在线零偏、姿态与 yaw。
- `BSP/src/param_store.c`：Flash 参数读写。
- `empty.syscfg`：引脚、时钟和外设配置的唯一源文件。
- 禁止手工修改 `Debug/ti_msp_dl_config.c/h`、链接脚本、对象文件和 `.out`。
- 禁止擅自更改 TB6612 A/B 通道、左右轮、灰度顺序、MPU PA10/PA11、SWD PA19/PA20。
- 未经用户明确要求不得烧录、启动电机或删除 Debug/工程目录。

## 8. 构建验证

每次修改源码后执行：

```powershell
& 'D:/ti/CCS_SDK/sysconfig_1.26.2/sysconfig_cli.bat' `
  --script 'D:/CCS/workspace_v12/MYCode_v6_MPUMode/empty.syscfg' `
  -o 'D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug' `
  -s 'D:/ti/CCS_SDK/mspm0_sdk_2_10_00_04/.metadata/product.json' `
  --compiler ticlang

Set-Location 'D:/CCS/workspace_v12/MYCode_v6_MPUMode/Debug'
& 'D:/ti/ccs1281/ccs/utils/bin/gmake.exe' -j4 all
```

只有 SysConfig、编译和链接均成功，并生成 `Debug/MYCode_v6_MPUMode.out`，才能说明构建通过。构建通过不等于实车验证通过。

## 9. 当前待实车验证项

- 快速逆时针/顺时针 90° 后 yaw 比例和回零误差。
- MPU ON 时直道是否减少摆动，方向修正是否正确。
- 普通弯道 gyro 阻尼是否过强或过弱。
- `TurnSpd` 在 80～250 范围内对不同直角弯的效果。
- 修改 `TurnSpd`、断电重启后参数是否正确恢复。

