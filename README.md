 # F407 HAL 自平衡车 (ICM42688 + 双环 PID)

基于 STM32F407、HAL 库与 ICM42688 六轴 IMU 的小型两轮自平衡车示例工程。项目采用 CMake 构建，集成时间片任务调度器 TMT、互补滤波姿态解算、速度/角度双环 PID、编码器测速、PWM 驱动直流电机以及 OLED 显示。

> 如果你来自 Keil/IAR 环境，本工程演示了使用 `arm-none-eabi-gcc + CMake` 的轻量化跨平台构建方式，便于在 VS Code / CLion 等现代 IDE 中开发与调试。

---

## ✨ 功能特性

- ICM42688 初始化与 6 轴数据读取（加速度 + 陀螺仪 + 温度）
- 互补滤波 (Complementary Filter) 姿态解算，输出欧拉角 (roll/pitch/yaw)
- 双环 PID：
  - 外环：速度（基于双路编码器计数平均值）
  - 内环：直立角（roll 保持）
- 电机 PWM 驱动 + 编码器测速（TIM2 / TIM3 作为编码器接口）
- OLED 显示调试信息（任务化刷新）
- 轻量级时间片任务调度器 TMT（定时器中断驱动 tick）
- 参数易调试：PID 结构体在 `main.c` 中集中初始化
- CMake + GCC 交叉编译，生成 `LED.elf / .map`

---

## 🛠 硬件平台

| 模块 | 说明 |
|------|------|
| MCU | STM32F407（外部 HSE，168MHz 配置） |
| IMU | ICM42688（I2C 地址 0x69） |
| 电机 | 两路直流编码电机（脉冲换算：`22 * 30 = 660` 脉冲/圈） |
| 显示 | 0.96"/0.91" OLED（I2C/SPI 取决于你的硬件实现） |
| 供电 | 电机 + 控制分开稳压（建议） |
| 调试 | ST-Link V2 |

> 请根据你实际接线核对 `imu42688.ioc` 中的引脚分配。

---

## 🧱 软件结构概览

```
Core/              STM32CubeMX 生成的 HAL 框架与启动文件
Drivers/           HAL 与 CMSIS 库
User_Core/         用户功能模块（电机 / PID / IMU / OLED / 任务）
TMT/               时间片任务调度器（Apache-2.0）
cmake/             交叉编译工具链与 CubeMX 子目录集成
CMakeLists.txt     顶层构建脚本
```

关键数据流：

1. TIM6 周期触发读取 IMU → `ICM42688_Get6Axis()` → 互补滤波更新 `car_angles`
2. TIM12 周期 (聚合计数达到条件) → 编码器速度计算 + 外环速度 PID → 生成角度偏置
3. 内环角度 PID → 输出 PWM → `Motor_SetLeftPWM / Motor_SetRightPWM`
4. TMT 调度 `Task_Attitude` / `Task_OLED` 周期刷新显示或运算

---

## 🧩 关键模块说明

| 模块 | 文件 | 说明 |
|------|------|------|
| IMU | `User_Core/Src/icm42688.c` / `icm42688.h` | I2C 读写 & WHO_AM_I 检测 & 6 轴原始数据转换 |
| 姿态解算 | `icm42688.c` | `Complementary_Filter()` 互补滤波；`Attitude_Init()` 初始化 |
| PID | `User_Core/Src/PID.c` / `PID.h` | 通用增量/位置式（当前实现为位置式）结构体 + 约束 |
| 电机 & 编码器 | `Motor.c` / `Motor.h` | PWM 设置与编码器脉冲读取 |
| 任务调度 | `TMT/TMT.c` / `TMT.h` | 基于 TIM14 tick 的简单时间片轮询 |
| 任务封装 | `Task.c` / `Task.h` | OLED / Attitude 更新任务入口 |
| 主控 | `Core/Src/main.c` | 时钟、外设、PID 参数、主循环 `tmt.run()` |

---

## ⚙️ 构建环境准备（Windows 示例）

1. 安装 ARM GCC 工具链：
   - 推荐：Arm GNU Toolchain (原 arm-none-eabi-gcc) 12.x 或以上
   - 安装后将其 `bin` 目录加入系统 `PATH`
2. 安装 CMake ≥ 3.22
3. 可选：Ninja（更快）
4. VS Code 插件（可选）：CMake Tools / Cortex-Debug

验证：

```powershell
arm-none-eabi-gcc --version
cmake --version
```

---

## 🧪 编译步骤（命令行）

```powershell
# 1. 创建构建目录
mkdir build; cd build

# 2. 生成工程 (Debug)
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..

# 3. 编译
cmake --build . --target LED -j

# 4. 查看尺寸
arm-none-eabi-size LED.elf
```

生成的产物：`build/LED.elf`、`LED.map`

> 如需 Release：`-DCMAKE_BUILD_TYPE=Release`

---

## 🔌 烧录 / 调试

使用 ST-Link：

1. 安装 OpenOCD 或 STM32CubeProgrammer
2. 典型 OpenOCD 烧录命令：

```powershell
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/LED.elf verify reset exit"
```

或者使用 `Cortex-Debug` 在 VS Code 中配置 `launch.json`：

```jsonc
{
  "name": "Debug F407",
  "type": "cortex-debug",
  "request": "launch",
  "servertype": "openocd",
  "executable": "${workspaceFolder}/build/LED.elf",
  "configFiles": [
    "interface/stlink.cfg",
    "target/stm32f4x.cfg"
  ],
  "runToEntryPoint": "main"
}
```

---

## 🧮 双环 PID 简述

主控在 `TIM12` 的周期回调中完成：

1. 读取编码器：`cntL / cntR` → 平均脉冲数 → 速度 (m/s)
2. 速度环（外环）：`pidSpeed.Target = Speed`（整车目标速度，可由按键或串口设定）
3. 速度环输出 → 角度偏置 `angle_offset`
4. 角度环（内环）：`pidAngle.Target = -angle_offset`，反馈 `roll`
5. 输出 PWM（两侧一致，可扩展差分转向）

当前 PID 参数位于 `main.c`：

```c
PID_t pidAngle = { .Kp=8.8f, .Ki=0.01f, .Kd=4.8f, .OutMax=200, .OutMin=-200 };
PID_t pidSpeed = { .Kp=5.0f, .Ki=0.0f,  .Kd=0.5f, .OutMax=50,  .OutMin=-50  };
```

调参建议：

| 阶段 | 操作 | 观察 |
|------|------|------|
| 1 | 置 `Ki=0`，先调 Kp | 直立是否快速回正/有余振 |
| 2 | 加 Kd | 抑制过冲，减少震荡 |
| 3 | 速度环 Kp 由小到合适 | 车不再明显前后漂移 |
| 4 | 需要时微加 Ki | 长期漂移校正（防积分饱和） |

---

## 🧭 姿态解算（互补滤波）

流程：

1. 陀螺积分获得短期动态角速度变化
2. 加速度（重力方向）提供低频参考
3. 加权融合：`angle = α*(angle + gyro*dt) + (1-α)*acc_angle`
4. 输出 `euler_angles_t car_angles`

如需更高精度，可替换为 Mahony / Madgwick / EKF（高算力时）。

---

## ⏱ 任务调度 (TMT)

`TIM14` 中断中调用 `tmt.tick()`，主循环持续 `tmt.run()`：

```c
tmt.create(Task_OLED, 100);     // 每 100 ticks 刷新 OLED
tmt.create(Task_Attitude, 100); // 可扩展，当前姿态主要在定时器中断完成
```

添加一个新任务：

```c
void Task_User(void) { /* ... */ }
tmt.create(Task_User, 50);  // 50 ticks 周期
```

---

## 🔧 参数 & 魔术数字来源

| 名称 | 位置 | 含义 |
|------|------|------|
| `wheel_circ` | `main.c` | 轮周长 (π * 直径，示例直径 65mm) |
| `pulses_per_rev` | `main.c` | 编码器一圈脉冲数 = 减速机构 * 编码器线数 |
| `control_dt` | `main.c` | 速度环控制周期 (s) |

请根据实际电机/减速比/编码器修改。

---

## 🐞 常见问题 (FAQ)

| 问题 | 可能原因 | 处理 |
|------|----------|------|
| ICM42688 init failed | I2C 地址/焊接/上电顺序 | 检查上拉电阻、示波器测 SCL/SDA |
| 姿态角跳变 | I2C 读包错/滤波参数不合理 | 增大互补滤波低通比例，检查溢出 |
| 车体快速倒下 | 角度环 Kp 太小或方向接反 | 反向 PWM / 增大 Kp |
| 震荡严重 | Kp 过大或 Kd 太小 | 减小 Kp，增大 Kd |
| 一侧轮不转 | PWM 通道/引脚定义不符 | 核对 CubeMX & `Motor.c` |

---

## 🚀 后续可拓展

- 串口/蓝牙参数在线调试
- 速度差分控制实现转向
- Mahony / Madgwick 替换互补滤波
- Flash 保存 PID 参数
- 加入 FreeRTOS（演示从 TMT 迁移）
- CAN / 无线遥控

---

## 📄 许可证与版权

| 组件 | 许可证 |
|------|--------|
| 本项目用户代码 | MIT（可根据需要自行补充 LICENSE） |
| STM32 HAL & CMSIS | ST 官方许可（位于 `Drivers/` 中的 `LICENSE.txt`） |
| TMT 调度器 | Apache-2.0 |

请确保下游分发时保留相关版权声明。

> 建议在仓库根目录新增 `LICENSE`（若你选择 MIT / Apache-2.0 / BSD 等）。

---

## 🙌 致谢

- STMicroelectronics HAL & CMSIS
- zeweni 的 TMT 任务调度框架
- 社区开源平衡车调参经验分享

---

## 📨 反馈

欢迎提交 Issue / PR：
- 优化姿态解算
- 改进 PID 结构（抗积分饱和 / 前馈）
- 更多显示/调试接口

如果你在使用中遇到具体问题，可在 README 末尾追加使用环境（编译器版本 / 供电 / 负载）。

祝调参愉快！🤖
