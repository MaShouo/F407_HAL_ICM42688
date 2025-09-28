
## 项目简介

本仓库是基于 STM32F407（HAL 库 + CMake 构建）实现的一个小型两轮自平衡车（或姿态采集平台）控制程序，核心包含：

1. ICM42688 六轴 IMU 采集与姿态解算（互补滤波）
2. 双环 PID（内环：直立角度；外环：速度/位移补偿）
3. 双路编码器测速（TIM2 / TIM3）
4. PWM 电机驱动（TIM8 CH1/CH2）
5. OLED 显示调试信息
6. 轻量级任务调度器 `TMT`（定时调度 OLED / 姿态刷新等任务）

> 目标：稳定保持小车直立，并可在后续扩展速度/位移/遥控功能。

---

## 硬件平台

| 模块 | 说明 |
| ---- | ---- |
| MCU | STM32F407（外部 HSE，168MHz） |
| IMU | ICM42688（I2C 接口，地址 0x47 WHO_AM_I） |
| 编码器 | 2 路 AB 相增量式编码器，分别接 TIM2 / TIM3  |
| 电机驱动 | 全桥（方向 GPIO + PWM） |
| 显示 | 0.96 寸 OLED（I2C/SPI，代码中为自写驱动接口） |
| 其他 | 按键、调试口、供电模块 |

（具体管脚分配参见根目录文档 `平衡车F4引脚分配.docx` 或 CubeMX 配置文件 `imu42688.ioc`）

---

## 软件 / 工具链

| 组件 | 版本 / 说明 |
| ---- | ---------- |
| 编译器 | arm-none-eabi-gcc (需支持 C11) |
| 构建系统 | CMake ≥ 3.22 |
| 依赖 | STM32 HAL / CMSIS（已随仓库提供） |
| 调试 | OpenOCD / ST-Link Utility / pyOCD 均可 |

Windows 下建议安装：
1. Arm GNU Toolchain（添加到 PATH）
2. CMake + Ninja
3. ST-Link 驱动 + OpenOCD 或 STM32CubeProgrammer

---

## 目录结构概览

```
Core/                # CubeMX 生成的核心工程：时钟、外设初始化、启动代码等
User_Core/           # 用户功能层（应用逻辑）
  Inc/               # 用户头文件
  Src/               # 用户源文件（PID / IMU / OLED / Motor / Task / Delay 等）
TMT/                 # 轻量级任务调度器
Drivers/             # HAL & CMSIS 库
cmake/               # CMake 工具链、CubeMX 子目录构建脚本
startup_stm32f407xx.s# 启动文件
STM32F407XX_FLASH.ld # 链接脚本
imu42688.ioc         # CubeMX 配置文件（如需修改外设重新生成）
```

关键自定义模块：

| 模块 | 文件 | 功能 |
| ---- | ---- | ---- |
| IMU 驱动 | `icm42688.c/h` | I2C 读写、寄存器配置、六轴数据采集 |
| 姿态解算 | `icm42688.c` | `Complementary_Filter` 互补滤波 + 初始静置校准 |
| PID 控制 | `PID.c/h` | 位置式 PID，包含积分开关与限幅防饱和 |
| 电机与编码器 | `Motor.c/h` | PWM 输出 + 正反转 GPIO 控制 + 编码器读数并清零 |
| 任务调度 | `TMT.c/h` | 周期任务创建、tick 节拍、`tmt.run()` 主循环调度 |
| OLED | `OLED*.c/h` | 基础字符/格式化输出刷新界面 |
| 任务逻辑 | `Task.c` | 信息显示、姿态/速度参数可视化 |

---

## 控制算法说明

整体采用 双环串级 PID：

1. 外环：速度（基于编码器计数 -> 线速度估算），目标为 0（保持原地），输出为“角度偏移指令”
2. 内环：直立角度 PID，目标角 = 0° - 外环输出（动态平衡）
3. PWM 输出：内环 PID 输出直接映射为左右轮功率（带方向 GPIO 控制）

主要参数（`main.c` 中初始化）：
```
pidAngle: Kp=8.8, Ki=0.01, Kd=4.8, 输出限幅 ±200
pidSpeed: Kp=5.0, Ki=0.0,  Kd=0.5, 输出限幅 ±50
```

编码器换算：
```
轮径约 65mm -> 轮周长 ≈ 0.2042 m
减速比*编码器：22 * 30 = 660 脉冲/轮圈
速度(m/s) = 平均脉冲 * 轮周长 / 660 / 控制周期
```

姿态解算：
```
互补滤波 alpha = 0.98 （陀螺积分为主，加速度校正为辅）
首次更新或 dt>0.1s ：使用加速度初始化 roll/pitch
Yaw 暂未融合磁力计，仅陀螺积分（会漂移）
```

定时器中断分工（`HAL_TIM_PeriodElapsedCallback`）：
| 定时器 | 任务 |
| ------ | ---- |
| TIM12 | 控制周期：编码器读取 + 双环 PID + PWM 更新（每若干 tick） |
| TIM14 | 为调度器 `tmt.tick()` 提供系统节拍 |
| TIM6  | 采样 IMU + 调用互补滤波（降低主循环负载） |

主循环：
```c
while(1) {
	 tmt.run(); // 执行注册的 OLED / Attitude 等任务
}
```

---

## 构建与烧录

### 1. 准备环境（Windows PowerShell）

安装并验证：
```
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

### 2. 生成与编译

```
mkdir build; cd build
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug ..
ninja
```

生成的目标：`LED.elf` / `LED.bin`

### 3. 烧录示例（任选其一）

OpenOCD：
```
openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program build/LED.elf verify reset exit"
```

STM32CubeProgrammer：
```
STM32_Programmer_CLI -c port=SWD -d build/LED.bin 0x08000000 -v -rst
```

JLink（如使用 JLink）：
```
JLink.exe -device STM32F407VG -if SWD -speed 4000 -CommanderScript flash.jlink
```

`flash.jlink` 示例：
```
r
erase
loadfile build/LED.bin,0x08000000
verifybin build/LED.bin,0x08000000
g
q
```

---

## 参数调试建议

1. 先只调内环（角度）：
	- 断开/屏蔽外环（把外环输出 angle_offset=0）
	- 增大 Kp 直到有轻微振荡，再稍微回退
	- 加入 Kd 抑制快速震荡；最后极小幅度增加 Ki 消除静差
2. 启用外环（速度）：
	- 保持 Ki=0 先调 Kp，确保前后摆幅减小
	- 再加入少量 Kd 平滑响应；最后再斟酌 Ki
3. 防积分饱和：`PID.c` 中已在 Ki=0 时清零积分，可避免积累爆发
4. OLED 实时查看：
	- `PS` 行：速度环输出
	- `Pitch/Roll/Yaw`：姿态角
	- `L/R`：两侧编码器增量
	- `S/T`：速度实际值 / 目标值
5. 若姿态抖动：
	- 查看 IMU 固定是否牢固、陀螺零漂是否过大（可扩展温漂补偿）
6. 若速度估算跳变：
	- 检查编码器接线/滤波，可对脉冲做低通或滑动平均

---

## 常见问题 FAQ

| 问题 | 排查思路 |
| ---- | -------- |
| ICM42688 init failed | 检查 I2C 供电/地址/上拉；示波器看 SCL/SDA 波形 |
| 姿态角突然跳变 | 采样间隔 dt 是否异常（任务被长阻塞）；IMU 震动过大 |
| 小车一侧轮子不转 | GPIO 方向脚或 PWM 通道引脚复用配置是否正确 |
| 编码器读数为 0 | TIM2/TIM3 时钟/复用、AB 相接反（可尝试交换 A/B） |
| PID 不收敛 | 先降速环输出限幅；逐级调参（内环 → 外环） |

---

## 后续可扩展方向

- [ ] 加入遥控（蓝牙/串口/RC 接收机）
- [ ] 位置环（对累计位移进行控制）
- [ ] 使用 Kalman / Mahony / Madgwick 取代互补滤波
- [ ] Flash 参数存储（保存 PID）
- [ ] 增加故障保护（倾角>阈值自动停 PWM）
- [ ] 电池电压检测 + OLED 显示
- [ ] Yaw 校正（加入磁力计或光流/视觉）

---

## 快速开始 TL;DR

1. 接好硬件，确认 I2C & 编码器 & PWM 引脚一致
2. 编译：`cmake -G Ninja -B build -DCMAKE_BUILD_TYPE=Release . && ninja -C build`
3. 烧录 `LED.bin` 到 0x08000000
4. 上电后观察 OLED：若显示 `ICM42688 init success` 即开始运行
5. 调参：优先 Angle 内环，其次 Speed 外环

---

## 许可协议

HAL/CMSIS 属于 ST 官方许可，本项目用户代码默认以 MIT 授权（如需变更可在根目录添加 LICENSE）。

---

## 致谢

感谢 STM32 HAL、开源社区提供的工具链支持。如果你有改进建议或发现问题，欢迎提交 Issue 或 PR。

---

若需英文 README 或补充波形/照片，请提 Issue 告知。

