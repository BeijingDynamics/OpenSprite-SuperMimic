# Jetson ICM45686：SPI0 固定关闭运行时省电

**项目约定（2026-11-06）：在 253 上把实际 ICM 使用的 SPI0 控制器 `power/control`
固定为 `on`，开机设置，runtime 退出也不恢复 `auto`。**
用户明确选择稳定采集时序优先，不需要为了这部分功耗在采集前后切换省电模式。
这是电机实时链路的前置条件之一，不能遗漏在新安装说明里。

## 这项设置控制什么

- 目标：Orin 的 `/sys/bus/platform/devices/3210000.spi/power/control`，对应当前的
  `/dev/spidev0.0`（同控制器 CS1 是 `/dev/spidev0.1`）。
- `on`：禁止该控制器运行时自动挂起；`auto`：允许内核运行时电源管理。
- **写 `on` 是关闭省电，不是开启省电。** 它不使能电机，也不修改 CAN、Kp/Kd、
  CPU governor、整机功耗档位或采样新鲜度保护。
- 另一控制器 `3230000.spi` 不在本项设置范围内，不对所有 SPI 设备一律改写。

## 为什么列为必需配置

旧成功版本通过采集包装脚本临时把 SPI runtime PM 设为 `on`，结束时恢复原值。
新独立目录直接运行 C++ 采集器时，没有经过该包装流程，控制器仍为 `auto`。

在 31 电机、四总线 C++ 500 Hz 零增益与 C++ ICM 800 Hz 并发的 45 秒测试中，
`auto` 条件下观察到：

| 量 | 记录 |
|---|---:|
| SPI 调用耗时中位数 / P95 / P99 | 336.811 / 966.741 / **1508.199 µs** |
| SPI 调用耗时最大值 | 3661.278 µs |
| GPIO 事件到开始读取 P99 | 371.923 µs |
| 800 Hz 单采样周期 | **1250 µs** |
| 处理事件 / 接受样本 / 超周期丢弃 | 35637 / 34665 / **972** |
| 最大接受样本间隔 | 6.314 ms |

采集器完成并成功标定，但“正常退出”不等于没有时序缺口。
超过单周期的读取仍按原规则丢弃，没有延长新鲜度限值来制造通过结果。
上述记录表明 SPI 调用尾部值得处理。不能把电机通信通过当作整条策略链路通过。

## 同条件复测：固定为 `on` 后

安装永久设置后再次运行 45 秒；仍为原生四总线 500 Hz 零增益并发 C++ ICM 800 Hz，
没有改变 CPU 亲和性、功耗档位或采样丢弃规则。

| 指标 | `auto` | `on` |
|---|---:|---:|
| SPI 调用中位数 | 336.811 µs | **96.416 µs** |
| SPI 调用 P95 | 966.741 µs | **546.333 µs** |
| SPI 调用 P99 | 1508.199 µs | **652.012 µs** |
| SPI 调用最大值（含丢弃样本） | 3661.278 µs | **1233.883 µs** |
| GPIO 事件至开始读取 P99 | 371.923 µs | **36.672 µs** |
| 超单采样周期丢弃 | 972 | **1** |
| 事件序号遗漏 / 合并 | 14 / 14 | **0 / 0** |
| 接受的 IMU 样本 | 34665 | **35652** |
| 最大接受样本间隔 | 6.314 ms | **2.533 ms** |

本次对比支持固定关闭运行时省电对该配置的时序有明显帮助；并不证明所有延迟都由
省电造成，也不是最坏时延保证。`on` 条件仍有 1 个样本被原规则丢弃，不能写成零丢失。
800 Hz 是配置 ODR；本次 45 秒实际接受约 792.3 样本/秒，不把配置值直接当成实测频率。

电机侧 `on` 条件发送 697279 帧、采集 697272 个回复，每电机约 499.82 Hz，
最大反馈间隔 3.469 ms；所有记录均为 disabled。内核总线错误/丢包均为 0。
原始数值汇总见 `reports/jetson-zero-gain/SPI_PM_COMPARISON.json`；原始 CSV 已拉回本地。

## 旧项目经验的有限摘录

从 `Access 192.168.0.234` 项目仅提取了相关结论文件
`work/icm45686_latency_findings_20261015.txt`、既有 PM 包装脚本及 ICM 调试文档，
没有将原聊天或其他训练实验上下文合并进本项目。

- **2026-10-15 已验证过同类现象。** IMU 单独采集、`auto`、30 秒：1250 个读取后迟到、
  3 个读取前迟到，全部读取中的最大 SPI 时长约 3.515 ms。`on`、30 秒：迟到/遗漏/
  合并均为 0，SPI P99 0.649 ms。`on`、300 秒：接受 237670 样本，遗漏/合并为 0，
  迟到丢弃 5 个，SPI P99 **0.655 ms**，实际接受约 **792.23 Hz**。
  这是旧 IMU-only 证据，不能冒充今天的电机并发负载结果。
- **旧脚本的 PM 临时状态是隐藏的迁移依赖。**
  `diagnose_sprite_icm_spi_pm_on_253.sh` 先验证 spidev 对应 `3210000.spi`，用 flock
  串行化 PM 所有权，设为 on，EXIT/INT/TERM 后恢复启动前原值。
  仅复制 C++ 采集器而绕过包装脚本，会丢失这项运行条件。本项目现在固定默认 on，
  不再依赖临时开关。
  旧 300 秒记录还区分了 wall time 与线程 CPU time：迟到读取的中位数约 1.270 ms，
  CPU 仅约 0.108 ms，余下约 1.162 ms 是 off-CPU 时间。不能把 ioctl 耗时直接等同
  线上传输时长，也不能仅凭 off-CPU 就断定全部是 PM resume 而不是其他等待/调度。
- **全零读数先排查信号连通性。** 旧记录有一次模块 SPI 串联电阻误拆，导致信号断路；
  修复后 WHO_AM_I 恢复 `0xE9`。这与 Jetson 低频 clock-parent 配置是不同问题，
  不能把硬件开路都归为 SPI 软件或频率故障。
- **保留已验证的采集合同。** Mode3/MSB、请求 1 MHz、800 Hz ODR、INT1/PIN22，
  每次一次 14 字节六轴/温度 burst；请求/驱动回读频率不等于示波器验证过线上频率。
  数据寄存器不是 FIFO 历史，不能把过期事件反复读取后伪装成新样本。
- **ICM 的 SPIMU003 不应伪装成 BNO 包。** 保留 ICM 原始量程、传感器坐标与独立适配器，
  后续只为新模型适配输入输出合同，不重新发明已经验收的电机/IMU 底层。

原项目中的运行授权、旧模型准入与动作目标均不继承到本项目；这里复用的是技术经验。

## 一次性安装：开机与设备绑定时设置

先将 runtime 源码放在 Jetson 的独立目录。查看脚本将安装哪些文件：

```bash
bash runtime/tools/install_icm_spi_performance.sh --dry-run
sudo bash runtime/tools/install_icm_spi_performance.sh
```

安装内容仅为：

- `/etc/udev/rules.d/80-open-sprite-spi-performance.rules`：匹配 `3210000.spi`
  的 add/bind 事件，设置 `power/control=on`。
- `/etc/systemd/system/open-sprite-spi-performance.service`：开机 oneshot 设置，安装时立即启动。

安装脚本会备份已存在的同名配置，再写入当前版本。无需保存 sudo 密码，也不增加
免密 sudo 权限；后续普通用户启动 runtime 时只需要读取已设好的状态。

检查：

```bash
cat /sys/bus/platform/devices/3210000.spi/power/control
systemctl is-enabled open-sprite-spi-performance.service
systemctl status open-sprite-spi-performance.service --no-pager
```

预期分别是 `on`、`enabled`、oneshot 服务成功且保持 `active (exited)`。
2026-11-09 在 253 开机约两分钟时复验，`power/control=on`，服务为 `enabled`、`active`，
ICM45686 的 `WHO_AM_I` 为 `0xE9`。四路 `kcan1`–`kcan4` 开机后均为 DOWN，
复测 CAN 前需在本机运行 `sudo bash /home/tony/setup_sprite0825_kcanfd_on_253.sh` 恢复接口配置。
接口恢复后，20 秒零增益链路完成了 1000 帧，推理结束前未错过 20 ms 截止期。
推理服务 P99 为 6.52 ms，最大 16.12 ms。最旧电机反馈到推理结束最长 17.92 ms，
该帧的反馈年龄约 1.80 ms。采集退出后 `power/control` 仍为 `on`。

## C++ 启动行为与退出行为

新 `native/icm45686/main.cpp` 在正式采集前验证 SPI0 PM 为 `on`；有写权限时会尝试设为 `on`。
如果状态仍为 `auto`，采集拒绝启动，并指出一次性安装命令。
只读 WHO_AM_I probe 不改变该设置。退出不写回 `auto`。

旧包装脚本若恢复的是“启动前原值”，在系统默认已经是 `on` 后也会恢复为 `on`。
不要再为新启动入口添加固定恢复 `auto` 的清理步骤。

运行同条件 A/B 测试时，要同时保留 SPI 调用分位数、事件序号遗漏、超周期丢弃、
采样间隔、CPU 亲和性和并发负载；不能只看 800 Hz 的配置值或一个最大值。
测试完成报告见 [runtime 当前状态](RUNTIME.md)，原始硬件轨迹仅保存在本地。
