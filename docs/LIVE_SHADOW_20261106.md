# 真实传感器到 SuperMimic：C++ 零增益只读 shadow

2026-11-06，在 253 的独立目录完成 10 秒和 60 秒测试，分别处理 **500 / 3000 帧**。
真实电机反馈、真实 ICM45686、原生 993 维观测与最终 run002 模型已连通。
所有实时环节均为 C++；Python 仅准备静态配置、监督进程和事后验证。
机器人由吊架支撑、脚着地，使用用户确认的 5 A 稳压供电；电机始终未使能。

## 数据流和隔离

```text
四路真实 CAN-FD ↔ C++ 零增益收发器（每电机500Hz）
                    ↓ 50Hz、528字节电机状态，使用最旧反馈的真实时间戳
                    ↓ Unix SOCK_SEQPACKET，只读actor已SHUT_WR
SPI0 → C++ ICM800Hz → SPIMU003 → C++ SuperMimic actor
                                   ↓ 10帧/993维 → ONNX CPU → 31维目标
                                   ↓ 仅保存文件，不写回电机状态通道
```

收发器在每次 CAN 写入前验证原始 Kp/Kd=0，速度和前馈均为零编码；它没有使能或模式切换入口。
状态服务器先确认同用户的 actor 已关闭写方向，再开始 CAN 测试；任何反向字节都会拒绝。
actor 继续使用内核 `SHUT_WR`，在计时循环中没有目标发送调用。

状态 ABI、31 电机名称/顺序哈希、校准零点和逆映射均验证。
状态发布时间戳取 31 电机中最旧的实际反馈，并扣除内核接收到用户态的排队时间，
不以发布时刻刷新旧反馈。反馈必须保留内核软件和原始硬件时间戳。
电机状态与 IMU 的 30 ms 新鲜度检查在推理前后保持生效，20 ms 周期没有放宽。

ICM 使用独立的 SPIMU003 量程、偏置和坐标转换。新参数 `--imu-kind icm` 显式选择它；
默认 software/SPIMU002 保留给原来的合成回放，跨类型数据会拒绝，不改包头伪装。
原有带电控制器没有替换；新增的是仅用于零增益诊断的状态发布路径。

## 结果

模型 SHA256：`f3529c927a44f970908d48e5694e63302d0fc838121aba2fa87c0a0863968bba`。
10 帧/993 维、31 动作、50 Hz，2 个推理线程绑定 CPU4/5；CAN CPU2，ICM CPU3。

可检查的[数值报告](../reports/jetson-live-shadow/summary.json)随源码保存。
413个证据文件已拉回本地并逐文件SHA256验证，私有归档
`live-supermimic-shadow-evidence.tar.gz` 的SHA256为
`aa098612af3809d90a86167b8933d6658846db781b04b59f224245c3a711df56`。
真实电机/IMU原始记录保留在本地，不上传仓库或公开Release。
命令均为 `(0,0,0)`，测量姿态交接 1.5 秒。SPI0 固定 `on`，整机功耗/时钟策略未修改。

| 指标 | 10 秒 | 60 秒 |
|---|---:|---:|
| 实际策略帧 | 500 | 3000 |
| 观测+推理+交接 P99 | 6.561 ms | **6.493 ms** |
| 上述服务最大值 | 6.772 ms | **8.607 ms** |
| 最旧电机反馈到推理完成 P99 | 8.380 ms | **8.369 ms** |
| 最旧电机反馈到推理完成最大值 | 8.788 ms | **11.923 ms** |
| 推理前电机状态最大年龄 | 2.140 ms | 4.514 ms |
| 推理前 IMU 最大年龄 | 6.406 ms | 7.940 ms |
| 超过20ms截止期 | **0** | **0** |
| 原生 CAN 发送 / 记录回复 | 154805 / 154798 | 928901 / 928892 |
| actor反向发送字节 / 使能尝试 | **0 / 0** | **0 / 0** |

CAN 发送与记录回复末尾有少量差值，工具在最后一个状态帧后有限退出，
不把未写入采样文件的末尾回复称为内核丢包；实际接口计数另行保存。
“反馈到推理完成”不包括目标回传、非零 PD、残余阻尼执行或电机施力。

IMU 存在真实的丢弃/合并事件。10 秒用例的完整 35 秒采集期有 6 个超周期丢弃、12 个
序号遗漏/合并；60 秒用例的完整 85 秒采集期为 19 个丢弃、45 个遗漏/合并。
按 actor 运行窗口筛选，分别为 2 / 17 个丢弃，SPI P99 约 0.181 / 0.187 ms。
配置 ODR 为 800 Hz，不把它写成实际零丢失的800Hz；被丢弃的样本没有重用为新样本。
所有实际消费的输入都通过既有新鲜度保护，原始计数保留在报告里。

## 独立验证与故障验证

- 实际消费的71值输入全部保存，离线 C++ 重放输出逐字节一致。
- 独立 Python 观测、原始动作、交接动作、最终目标递推最大误差均为 **0**。
  复核使用相同的实际输入和明确的75步交接，不修改冻结模型包。
- 软件 IPC 的正常、陈旧状态、乱序、断开、IMU冻结五个用例全部符合预期。
- 原生只读服务器拒绝反向字节、未关闭写方向的客户端、陈旧状态及覆盖已有 socket。
- ICM 测试覆盖200种姿态、量程/坐标转换、SPIMU002/003错配、损坏与新鲜度错误。
- 14 个坏输入/错模型回放用例通过。6 项原生测试通过。
- Jetson现有Python环境以unittest运行308项：307通过，1项因未安装可选MuJoCo而跳过；
  该MuJoCo惯量轴回归此前已在租用服务器通过。未为测试修改机器人原有Python环境。

## 复现入口

先确认吊架、驱动供电、全部未使能、已确认MIT模式和四路CAN-FD 1Mbps/5Mbps配置。
先安装 [SPI固定on设置](SPI_POWER_MANAGEMENT.md)。使用现有CPU ORT1.23.2及匹配头文件，
不要用x86依赖文件覆盖Jetson环境。

```bash
export SPRITE_REPLAY_PYTHON=/path/to/validated/python
export SPRITE_ORT_INCLUDE=/path/to/matching/ort/headers
export SPRITE_REPLAY_BUILD=build/native
bash runtime/tools/build_supermimic_replay.sh
cmake -S runtime/native/icm45686 -B runtime/build/icm -DCMAKE_BUILD_TYPE=Release
cmake --build runtime/build/icm --parallel 2
ctest --test-dir runtime/build/icm --output-on-failure

PYTHONPATH=runtime/src "$SPRITE_REPLAY_PYTHON" runtime/tools/run_supermimic_live_shadow.py \
  --bundle /path/to/run002/bundle --hardware /path/to/frozen/hardware.json \
  --output /path/to/new-private-output --seconds 10 \
  --supported-unloaded --acknowledge-zero-gain-tx
```

启动前再次核对总线参数与冻结硬件哈希。IMU先静态标定15秒，再连接只读actor。
输出目录必须全新。现场需要人工确保物理状态，不能在依靠电机保持力矩支撑时使用零增益。
监督程序会等待原生程序有限退出，做离线复核；异常时终止子进程，不触发使能或自动恢复。

这次通过的是**零命令、未使能、机械支撑条件下的传感器到策略只读链路**。
它不验证机器人实际跟踪策略目标、并联踝带载表现、完整执行器时序或动态平衡。
带电运动准入仍为false，下一阶段是基于真实记录核对执行器目标/约束与完整调度，
不能从这份报告直接跳到带电走路。
