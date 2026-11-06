# CPU 回放、严格闭环和 Jetson

本项目 runtime 快照来自 `open_sprite_runtime` 的 `codex/supermimic-v5` 分支提交 `5f8225c`。
原生接口为 10 帧历史、993 维输入、31 维输出，保持旧 8 帧接口独立。
这里所有推荐命令均为文件回放或仿真，没有硬件发送后端。

## 取得模型与仿真输入

从 [Release](https://github.com/BeijingDynamics/OpenSprite-SuperMimic/releases)下载
`run002-models.tar.gz` 并校验 Release 中的 `SHA256SUMS.json`：

```bash
mkdir -p downloads
tar -xzf /path/to/run002-models.tar.gz -C downloads
python3.12 -m venv .venv-cpu
source .venv-cpu/bin/activate
python -m pip install -r requirements-cpu.txt
python -m pip install -e runtime
python -m open_sprite_runtime.supermimic_run002 verify --bundle downloads/run002/bundle
```

模型包不包含任何参考动作。包内 `training-scene.mjb` 是原 GPU 环境导出的实际场景，
加载要求 MuJoCo 3.11.0；重建后场景哈希不同不能冒充原证据输入。
`configs/runtime/hardware-source.json` 是 `configured=false` 的冻结仿真夹具，不是准许上电的配置。

## 严格启动复现

```bash
python runtime/tools/reproduce_strict_startup.py \
  --profile runtime/config/supermimic_run002_strict_startup.experiment.json \
  --contract downloads/run002/bundle/contract.json \
  --hardware configs/runtime/hardware-source.json \
  --scene downloads/run002/training-scene.mjb \
  --commands configs/evaluation/slew-static-commands.json \
  --timing pressure --output outputs/strict-reproduction
```

替换为 `slew-continuous-commands.json` 验证四组连续动作；选择 `--timing nominal` 验证名义时序。
输出目录必须不存在。工具验证输入哈希，强制 `runtime_strict`，拒绝未测请求点，保留 2 秒零命令前缀。
原始请求与斜率限制后的输入分别记录，没有静默裁掉请求而不记录。

候选是 1.5 秒实测姿态交接、50 Hz 残余阻尼前馈、0.25 m/s² 线速度命令变化率和
0.3 rad/s² 偏航命令变化率；嵌入式 Kd 上限仍为 3。其测试点仅为停止、
前进 0.15/0.25、倒退 -0.15、`(0.25,0,±0.15)` 转向与负向横移 -0.15。
不能从这些点推断整个速度命令立方体或任意切换顺序都安全。

仿真包含并联踝/差分映射、500 Hz 踝主机 PD、交错发送、非踝 1 kHz 嵌入式 PD、
50 Hz 策略/命令保持。名义时序延迟 10 ms；压力时序延迟 12 ms、反馈 2 ms、响应滞后 5 ms。
保护包络仍是保守速度/铭牌模型与协议字段代理，不是测功机实测输出曲线。

完整消融和原事件见 [严格启动报告](history/SUPERMIMIC_STRICT_STARTUP_20261106.md)。
原 2 秒交接、无补偿/斜率限制配置仍在 1.30 秒触发原保护，证明没有放宽限制。

## Native CPU 回放

安装 CMake、C++17 编译器，准备与当前 ONNX Runtime **精确同版本**的 C API 头文件。
头文件从对应 ORT GitHub tag 的 `include/onnxruntime/core/session/` 取得，不混用另一版本。

```bash
export SPRITE_REPLAY_PYTHON="$PWD/.venv-cpu/bin/python"
export SPRITE_ORT_INCLUDE=/path/to/matching-ort-headers
bash runtime/tools/build_supermimic_replay.sh
```

该脚本只构建 replay、read-only IPC 和数学测试目标，不启动传感器或 CAN。
下载 `run002-velocity-evaluation.tar.gz`，解压到 `downloads/` 后，可选一组录制的速度测试：

```bash
python runtime/tools/verify_supermimic_replay.py \
  --contract downloads/run002/bundle/contract.json \
  --rollout downloads/velocity-evaluation/final/velocity-seed101/rollout.npz \
  --summary downloads/velocity-evaluation/final/velocity-seed101/summary.json \
  --binary runtime/build/supermimic_offline/sprite_supermimic_replay \
  --output-dir outputs/native-replay --handoff both
```

## Jetson 已做什么、还没做什么

原 Orin Nano Super、25 W、schedutil，CPU ORT 1.23.2；未改功耗/频率策略。
双线程 CPU4/5 的观测+推理+交接 P99 **6.889 ms**，最大 **7.132 ms**，
60 秒/3000 个 20 ms 周期没有截止期遗漏。单线程也通过。
详见 [已保存的 Jetson 报告](history/SUPERMIMIC_RUN002_JETSON_20261106.md)。

这些是录制状态的 CPU 测试；没有真实 IMU/CAN、物理反馈、零增益电机帧或使能。
新增残余阻尼执行器路径尚未在真实反馈与实时总调度中验证。
接下来是接收状态与无发送 shadow、前馈/映射/调度验证；电机运动准入仍为 false。
runtime 中保留的历史硬件调试工具不等于本模型已获准执行那些动作。
