# 安装、训练与评估

建议 Linux、Python 3.12、支持 CUDA 12.8 的 NVIDIA 驱动。run002 实测硬件为 L40S 48GB。
Windows 本机用于归档和文档；没有验证 Windows GPU 训练。不要把不同版本的 MuJoCo 训练环境混用。

发布前已从本仓库的独立目录验证：训练侧 112 项测试、runtime 308 项测试及 33 个子测试通过，
实际 GPU 参数预检、梯度路由、三步联合训练和 ONNX 导出均通过。
记录在 `reports/repository-validation/`，短训练导出的最大误差约 `3.3e-9`。

## 1. 安装与机器人资产

```bash
git clone https://github.com/BeijingDynamics/OpenSprite-SuperMimic.git
cd OpenSprite-SuperMimic
python3.12 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-training.txt
python -m pip install -e .
python scripts/fetch_robot_assets.py
```

下载工具只获取固定提交的 33 个 STL，逐文件验证 `manifest.json` 中的 SHA256。
已存在但哈希不同的文件会报错，不以更新上游版本来替代。
URDF、发布控制快照与实际训练 MJCF 已在 Git 中；网格和依赖缓存不重复入库。
`provenance/requirements-training-lock.txt` 是原实验的完整环境记录，包含与训练无关的已安装包；
优先使用精简的 `requirements-training.txt` 安装。

## 2. 本地参考动作

阅读 [REFERENCE_MOTIONS.md](REFERENCE_MOTIONS.md)。已有本项目私有本地参考库时：

```bash
python scripts/install_reference_bank.py --source /path/to/authorized/v5_locomotion.npz
```

该工具检查原 run002 参考库哈希。新生成的参考库应放在独立目录，重新执行数值 QA 与人工渲染检查，
而不是修改元数据假装它等同于原库。Git 与 Release 都不提供这些动作数据。

## 3. 预检与短训练

```bash
export MUJOCO_GL=egl
python scripts/preflight_run002.py
python scripts/test_joint_gradient_routing.py
python scripts/check_reference_fk_precision.py
python scripts/train_joint.py --config configs/run002.json \
  --motion data/motions/contact_v5_v11/v5_locomotion.npz \
  --quality data/motions/contact_v5_v11/v5_locomotion.qa.json \
  --output outputs/smoke --smoke
```

预检实际创建 GPU 环境，核对每个关节的 Kp/Kd、驱动力矩范围和一步推进 20 ms。
梯度路由测试确认动作接口可更新共享解码器，而速度接口只更新速度编码器、仍可通过冻结
解码器向输入传播梯度；参考潜变量目标分离梯度。FK 检查禁止低精度破坏参考几何。
输出目录必须不存在，防止覆盖既有实验。

## 4. 完整联合训练

```bash
python scripts/train_joint.py --config configs/run002.json \
  --motion data/motions/contact_v5_v11/v5_locomotion.npz \
  --quality data/motions/contact_v5_v11/v5_locomotion.qa.json \
  --output outputs/joint-waist-v5-002
```

配置上限为 100000 更新，但首阶段墙钟上限为 43200 秒。本次实际到达 15398 更新，
不是训练了 100000 更新。原生 4096 环境及大网络消耗约 14.5 GiB，FP32/禁用 TF32，
实测约 0.35 更新/秒；不能仅按显存或理论 FLOPS 推算缩短时间。

长任务应由会话外监督工具管理，检查完成标记、保存退出码并及时拉回检查点。
租期结束前先保全最终与最佳权重、配置、源代码、参考库、本项目产生的轨迹和日志；
不备份公开数据全集、虚拟环境或通用缓存。

续训用 `--resume checkpoint.pt`，物理/参考/奖励契约不一致会拒绝。回放缓存、实时环境与 RNG
不恢复，因此不声称逐位连续。模型或控制修订应该是新实验，不能悄悄修改正在训练的配置。

## 5. 评估和录像

```bash
python scripts/evaluate_joint.py --checkpoint downloads/run002/model_015398.pt \
  --motion data/motions/contact_v5_v11/v5_locomotion.npz --interface velocity \
  --commands-file configs/evaluation/commands.json --seed 101 --seconds 30 \
  --output outputs/eval-velocity-s101
python scripts/render_velocity.py --rollout outputs/eval-velocity-s101/rollout.npz \
  --output outputs/eval-velocity-s101/video.mp4 --envs 0 1 3 4
```

对种子 202/303 重复，并用 `configs/evaluation/sequences.json` 做 62 秒连续切换。
动作跟踪选择 `--interface motion`；时长受原片段约 7.98 秒限制，不应写成 30 秒。
评估不在失败时重置；录像继续积分，标记第一次失败，不能用重置掩盖摔倒。
视频渲染实际 qpos，画面显示请求与测量速度。

注意 GPU 评估默认从参考库中安静姿态启动；它与严格电机闭环从 URDF 零位启动不同。
初态、根高度和状态历史都属于评估条件，详见结果文件。

## 6. 导出

```bash
python scripts/export_joint.py --checkpoint downloads/run002/model_015398.pt \
  --output outputs/export
```

部署 actor 只需速度编码器和共享解码器；参考库、训练世界模型和价值网络不进入 993 维速度接口。
导出检查 ONNX 与 PyTorch 一致性。导出本身不会授予硬件准入，后续按 [RUNTIME.md](RUNTIME.md)验证。
