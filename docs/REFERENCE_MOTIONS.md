# 如何在本地得到 v5 重定向参考动作

仓库与 Release 不包含 LAFAN1、裁剪 BVH、重定向 NPZ、UMR 对应关系数据/权重或参考动作录像。
本项目原始成果保存在本地；公开的是源片段索引、处理代码、QA 与哈希。
外部数据遵守其自己的条款，见 [DATA_AND_LICENSES.md](../DATA_AND_LICENSES.md)。

run002 使用的五段动作是 `v5_straight`、`v5_left`、`v5_right`、`v5_start`、`v5_stop`，
各约 400 帧、50 Hz。最终库 SHA256 为
`6f9cefe15646b532ccf57a40f20292c69c18f77ae0c33254abdedfcd1c66a8d0`。
这代表既有实验的确切输入，不保证不同机器重新优化后得到逐位相同的轨迹。

## 1. 取得源输入，保留身份

LAFAN1 由 [Ubisoft La Forge](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)发布。
本实验使用 [Orange Duck 的 resolved/Geno 版本](https://theorangeduck.com/page/animation-data)：
`walk1_subject1.bvh`、`walk2_subject1.bvh`，而不是把原 LAFAN 骨架直接当 Geno。

```bash
python scripts/download_lafan_walks.py
python scripts/select_lafan_segments.py
python scripts/fetch_reference_inputs.py --local-use
```

下载器使用 HTTP Range 只提取两个 BVH，拒绝默默下载整个约 1 GB 包。
`data/sources/lafan1-resolved/selected_segments.json` 记录原文件哈希、帧区间和源 fps。
片段选择从真实连续动作中寻找直走、转弯、起步和停止，不凭空生成根轨迹。
Geno FBX/stance 从固定 tracker 提交获取并校验；请同时保留 Geno 声明。
这些下载文件都受 `.gitignore` 排除。

源 fps 约 59.9988，机器人输出为 50 Hz。位置、SO(3) 与时间重采样应保持一致，
禁止只缩放角速度或外推末帧以凑时长。BVH 坐标系是 Y-up；机器人为 Z-up，
脚本明确处理基坐标变换、腿长缩放及根方向常量对齐。

## 2. 单独安装 UMR 环境

```bash
python3.12 -m venv .venv-umr
source .venv-umr/bin/activate
python -m pip install -r requirements-retarget.txt
```

UMR 使用 MuJoCo 3.3.7，训练使用 3.11.0；两个环境分开。
另外需要 Node.js >=18。原生 UMR 与最小 FBX/Three.js 解析依赖已保留在
`third_party/umr/`，各自许可证一并保留。不下载 SMPL 人体参数或其他机器人的数据作为替代。

先按主安装说明获取机器人网格。`configs/umr/opensprite.json` 指向 Sprite 的实际模型，
使用以 pelvis 为中心的表面和新的 `geno/opensprite0825` 对应关系身份。

## 3. 训练对应关系、做原生重定向

```bash
python scripts/retarget_lafan.py --stage build \
  --motions v5_straight v5_left v5_right v5_start v5_stop \
  --output-dir outputs/umr-v5-v3
python scripts/retarget_lafan.py --stage train \
  --motions v5_straight v5_left v5_right v5_start v5_stop \
  --output-dir outputs/umr-v5-v3
python scripts/retarget_lafan.py --stage retarget \
  --motions v5_straight v5_left v5_right v5_start v5_stop \
  --output-dir outputs/umr-v5-v3
python scripts/retarget_lafan.py --stage convert \
  --motions v5_straight v5_left v5_right v5_start v5_stop \
  --output-dir outputs/umr-v5-v3 --motion-output-dir data/motions/native_v5_v3
```

对应关系训练使用 GPU，原生动作优化使用 CPU。仅做 retarget 不会偷偷启动 GPU 对应关系训练；
身份不匹配会拒绝。`--max-frames` 可做局部检查，但正式转换禁止把局部结果冒充完整参考库。

本地旧对应关系可复用的前提是独立证明中心化网格、轴、关节变换与范围兼容；
不能因为两个机器人名字相似就沿用对应关系或关节索引。

## 4. 接触约束轨迹优化

单纯关节平滑不能保证脚不滑。该阶段使用源动作的支撑/抬脚信息，对整段机器人轨迹做
接触锚点、脚底高度、关节范围/速度、自碰撞与根部轨迹约束优化。
根方向来自真实 Hips SO(3)，不是为了“能转弯”额外合成偏航。

回到训练环境运行接触后处理（优化器与训练 GPU 版本保持隔离）：

```bash
source .venv/bin/activate
for name in v5_straight v5_left v5_right v5_start v5_stop; do
  python scripts/refine_reference_contacts.py --name "$name" \
    --input-dir data/motions/native_v5_v3 --native-dir outputs/umr-v5-v3 \
    --output-dir data/motions/contact_v5_rebuild
done
python scripts/pack_contact_references.py --folder data/motions/contact_v5_rebuild
```

原 v11 是多次优化、检查与保存迭代后选定的可行轨迹。新优化未收敛时应分析报告，
不能放松 QA 阈值让它通过。`finalize_contact_candidate.py` 保留了原历史保存迭代的验收方法，
它依赖原 `refinement-debug` 状态，不是新用户从零执行的必要步骤。

## 5. QA 与实际看图

`pack_contact_references.py` 会做完整参考库 QA 并渲染预览。
主要门槛包括：

- 双脚真实抬起事件；直走、左右转向含义与源动作一致。
- 脚底不穿地超过 2 mm，非脚部碰撞体不穿地超过 2 mm，自穿透不超过 1 cm。
- 支撑脚 RMS 平移速度不超过 0.08 m/s，关节速度不超过原机器人范围容差。
- 根位置/速度/加速度与重采样一致，机器人 pelvis 高度合理。
- 新资产 FK 与参考的身体位置、旋转最大误差小于 `1e-4`。

数值 QA 通过后，逐段看 `previews/*.png` 的脚接触、抬脚、交叉碰撞及根运动。
真实看完才能创建独立视觉评审 JSON，再用 `scripts/record_reference_review.py` 绑定
参考库与预览的哈希；不要直接把 `visual_review_passed` 改成 true。
评审 JSON 的 `reviewed_candidates` 每项包含 `motion`、`preview_file`、`sha256` 和 `passed`，
顶层包含 `motion_bank_sha256`、`reviewed_at`、`passed`。

```bash
python scripts/record_reference_review.py \
  --quality data/motions/contact_v5_rebuild/v5_locomotion.qa.json \
  --review /path/to/your-actual-visual-review.json
```

新库通过后，将训练命令的 `--motion` / `--quality` 指向新目录。
新库的参考哈希与 checkpoint 绑定；不应把它用于原 run002 checkpoint 并假装是同一次实验。
