# 代码、模型与数据的来源

仓库保持创建时的 [AGPLv3 许可证](LICENSE)，覆盖项目代码；它不会改变外部数据、
角色资产或保留第三方许可证的软件的条款。

| 内容 | 来源及处理 |
|---|---|
| 训练基础 | `lzjever/opensprite-tracker`，固定提交 `4beb018567574215b5c9f95e0cc13cf5e04012d3`，保留 AGPL |
| 机器人 | `BeijingDynamics/open_sprite`，腰部模型提交 `f42a266c860db70d73ce92f34e26d253a2659636`，控制提交 `05774a9a37f47ed8a66f82ed36324e7f712bdfcb` |
| runtime | `BeijingDynamics/open_sprite_runtime`，提交 `5f8225ce6258cb38db174ee18c55162fa34d96a6`，AGPLv3 |
| UMR | `third_party/umr/` 保留原 MIT LICENSE 与贡献者信息 |
| Three.js 等 | 保留 `third_party/umr/scripts/vendor/` 中的各自声明 |
| TDK IMU 头文件 | runtime 快照中的 `native/third_party/` 保留原声明 |
| LAFAN1 | Ubisoft La Forge；[官方仓库](https://github.com/ubisoft/ubisoft-laforge-animation-dataset)与[许可证](https://github.com/ubisoft/ubisoft-laforge-animation-dataset/blob/master/license.txt)注明 CC BY-NC-ND 4.0 |
| Geno 角色 | 单独的原始声明保留在 `assets/geno/license.txt`；角色文件不入库 |

按照项目所有者要求，Git 和 GitHub Release 都不包含 LAFAN1 原始动作、裁剪动作、
重定向后的 NPZ、UMR 对应关系数据/权重或参考动作录像。已生成的数据保存在原本地工作区，
不因租用服务器到期而丢失。源数据可以从原发布方取得，在本地按
[动作生成说明](docs/REFERENCE_MOTIONS.md)处理；使用与再分发应遵守源数据条款。

公开的数值 QA、源片段索引与哈希属于复现元数据，不包含逐帧参考姿态。
公开录像和轨迹仅来自速度命令驱动的策略物理仿真；不以参考动画替代模型表现。
训练权重注明其训练数据来源，不把项目代码许可证解释为源数据的重新授权，
也不声称商业使用已经获得外部数据许可。

本项目的 31 自由度 Sprite、控制频率、执行器映射和速度范围与论文 G1 实验不同。
所有性能结论以本仓库实际配置与记录为准，没有声称获得论文原机器人基准成绩。
