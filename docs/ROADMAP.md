# ROADMAP · lerobot_robot_episode1 深度规划（2026-07-31）

> 本文档是「替代恩培 fork、统一到官方 lerobot」这条线的深度研究与阶段规划。
> 研究基于：官方 lerobot 0.6.0/0.6.1 源码、恩培 fork（基点 91b110d8）全量 diff、厂商教程 2.2–2.10。

## 一、深度研究结论（插件包方案的边界条件）

### 1. fork 对官方文件的修改，在 0.6 上全部不需要 ✅

- **teleoperate 主循环**：官方 0.6 已无条件每轮 `robot.get_observation()`
  （`lerobot_teleoperate.py:195`），恩培改的「每轮取观测 + None 退出」已是官方行为。
- **record 主循环**：0.6 原生支持 teleop/键盘/策略三路 action，无需任何修改。
- **相机 MJPG**：官方 0.6 的 `OpenCVCameraConfig` 已原生支持 `fourcc="MJPG"` 字段
  （`configuration_opencv.py:52`）——恩培硬写进驱动的 hack 用 config 就能解决：
  `--robot.cameras="{handeye: {type: opencv, index_or_path: 2, width: 320, height: 240, fps: 30, fourcc: MJPG}, ...}"`。
- **train.py 的 pin_memory 改动、modeling_act 的 MPS 回退**：前者与 Episode 无关，
  后者是撤销官方 bug 修复——两者都确认丢弃。

结论：**官方 0.6 + 纯插件**完整覆盖厂商教程 2.4 的全部软件需求，无功能缺口。

### 2. 校准入口有两条，机制必须共用

- 官方通用入口：`lerobot-calibrate --teleop.type=episode1_leader ...`
  调设备的 `calibrate()` 方法。
- 我们的 `episode1-set-middle`：厂商教程的交互式中位校准（Torque_Enable=128 写法）。
- ➡️ 实现要求：`Episode1Leader.calibrate()` 与 set-middle 走**同一套校准代码与存储**
  （官方 calibration_dir + id），两个入口等价，不会出现两套校准数据。

### 3. 上位机侧（Episode1 本体控制程序）的要求——agent 管不着，但要拦住误用

教程 2.4 对上位机有四个前置要求，全部是人的手动操作，插件无法代劳：

1. Episode1 六个驱动板参数 **Response 改为 None**（上位机 GUI 里改）
2. 上位机版本 **≥ V0.9.8**
3. 遥操前用上位机**归零、回默认位置**；夹爪控制盒插上、夹爪先不装
4. 上位机关闭「启用日志」「启用状态刷新」复选框

➡️ 对策：`episode1-doctor` 用只读 TCP 命令（`get_motor_angles`）探测上位机可达性与臂状态，
并在报告中**逐条提醒**上述人工前置；README 真机清单单列一节。

### 4. 双臂（教程 2.5）是另一个 fork，扩展路径已确认

- 2.5 用的是 `enpeizhao/lerobot_two_student`（又一个魔改 fork），双主臂（ttyACM0/ACM1）+ 双从臂。
- 官方 0.6 有 `bi_so_follower` 复合设备模式（一个 Robot 包两个 follower）。
- ➡️ 未来扩展（本期不做）：加 `episode1_bi_follower` 复合 Robot + 第二个 leader 实例，
  主臂按 port/id 区分左右，结构已预留。

### 5. 训练侧（教程 2.6–2.10）零障碍

- 数据集的 action/observation features 来自设备类的 `action_features`/`observation_features`
  （7 维：`joint1..6.pos + gripper.pos`），官方 record/train 对 feature 名无特殊要求。
- ACT / Pi0 训练是官方能力，与设备无关；fork 里那套 openpi-client 不带，
  以后要玩 Pi0 直接装官方 openpi-client。
- ⛔ 训练要走工作区 `tsp` GPU 队列（RTX 5070 Ti 16GB），见工作区 CLAUDE.md / GPU训练队列.md。

## 二、阶段路线图

### Phase 1 · 插件包建成（当前，agent 施工）

产物：`lerobot_robot_episode1` 包 + 无硬件测试全绿 + mock 端到端 teleop 验证。
验收：`pytest` 绿；`register_third_party_plugins` 发现本包；mock 硬件跑通官方 teleoperate。

### Phase 2 · 真机对拍（Jeff 亲手，agent 只准备命令）

1. `episode1-doctor` 全绿（含上位机 TCP、总线 7 颗、相机 2 路）
2. `episode1-set-middle` 中位校准，校准文件落在官方 calibration_dir
3. 低速遥操（`speed_mode=record`）：6 关节跟随 + 夹爪，与教程 2.4 预期输出对拍
4. 验收标准：行为与恩培 fork 一致；异常路径（拔线/超时）按防炸设计表现

### Phase 3 · 相机 + 数据采集

1. 装相机（教程 2.3），`lerobot-find-cameras opencv` 确认 2 路 30fps
2. 带相机遥操：`--robot.cameras="{handeye: {...fourcc: MJPG}, fixed: {...fourcc: MJPG}}"`
3. `lerobot-record` 采第一批数据（教程 2.8 任务一：单臂抓取放置）
4. 验收：数据集 features 完整（7 维 action + 7 维 obs + 2 路图像），回放 `lerobot-replay` 正常

### Phase 4 · 训练与部署（走 tsp 队列）

1. ACT 训练（官方 `lerobot-train`，数据集来自 Phase 3）——GPU 排队，先读 GPU训练队列.md
2. 推理回环：`lerobot-record --policy.path=...` 或官方 eval 脚本
3. （可选）Pi0/openpi 路线：装官方 openpi-client，不引入恩培的 vendored 版本

### Phase 5 · 双臂扩展（教程 2.5，远期）

`episode1_bi_follower` 复合设备 + 双 leader；届时单独立项。

## 三、风险清单

| 风险 | 等级 | 对策 |
|---|---|---|
| 上位机 Response=None 未设置 → 遥操指令时序异常 | 高 | doctor 提醒 + README 清单第 1 条 |
| 主从臂初始姿态差异大 → 启动跳变 | 高 | 插件内首帧 30° 阈值检查（已实现于设计） |
| 官方 lerobot 升级 0.7 改插件 API | 中 | pyproject 钉 `>=0.6,<0.7`；升级前跑测试套件 |
| 恩培更新 fork（修 bug/新功能） | 低 | 我们已接管代码，按需手动挑拣；NOTICE 记录基点 |
| 12.4V 超压长期运行（厂商口径正常） | 低 | 记忆里有案；舵机异常发热时回头查 |
| 教程命令与本包命令不一致 → 照抄教程出错 | 中 | README 对照表；doctor 输出引导到本包命令 |

## 四、已关闭的问题（不再纠结）

- ~~恩培版与官方能否共存~~ → 同名发行包必冲突，已弃 fork
- ~~官方 main 能否支持 Episode~~ → 插件机制实测通过（dummy 端到端）
- ~~相机 MJPG 要不要做自己的相机插件~~ → 官方 0.6 原生 fourcc config
- ~~要不要 conda~~ → venv 全链路验证通过（24.04 + py3.12）
