# RC_Legged_Training_Simulation

## 山东华宇工学院16DOF轮足机器人算法开源仓库

本目录整理自 `RC_WheelLeg` 轮足机器人开源项目，用于 RM 论坛开源和 GitHub/Wiki 说明。内容聚焦强化学习训练、MuJoCo 仿真、Sim2Sim 策略验证和部署前接口检查；实机部署、ROS 2/C++ 控制主线、比赛任务集成和打点工具见项目 [`RC_Legged_Control/`](https://github.com/Dichen33/)。全开源完整项目入口 [RC_WheelLeg 轮足全开源](https://github.com/zeitvex/RC_WheelLeg)。

本整理包采用 MIT 开源协议，协议正文见 [`LICENSE`](LICENSE)。第三方依赖、mjlab、MuJoCo、PyTorch、CUDA、网格资源和示例策略权重仍需遵守其各自许可证或授权要求。

## 软件功能

本工程面向 16DOF 串联轮足机器人，覆盖从机器人描述、强化学习任务、策略训练到仿真回放的主要链路：

- MJCF 机器人模型：`rc_mjlab/mjcf/wheelleg.xml`、`scene.xml` 和 `meshes/` 中的 STL 网格。
- mjlab 训练任务：注册 `Robot-Flat-v0`、`Robot-Rough-v0`、`Robot-Crawl-v0` 三类任务。
- PPO 强化学习：包含 actor/critic 网络配置、奖励项、课程、随机化、动作滤波和扰动项。
- MuJoCo 独立仿真：`mujoco_sim/` 用于运动学、动力学和 MPC 调试。
- Sim2Sim 验证：`sim2sim/` 用于策略权重回放、地形验证和交互式导航观察。
- 示例权重：`model_rough.pt` 与 `model_crawl.pt` 用于本地接口回放和策略链路检查。

## 依赖环境

推荐环境：

| 类型 | 建议配置 |
|---|---|
| 操作系统 | Ubuntu 22.04 或其他 Linux 发行版 |
| Python | 3.10 或 3.11 |
| GPU | NVIDIA GPU，CUDA 环境可用 |
| 包管理 | `uv` |
| 主要依赖 | `mjlab[cu128]`、MuJoCo、PyTorch、warp-lang、pynput |
| 图形环境 | 运行 MuJoCo viewer、Pygame 面板时需要本地图形界面或正确配置的远程显示 |

依赖版本由 `rc_mjlab/pyproject.toml` 和 `rc_mjlab/uv.lock` 约束。`mjlab` 以本地 editable 依赖方式引用 `rc_mjlab/mjlab/`，不要只复制单个训练脚本运行。

## 安装方式

```bash
cd rc_mjlab
uv sync
```

如果本机 CUDA、PyTorch 或 MuJoCo 版本与锁定文件不兼容，需要先按本机环境调整 `pyproject.toml` 中的索引和 CUDA extra，再重新同步依赖。

## 目录结构

```text
RC_Legged_Training_Simulation/
├─ LICENSE                         # MIT 开源协议
├─ README.md                       # 本说明文档
└─ rc_mjlab/
   ├─ README.md                    # 原训练工程说明
   ├─ pyproject.toml               # Python 项目依赖
   ├─ uv.lock                      # 锁定依赖版本
   ├─ model_rough.pt               # Rough 示例策略权重
   ├─ model_crawl.pt               # Crawl 示例策略权重
   ├─ mjlab/                       # 本地 mjlab 依赖
   ├─ mjcf/                        # 机器人 MJCF、场景和 STL 网格
   ├─ src/robot/                   # 训练任务、奖励、课程、随机化和 PPO 配置
   ├─ sim2sim/                     # 策略回放、地形仿真和交互式导航
   └─ mujoco_sim/                  # 独立 MuJoCo/MPC 调试工具
```

## 系统框图与数据流

```text
MJCF / meshes / terrain
        |
        v
   MuJoCo physics
        |
        +--> mjlab ManagerBasedRlEnv
        |        |
        |        +--> observations / rewards / curriculum / events
        |        |
        |        +--> PPO trainer -> checkpoints (*.pt)
        |
        +--> mujoco_sim standalone debugging
        |
        +--> sim2sim policy replay -> deployment interface check
                                      |
                                      v
                       RC_Legged_Control real robot stack
```

训练侧产物主要是策略 checkpoint 和与策略一致的接口约定。实机部署侧需要再次核对观测顺序、动作维度、关节顺序、零位、方向、动作缩放、控制频率和安全限幅。

## 强化学习训练

```bash
cd rc_mjlab

# 平地基础训练
uv run train Robot-Flat-v0

# 多障碍粗糙地形训练
uv run train Robot-Rough-v0

# 爬坡和限高任务训练
uv run train Robot-Crawl-v0

# 回放策略
uv run play Robot-Rough-v0
```

任务注册位于 `src/robot/__init__.py`。主要配置位置：

- `src/robot/config/env_cfgs.py`：环境、观测、奖励、事件、地形和终止条件。
- `src/robot/config/rl_cfg.py`：PPO 网络结构、学习率、折扣因子、KL 目标和迭代次数。
- `src/robot/robot_cfg.py`：机器人 MJCF 读取、执行器和物理实体配置。
- `src/robot/mdp/rewards.py`：速度跟踪、姿态、接触、能耗、动作平滑等奖励项。
- `src/robot/mdp/curriculums.py`：地形课程和速度指令自适应课程。
- `src/robot/mdp/lowpass_actions.py`：腿部位置动作、轮部速度动作的低通滤波与延迟建模。
- `src/robot/mdp/disturbances.py`：持续外力和力矩扰动。
- `src/robot/terrains/competition_terrains.py`：比赛相关障碍地形生成。

## Sim2Sim 验证

```bash
cd rc_mjlab
uv run python sim2sim/sim2sim.py
uv run python sim2sim/nav_sim2sim.py
```

`sim2sim.py` 默认加载本包内的 `model_rough.pt`。`nav_sim2sim.py` 同时支持 Rough 与 Crawl 示例权重，并提供交互式导航与地形回放。运行前请确认图形界面、MuJoCo 渲染和 Pygame 可用。

## MuJoCo/MPC 调试

```bash
cd rc_mjlab
uv run python mujoco_sim/run.py
```

`mujoco_sim/` 使用 `mujoco_sim/config.py` 中的相对路径读取 `mjcf/scene.xml` 和 `mjcf/wheelleg.xml`，适合在不启动完整训练框架的情况下检查机器人模型、姿态、控制器和 MPC 思路。

## 原理与技术说明

本项目采用“训练任务配置化、物理接口统一、部署前 Sim2Sim 检查”的思路。

1. 机器人模型由 MJCF 描述，MuJoCo 负责物理步进、碰撞和传感器状态。
2. mjlab 的 `ManagerBasedRlEnv` 将观测、动作、奖励、事件、课程和地形拆成可组合模块。
3. PPO 使用 actor/critic 结构训练策略，默认网络规模为 `(512, 256, 128)`，控制频率为 50 Hz，物理步长为 2 ms。
4. 训练中加入摩擦、质心、编码器偏置、执行器刚度/阻尼、力矩上限、负载质量、推撞和持续外力等随机化，提高从仿真到实机的鲁棒性。
5. Sim2Sim 使用相同 MJCF 和策略权重进行 MuJoCo 回放，重点检查策略输入输出、地形行为、姿态稳定性和控制接口，而不是替代真机安全测试。

## 软件效果展示与量化指标

本整理包提供可复现实验入口，不预置新的测试结论。建议在论坛发布时补充以下可视化材料：

- `uv run play Robot-Rough-v0` 的回放录屏或 GIF。
- `sim2sim/nav_sim2sim.py` 的比赛地形回放视频。
- 训练曲线：episode reward、速度跟踪误差、姿态误差、关节力矩、动作变化率。
- Sim2Sim 指标：通过率、最大 roll/pitch、平均速度、碰撞次数、任务完成时间。
- 与 `RC_Legged_Control` 实机视频对应的部署前后对照说明。

可在 README 或 GitHub Wiki 中新增“实验记录”页面，避免主 README 过长。

## 软件架构与层级

```text
Application entry
  train / play / sim2sim / mujoco_sim

Task layer
  Robot-Flat-v0 / Robot-Rough-v0 / Robot-Crawl-v0

Configuration layer
  env_cfgs.py / rl_cfg.py / robot_cfg.py

MDP components
  commands / rewards / actions / disturbances / curriculums / terrains

RL backend
  RSL-RL style runner / PPO / actor-critic / rollout storage

Physics backend
  MuJoCo / MJCF / meshes / terrain assets
```

## 代码规范与设计模式

### 开源协议

本整理包采用 MIT License。协议文件位于 [`LICENSE`](LICENSE)，README 顶部也明确了授权范围。第三方依赖和资源按照各自许可证执行。

### 命名与注释

Python 代码整体使用 snake_case 函数与变量命名，配置类使用 PascalCase，例如 `JointPositionLowPassActionCfg`、`UniformThresholdVelocityCommandCfg`。关键接口在动作滤波、环境 wrapper、奖励函数和地形生成处保留了类型标注或说明性注释。

### 可运行与测试

本包提供训练、回放、Sim2Sim 和独立 MuJoCo 调试入口。由于依赖 CUDA、MuJoCo 和图形环境，训练前建议至少完成以下环境检查：

```bash
cd rc_mjlab
uv sync
uv run play Robot-Flat-v0
uv run python sim2sim/sim2sim.py
uv run python mujoco_sim/run.py
```

### 设计模式

项目主要遵循以下设计模式和架构习惯：

- 配置对象模式：`env_cfgs.py`、`rl_cfg.py`、`robot_cfg.py` 将训练参数、物理实体和 PPO 参数集中定义，避免将常量散落在训练流程中。
- 注册表模式：`src/robot/__init__.py` 通过 `register_mjlab_task` 注册 `Robot-Flat-v0`、`Robot-Rough-v0`、`Robot-Crawl-v0`，训练入口只需按任务 ID 调用。
- 组合式 MDP 模式：奖励、命令、动作滤波、扰动和课程分别拆成独立模块，在环境配置中组合，便于替换或裁剪单个能力。
- Builder/Factory 风格：动作配置类的 `build()` 方法根据环境构造具体动作处理对象，如低通滤波动作和延迟动作。
- Wrapper/Adapter 模式：`HimMjlabVecEnvWrapper` 将 mjlab 环境适配到 RSL-RL 风格接口，降低训练后端与环境实现的耦合。
- Strategy 模式：Flat、Rough、Crawl 任务共用主框架，但通过不同环境配置、奖励权重和地形课程形成不同训练策略。

这些模式的核心目标是让“机器人模型、训练任务、策略算法、仿真验证”彼此解耦，方便其他队伍按自己的机械结构、传感器和比赛任务替换局部模块。

## 复现与安全注意事项

- 示例权重仅用于接口验证和回放，不代表任何实机性能承诺。
- 仿真通过不等于可以直接上电，真机调试必须架空机器人并确认急停链路。
- 训练和回放结果受 GPU、CUDA、MuJoCo、PyTorch、mjlab、随机种子和地形配置影响。
- 迁移到其他机器人时必须重新核对 MJCF、关节顺序、动作尺度、观测维度和执行器限幅。

## Roadmap

- 补充标准化实验记录模板和训练曲线导出脚本。
- 增加更轻量的 CPU-only 或无渲染 smoke test。
- 将 Sim2Sim 指标统计为 CSV，便于与真机测试记录对照。
- 把观测/动作接口生成部署契约文件，减少训练与实机部署不同步。
- 将 README 中的长篇原理内容拆分到 GitHub Wiki，保留主 README 为快速入口。