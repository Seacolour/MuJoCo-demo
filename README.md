# MuJoCo Agent Demo

一个独立于 PaperPuppy 的 Windows 原生机械臂仿真项目。第一阶段的目标是让 Agent 将自然语言指令解析为受约束的抓取任务，并在 MuJoCo 中执行、验证和处理失败。第二阶段再加入 ROS 2 Jazzy 执行接口。

## 技术栈

- Windows 11、Pixi、Python 3.12
- MuJoCo 官方 Python 包与交互式 viewer；Franka Panda 机械臂模型来自 MuJoCo Menagerie
- NumPy 用于运动学计算；Pydantic 校验 Agent 输出；pytest 运行无界面的仿真验收
- LLM 客户端保持可替换；选定服务后再加入其 SDK，不让模型直接输出关节角或控制量

## 第一阶段的目标链路

自然语言 → Agent 生成 `PickPlaceTask` → 场景实体校验 → 抓取任务状态机 → MuJoCo 执行 → 物理状态验证 → 结果反馈给 Agent。

首个演示任务：将指定颜色的方块抓取并放入指定盒子。成功条件包括真实夹爪接触、物体抬升、搬运及释放后在目标区域稳定停留。初期读取仿真状态定位物体，不将其表述为视觉感知。

实现时保留 `RobotBackend` 边界：任务编排只依赖场景观察和技能执行接口。第二阶段用 ROS 2 节点、Topic 和 Action 接入同一编排层。

## 本地环境

```powershell
pixi install
pixi run python -c "import mujoco; print(mujoco.__version__)"
```

机械臂场景、Agent 和执行器尚未实现。实现仿真场景后，使用 `pixi run test` 运行无界面的验收测试。
