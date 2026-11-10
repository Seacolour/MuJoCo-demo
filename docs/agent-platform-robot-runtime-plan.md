# Agent 平台与机器人运行时拆分计划

## 目标与当前状态

这个项目要展示自然语言规划、平台任务调度与机器人技能执行如何协作：用户在遥控台提交指令，DeepSeek 生成受约束的任务步骤，平台逐步调度，ROS 2 将单步目标交给持续运行的 MuJoCo xArm7，物理结果回流后决定继续、失败或取消。

目前已有浏览器遥控台、双物体三区域场景、最多四步的 `pick_place` 计划、执行前占用检查、跨节点阶段反馈与取消。单步执行仍使用 ROS 2 Topic；九个机械臂动作是确定性的顺序流程。项目尚未使用 LangGraph、行为树或自定义 ROS 2 Action，任务历史在平台进程重启后会丢失。

## 架构决定

保留一个 Git 仓库，整理成两个可独立启动的运行模块和一个轻量共享契约模块。当前的两个进程边界继续保留；这次不拆成两个仓库，也不因目录调整重写已验证的抓取控制。

```text
浏览器遥控台
    │ HTTP 提交／取消，SSE 状态
    ▼
Agent 平台进程
    DeepSeek 规划 → 整单校验 → 父任务状态与队列 → 单步技能客户端
    │                     ▲
    │ ROS 2 Goal          │ Feedback／Result／场景快照
    ▼                     │
机器人运行时进程
    ROS 2 技能服务端 → 确定性执行与局部验收 → MuJoCo xArm7／Viewer
```

### 模块所有权

| 模块 | 持有的状态与职责 | 不应依赖 |
| --- | --- | --- |
| Agent 平台 | HTTP/SSE、DeepSeek、`TaskPlan`、整单校验、父任务队列、步骤调度、事件记录和用户取消 | MuJoCo 的 `MjData`、关节控制细节 |
| 机器人运行时 | 唯一的 MuJoCo 模型与物理状态、ROS 2 技能服务端、抓取控制、接触验收、单步取消 | 自然语言提示词、浏览器状态 |
| 共享契约 | 物体与区域 ID、任务 Goal、阶段 Feedback、执行 Result、错误码和场景版本 | FastAPI、模型 SDK、MuJoCo 实现 |

`pick_place(object_name, target_name)` 是当前唯一的运动技能。平台把每个有序步骤映射为一次技能调用；机器人运行时负责接近、下降、夹取、抬升、搬运、释放和物理验收。`observe_scene` 属于只读能力。未来加入 `move_home` 等技能时，分别定义输入、前置条件、占用的机器人资源、反馈和完成判据；模型不能直接输出 `data.ctrl` 或关节轨迹。

## 实施阶段

### 阶段 0：冻结当前可运行基线

状态：已完成。拆分前记录了 21 个 Python 测试通过，`smoke-multi` 与 `smoke-plan` 均通过。

- 保留单方块基线、双物体六种组合、两步复合指令、冲突整单拒绝和取消用例。
- 记录 `pixi run test`、`pixi run --manifest-path ros2/pixi.toml smoke-multi` 和 `smoke-plan` 的结果。模块迁移时先保证行为与界面结果一致。

完成条件：重构前后的任务结果、场景版本与失败原因一致。

### 阶段 1：在同仓库整理模块

状态：代码边界已整理，浏览器操作待手动复核。平台、机器人运行时与共享契约已归入独立 Python 包；原有脚本保留兼容入口。拆分后 21 个 Python 测试与两步 ROS 2 任务检查通过。

- 将 `ros2/control_platform.py` 中的 HTTP 服务、Planner 与 Task Runtime 整理到 `platform/`；ROS 2 客户端保留为窄适配层。
- 将 `ros2/bridge_node.py`、`pick_place.py`、IK 与场景目录整理到 `robot_runtime/`；只有这一侧修改 MuJoCo 物理状态。
- 将 `TaskPlan`、技能输入输出与场景快照约定放入共享契约目录。依赖方向为“平台／机器人运行时 → 共享契约”，不出现反向导入。
- 保持两个启动命令和现有 UI 可用；先做纯目录与导入边界调整，不改任务语义。

当前包路径为 `mujoco_demo.platform`、`mujoco_demo.robot_runtime`、`mujoco_demo.shared`。平台的 ROS 2 Topic 收发集中在 `platform/ros_skill_client.py`，任务调度不直接操作 Topic 消息；机器人运行时只依赖共享契约。两个进程目前仍共用 `ros2/pixi.toml` 环境，依赖安装暂未拆开。

完成条件：现有物理测试、ROS 2 连续任务检查和浏览器两步指令仍通过。

### 阶段 2：把单步技能迁移到 ROS 2 Action

- 定义 `PickPlace.action`。Goal 至少包含请求 ID、物体 ID、目标区 ID 与预期场景版本；Feedback 报告动作阶段；Result 返回成功标志、失败码、原因、最终物体位置和场景版本。
- 机器人运行时一次只执行一个 Goal；取消请求传到执行循环，停止当前动作并返回 `CANCELED`，不报告成功。平台继续持有多步骤父任务和调度顺序。
- 先保留 Topic 适配作回归对照；验证 Action 的接受、反馈、结果、取消与重复请求后，再收敛旧命令 Topic。

完成条件：单步 Goal 可跨进程执行并返回物理验收；两步指令仅在两次 Action 都成功后完成；取消与失败会停止后续步骤。

### 阶段 3：任务恢复与有条件重规划

- 用 SQLite 持久化父任务、步骤与事件。平台重启后保留历史；对运行中断的任务先读取机器人场景与 Action 状态，再决定恢复、重试或标记失败，避免重复搬运。
- 选一个具体失败场景做有限恢复，例如夹爪闭合后方块未抬起：观察新状态 → 改变抓取偏移并重试一次 → 仍失败则上报 Planner。
- 仅当条件分支、暂停恢复或状态持久化让现有 Task Runtime 明显复杂时，在 Agent 平台内试用 LangGraph。图节点可对应观察、规划、校验、技能调用、验证与有限重规划；图检查点不能替代 MuJoCo 世界状态，恢复前仍要比对场景版本。

完成条件：失败不会被误报为成功；重启后的任务状态可解释；重复指令不会重复执行已完成的物理步骤。

### 阶段 4：按需要评估行为树

当前顺序执行器不能称为行为树。若单个 `pick_place` 技能需要局部条件、Fallback 和有限重试，再在机器人运行时选一个技能试做行为树；比较它与现有状态机的可读性、取消行为和测试结果。平台的多步骤规划与父任务状态仍由平台负责。

## 验收与对外表述

- 一句话两步任务在页面上显示两个步骤，各自有反馈和结果；第二步失败时父任务为失败，并保留第一步已完成的事实。
- “红方块到蓝区，然后绿方块也到蓝区”在物理运动前整单拒绝；目标占用、未知实体、取消和超时都有明确原因。
- 同一 MuJoCo 场景连续执行，另一块方块没有被误碰走；平台页面与 Viewer 的状态一致。
- 简历中的框架与协议名称以实际完成为准：当前可写 DeepSeek 规划、平台任务调度、ROS 2 Topic 桥接和 MuJoCo 物理闭环；完成相应验收后再写 LangGraph、行为树或 ROS 2 Action。

参考：[LangGraph 状态化工作流](https://docs.langchain.com/oss/python/langgraph/overview)、[LangGraph 持久化](https://docs.langchain.com/oss/python/langgraph/persistence)、[ROS 2 Action](https://github.com/ros2/ros2_documentation/blob/rolling/source/ROS-Framework/interfaces/About-Actions.rst)。
