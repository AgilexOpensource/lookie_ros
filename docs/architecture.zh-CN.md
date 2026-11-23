# lookie_ros 整体流程框架

[English](architecture.md)

本文以 `trio` 命名空间为例（Neo 机型将 `/trio` 换成 `/neo`），描述 lookie_ros 各功能包、依赖仓库、启动流程与数据流的关系。

## 1. 分层架构

```mermaid
flowchart TB
    subgraph USER["操作层"]
        HMD["XR 头显 / 手柄<br/>(浏览器 https://&lt;IP&gt;:9443)"]
        VIEWER["kstone_viewer<br/>(调试, 可选)"]
        RVIZ["RViz (可选)"]
    end

    subgraph BRINGUP["lookie_bringup：整机启动与机型配置"]
        MODEL["LOOKIE_MODEL<br/>config/&lt;model&gt;/bringup.yaml"]
        L1["lookie.launch.py"]
        L2["kstone.launch.py"]
        L3["teleop.launch.py"]
    end

    subgraph APP["应用层"]
        XR["xr_media_ros<br/>/xr_media"]
        TELEOP["lookie_teleop<br/>/trio/lookie_teleop"]
    end

    subgraph CTRL["控制层：lookie_control"]
        KSTONE["kstone_ros 节点 /trio/kstone<br/>(kstone C++ 全身 IK 引擎)"]
    end

    subgraph DRV["驱动层：lookie_driver"]
        BRIDGE["joint_bridge<br/>关节反馈合并 / 命令分发 (50 Hz)"]
        HEAD["head_ros<br/>2DoF 头部"]
        TORSO["fold_ros (Trio) / lift_ros (Neo)<br/>躯干"]
        ARM["agx_arm_ros ×2<br/>左右臂 + 夹爪/Revo2 手"]
        CHASSIS["lookie_neo_base / lookie_trio_base + ugv_sdk<br/>差速（Neo）/ 全向（Trio）底盘"]
    end

    subgraph DESC["描述层"]
        URDF["lookie_description<br/>URDF / collision URDF / initial"]
        RSP["robot_state_publisher → TF"]
    end

    SIM["关节滑块<br/>(sim:=true, 代替驱动,<br/>发布 driver/feedback/joint_states)"]

    HW[("硬件 (CAN 总线)<br/>can_bind.sh 启用接口")]

    MODEL --> L1 & L2 & L3
    L1 --> DRV
    L1 --> RSP
    L1 -.sim:=true.-> SIM
    SIM -.-> KSTONE
    SIM -.-> RSP
    L1 -.rviz:=true.-> RVIZ
    L2 --> KSTONE
    L3 --> XR & TELEOP

    HMD <--> XR
    XR --> TELEOP
    TELEOP --> KSTONE
    KSTONE --> BRIDGE
    KSTONE --> CHASSIS
    BRIDGE --> KSTONE
    BRIDGE <--> HEAD & TORSO & ARM
    HEAD & TORSO & ARM & CHASSIS <--> HW
    URDF --> RSP
    URDF --> KSTONE
    BRIDGE --> RSP
    RSP --> RVIZ
    KSTONE -.debug_port.-> VIEWER
```

## 2. 启动流程

```mermaid
flowchart LR
    S0["安装<br/>vcs import lookie.repos<br/>colcon build"] --> S1["设置 LOOKIE_MODEL"]
    S1 --> S2["ros2 run lookie_bringup<br/>can_bind.sh"]
    S2 --> S3["终端1<br/>lookie.launch.py<br/>驱动 + robot_state_publisher<br/>(sim:=true 用关节滑块代替驱动)"]
    S3 --> S4["终端2<br/>kstone.launch.py<br/>收到新鲜反馈后保持当前位姿"]
    S4 --> S5["终端3<br/>teleop.launch.py<br/>xr_media_ros + lookie_teleop"]
    S5 --> S6["头显打开 URL<br/>信任证书, 进入 VR"]
```

每个 launch 启动前都会读取 `bringup.yaml`，解析其中的 `package://`/相对路径，并对照 URDF 校验驱动配置：

```mermaid
flowchart LR
    ENV["LOOKIE_MODEL / model:=..."] --> BY["config/&lt;model&gt;/bringup.yaml"]
    BY --> NS["namespace: trio / neo"]
    BY --> D["description:<br/>urdf / collision_urdf / initial"]
    BY --> DR["driver:<br/>driver_motors.yaml<br/>driver_ros.yaml<br/>driver_joint_bridge.yaml"]
    BY --> K["kstone:<br/>kstone.yaml"]
    BY --> T["teleop:<br/>xr_media.yaml<br/>common.yaml + teleop.yaml"]
    DR --> V{"对照 URDF 校验"}
    D --> V
    V -->|通过| RUN["启动节点"]
    V -->|失败| ERR["报错退出"]
```

## 3. 运行时数据流（ROS 话题）

```mermaid
flowchart LR
    subgraph XRN["/xr_media"]
        XT["*/transform<br/>TransformStamped"]
        XB["{left,right}_controller/buttons<br/>Joy"]
        XH["{left,right}_controller/haptic<br/>Float32MultiArray"]
    end

    TEL["lookie_teleop<br/>标定 / 映射 / 安全 / 追赶"]

    subgraph KS["/trio/kstone"]
        KD["discover · state<br/>(latched JSON)"]
        KP["&lt;M&gt;/control/pose"]
        KJ["control/joint"]
        KV["chassis/control/cmd_vel"]
        KC["chassis/control/pose<br/>(仅全向底盘, Trio)"]
        KH["&lt;M&gt;/home (srv)"]
        KF["&lt;M&gt;/feedback/pose"]
        ENG(("kstone 引擎<br/>全身 IK + 碰撞/质心/零空间"))
    end

    subgraph DV["/trio/driver"]
        FB["feedback/joint_states"]
        CT["control/joint_states"]
        JB["joint_bridge"]
        MODS["head / torso /<br/>left_arm / right_arm"]
        CV["chassis/cmd_vel"]
        OD["chassis/odom<br/>TF odom→base_footprint"]
        SC["chassis/lookie_trio_base"]
    end

    XT & XB --> TEL
    TEL --> XH
    KD --> TEL
    KF --> TEL
    TEL --> KP & KJ & KV
    TEL -.-> KC
    TEL --> ST["/trio/lookie_teleop/status"]
    TEL -.-> KH
    KP & KJ & KV & KC & KH --> ENG
    ENG --> KD & KF
    FB -->|关节反馈| ENG
    OD -->|里程计| ENG
    ENG -->|关节命令| CT
    ENG -->|速度命令| CV
    CT --> JB --> MODS
    MODS --> JB --> FB
    CV --> SC --> OD
```

## 4. 控制闭环

```mermaid
sequenceDiagram
    participant H as XR 头显/手柄
    participant X as xr_media_ros
    participant T as lookie_teleop
    participant K as kstone_ros
    participant B as joint_bridge
    participant D as 各模组驱动/底盘
    participant HW as 硬件 (CAN)

    K-->>T: discover / state (latched)
    loop 50 Hz 反馈
        HW->>D: 电机状态
        D->>B: 各模组 joint_states
        B->>K: driver/feedback/joint_states
        D->>K: driver/chassis/odom
    end
    H->>X: 头显、手柄位姿与按键
    X->>T: */transform, */buttons
    T->>T: 标定、坐标映射、安全检查
    T->>K: 位姿/关节/速度目标, home 服务
    K->>K: 全身 IK 求解
    K->>B: driver/control/joint_states
    B->>D: 按模组拆分关节命令
    K->>D: driver/chassis/cmd_vel
    D->>HW: 电机/底盘指令
    T-->>X: haptic 震动反馈
```

## 5. 功能包与依赖仓库

```mermaid
flowchart LR
    BR["lookie_bringup"] --> DRV["lookie_driver"]
    BR --> CTL["lookie_control"]
    BR --> TEL["lookie_teleop"]
    BR --> LD["lookie_description"]
    CTL --> KR["kstone_ros"] --> KE["kstone"]
    CTL --> LD
    TEL --> XMR["xr_media_ros"]
    DRV --> HR["head_ros"]
    DRV --> LR["lift_ros (Neo)"]
    DRV --> FR["fold_ros (Trio)"]
    DRV --> AR["agx_arm_ros"]
    DRV --> SR["lookie_neo_base / lookie_trio_base"] --> UG["ugv_sdk"]
```

## 6. 反馈新鲜度与超时链

kstone 只在反馈新鲜时才向驱动输出命令。新鲜度逐层判断，每层超时后只停用对应的模组或部位，其他部分照常运行。

```mermaid
flowchart LR
    HW["硬件 (CAN)"] -->|"0.5 s"| D["驱动"]
    D -->|"关节 0.15 s"| B["joint_bridge"]
    B -->|"0.25 s"| K["kstone_ros"]
    D -->|"odom 0.25 s"| K
    K -->|"~/state 0.3 s"| T["lookie_teleop"]
```

| 层 | 判断依据 | 超时后 |
|---|---|---|
| 驱动 | 硬件数据是否仍在更新 | 停发该模组的反馈 |
| joint_bridge | 各模组反馈的接收时刻 | 合并反馈中去掉该模组 |
| kstone | 各反馈组的时间戳 | 该组脱离控制；重新新鲜一段时间后自动接入 |
| lookie_teleop | `~/state` 的接收时刻及各组状态 | 只停受影响的部位；`~/state` 超时则整体失能（见[遥操作设计 §12](../lookie_teleop/docs/vr_teleop_dev_design.zh-CN.md)） |

- **延迟：** 判定为掉线之前，系统按最后收到的数据运行，最长约 1 s。
- **约束：** 关节桥超时不大于 kstone 的超时（启动时检查，见[回归测试](regression.zh-CN.md)）；各级超时应明显大于对应的发布周期。
- **观察：** `/trio/kstone/state` 的 `groups`、`fault`，`/trio/lookie_teleop/status`，以及驱动日志中的 `feedback lost / recovered`。
- 仿真没有底盘里程计，`base` 组始终不新鲜，属预期。
