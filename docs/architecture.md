# lookie_ros Architecture

[中文](architecture.zh-CN.md)

How the lookie_ros packages, dependency repositories, launch flow and data flow fit together. Examples use the `trio` namespace; for Neo replace `/trio` with `/neo`.

## 1. Layered architecture

```mermaid
flowchart TB
    subgraph USER["Operator"]
        HMD["XR headset / controllers<br/>(browser https://&lt;IP&gt;:9443)"]
        VIEWER["kstone_viewer<br/>(debug, optional)"]
        RVIZ["RViz (optional)"]
    end

    subgraph BRINGUP["lookie_bringup: whole-robot launch and per-model config"]
        MODEL["LOOKIE_MODEL<br/>config/&lt;model&gt;/bringup.yaml"]
        L1["lookie.launch.py"]
        L2["kstone.launch.py"]
        L3["teleop.launch.py"]
    end

    subgraph APP["Application"]
        XR["xr_media_ros<br/>/xr_media"]
        TELEOP["lookie_teleop<br/>/trio/lookie_teleop"]
    end

    subgraph CTRL["Control: lookie_control"]
        KSTONE["kstone_ros node /trio/kstone<br/>(kstone C++ whole-body IK engine)"]
    end

    subgraph DRV["Drivers: lookie_driver"]
        BRIDGE["joint_bridge<br/>merge feedback / split commands (50 Hz)"]
        HEAD["head_ros<br/>2-DoF head"]
        TORSO["fold_ros (Trio) / lift_ros (Neo)<br/>torso"]
        ARM["agx_arm_ros ×2<br/>left/right arm + gripper/Revo2 hand"]
        CHASSIS["scout_ros2 + ugv_sdk<br/>Scout Mini chassis"]
    end

    subgraph DESC["Description"]
        URDF["lookie_description<br/>URDF / collision URDF / initial"]
        RSP["robot_state_publisher → TF"]
    end

    SIM["Joint sliders<br/>(sim:=true, replaces drivers,<br/>publishes driver/feedback/joint_states)"]

    HW[("Hardware (CAN bus)<br/>enabled by can_bind.sh")]

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

## 2. Launch flow

```mermaid
flowchart LR
    S0["Install<br/>vcs import lookie.repos<br/>colcon build"] --> S1["Set LOOKIE_MODEL"]
    S1 --> S2["ros2 run lookie_bringup<br/>can_bind.sh"]
    S2 --> S3["Terminal 1<br/>lookie.launch.py<br/>drivers + robot_state_publisher<br/>(sim:=true uses joint sliders)"]
    S3 --> S4["Terminal 2<br/>kstone.launch.py<br/>holds current pose once feedback is fresh"]
    S4 --> S5["Terminal 3<br/>teleop.launch.py<br/>xr_media_ros + lookie_teleop"]
    S5 --> S6["Open URL in headset<br/>trust certificate, enter VR"]
```

Before starting any node, each launch file reads `bringup.yaml`, resolves its `package://` and relative paths, and checks the driver config against the URDF:

```mermaid
flowchart LR
    ENV["LOOKIE_MODEL / model:=..."] --> BY["config/&lt;model&gt;/bringup.yaml"]
    BY --> NS["namespace: trio / neo"]
    BY --> D["description:<br/>urdf / collision_urdf / initial"]
    BY --> DR["driver:<br/>driver_motors.yaml<br/>driver_ros.yaml<br/>driver_joint_bridge.yaml"]
    BY --> K["kstone:<br/>kstone.yaml"]
    BY --> T["teleop:<br/>xr_media.yaml<br/>common.yaml + teleop.yaml"]
    DR --> V{"Check against URDF"}
    D --> V
    V -->|pass| RUN["Start nodes"]
    V -->|fail| ERR["Error and exit"]
```

## 3. Runtime data flow (ROS topics)

```mermaid
flowchart LR
    subgraph XRN["/xr_media"]
        XT["*/transform<br/>TransformStamped"]
        XB["{left,right}_controller/buttons<br/>Joy"]
        XH["{left,right}_controller/haptic<br/>Float32MultiArray"]
    end

    TEL["lookie_teleop<br/>calibration / mapping / safety / catch-up"]

    subgraph KS["/trio/kstone"]
        KD["discover · state<br/>(latched JSON)"]
        KP["&lt;M&gt;/control/pose"]
        KJ["control/joint"]
        KV["chassis/control/cmd_vel"]
        KC["chassis/control/pose<br/>(omni base only, Trio)"]
        KH["&lt;M&gt;/home (srv)"]
        KF["&lt;M&gt;/feedback/pose"]
        ENG(("kstone engine<br/>whole-body IK + collision/CoM/null space"))
    end

    subgraph DV["/trio/driver"]
        FB["feedback/joint_states"]
        CT["control/joint_states"]
        JB["joint_bridge"]
        MODS["head / torso /<br/>left_arm / right_arm"]
        CV["chassis/cmd_vel"]
        OD["chassis/odom<br/>TF odom→base_footprint"]
        SC["chassis/scout"]
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
    FB -->|joint feedback| ENG
    OD -->|odometry| ENG
    ENG -->|joint commands| CT
    ENG -->|velocity command| CV
    CT --> JB --> MODS
    MODS --> JB --> FB
    CV --> SC --> OD
```

## 4. Control loop

```mermaid
sequenceDiagram
    participant H as XR headset/controllers
    participant X as xr_media_ros
    participant T as lookie_teleop
    participant K as kstone_ros
    participant B as joint_bridge
    participant D as Module drivers/chassis
    participant HW as Hardware (CAN)

    K-->>T: discover / state (latched)
    loop 50 Hz feedback
        HW->>D: Motor state
        D->>B: Per-module joint_states
        B->>K: driver/feedback/joint_states
        D->>K: driver/chassis/odom
    end
    H->>X: Headset/controller poses and buttons
    X->>T: */transform, */buttons
    T->>T: Calibration, frame mapping, safety checks
    T->>K: Pose/joint/velocity targets, home service
    K->>K: Whole-body IK solve
    K->>B: driver/control/joint_states
    B->>D: Joint commands split per module
    K->>D: driver/chassis/cmd_vel
    D->>HW: Motor/chassis commands
    T-->>X: Haptic feedback
```

## 5. Packages and dependency repositories

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
    DRV --> SR["scout_ros2"] --> UG["ugv_sdk"]
```

## 6. Feedback freshness and timeout chain

kstone sends commands to the drivers only while their feedback is fresh. Each layer judges freshness on its own; a timeout disables only the affected module or part, and the rest keeps running.

```mermaid
flowchart LR
    HW["Hardware (CAN)"] -->|"0.5 s"| D["Drivers"]
    D -->|"joints 0.15 s"| B["joint_bridge"]
    B -->|"0.25 s"| K["kstone_ros"]
    D -->|"odom 0.25 s"| K
    K -->|"~/state 0.3 s"| T["lookie_teleop"]
```

| Layer | Basis | On timeout |
|---|---|---|
| Drivers | Whether hardware data is still updating | Stop publishing that module's feedback |
| joint_bridge | Receipt time of each module's feedback | Leave the module out of the merged feedback |
| kstone | Stamp of each feedback group | Detach the group; re-attach automatically after it stays fresh for a while |
| lookie_teleop | Receipt time of `~/state` and the group states | Hold only the affected part; a `~/state` timeout disables teleop (see [teleop design §12](../lookie_teleop/docs/vr_teleop_dev_design.md)) |

- **Latency:** until a dropout is detected, the system runs on the last received data, for up to about 1 s.
- **Constraints:** the joint bridge timeout must not exceed kstone's (checked at startup, see [Regression checks](regression.md)); every timeout should be well above the matching publish period.
- **Observe:** `groups` and `fault` in `/trio/kstone/state`, `/trio/lookie_teleop/status`, and `feedback lost / recovered` in the driver logs.
- Simulation has no chassis odometry, so the `base` group is never fresh; this is expected.
