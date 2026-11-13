# CAN 配置

[English](can.md)

`can_bind.sh` 给每个 CAN 口固定名称，并按对应波特率启动，重启或重新插拔后驱动仍能找到同一个设备。

## 接口

Lookie 驱动使用以下名称，Trio 和 Neo 相同。

| 接口 | 设备 | 波特率 |
| --- | --- | --- |
| `can_head` | 头部 | 1000000 |
| `can_torso` | 躯干（升降或折叠） | 1000000 |
| `can_left_arm` | 左臂 | 1000000 |
| `can_right_arm` | 右臂 | 1000000 |
| `can_chassis` | 底盘 | 500000 |
| `can_chassis_imu` | 底盘 IMU，Lookie 驱动不使用 | 500000 |

## 快速开始

Lookie 工控机的端口映射已写在脚本里：

```bash
ros2 run lookie_bringup can_bind.sh
ip -br link show type can
```

上表中的接口都应显示为 `UP`。脚本只在改动接口时才请求 `sudo`。名称和波特率在重启或重新插拔后会失效，需要重新运行。

## 在其他电脑上使用

脚本里的映射对应 Lookie 工控机。在你自己的电脑上，先复制脚本，再修改其中的 `MAPPING`：

```bash
cp "$(ros2 pkg prefix lookie_bringup)/lib/lookie_bringup/can_bind.sh" ~/can_bind.sh
```

1. 列出端口。分不清哪个口对应哪个设备时，一次只插一个适配器。

   ```bash
   ~/can_bind.sh --scan
   ```

   ```text
   IFACE        bus_addr       dev_id driver       Vendor   Device   UP
   --------------------------------------------------------------
   can0         0000:30:00.0   0x0    peak_pci     0x001c   0x0003   no
   can1         0000:30:00.0   0x1    peak_pci     0x001c   0x0003   no
   can2         1-2.3:1.0      0x0    gs_usb       0x1d50   0x606f   no
   ```

   扫描结果末尾会给出可直接粘贴的 `MAPPING` 键。

2. 按[接口](#接口)中的名称和波特率填写 `MAPPING`。同一地址下有多个口时保留 `:dev_id`。

   ```bash
   declare -a MAPPING=(
       "0000:30:00.0:0x0=can_left_arm:1000000"
       "0000:30:00.0:0x1=can_right_arm:1000000"
       "1-2.3:1.0=can_chassis:500000"
   )
   ```

3. 先预览计划，再执行。

   ```bash
   ~/can_bind.sh --list
   ~/can_bind.sh
   ```

PCIe 键对应板卡插槽，USB 键对应物理 USB 口，所以每个适配器要插回原来的 USB 口。

## 选项

| 选项 | 说明 |
| --- | --- |
| 无参数或 `--up [names]` | 重命名、设置波特率并启动全部或指定接口 |
| `--bitrate N` | 启动的接口统一使用波特率 `N` |
| `--down [names]` | 重命名后关闭全部或指定接口 |
| `--rename` | 只重命名 |
| `--scan` | 列出 CAN 口及其键，只读 |
| `--list` | 显示状态和重命名计划，只读 |

`names` 用逗号分隔，例如 `can_head,can_torso`。不传 `--bitrate` 时，先用 `MAPPING` 里的波特率，再用接口上已设置的波特率。

## 常见问题

- **`not in MAPPING`**：`--scan` 打印的键不在 `MAPPING` 里，或者适配器换了插槽或 USB 口。
- **`shorthand key is ambiguous`**：该地址下有多个口，改用完整的 `bus_addr:dev_id` 键。
- **`could not be brought up directly`**：没有可用波特率，给条目加上 `:bitrate` 或传 `--bitrate`。
- **驱动报找不到 CAN 设备**：用 `ip -br link show type can` 检查，名称必须与[接口](#接口)完全一致。
