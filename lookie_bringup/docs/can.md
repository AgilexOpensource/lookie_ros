# CAN setup

[中文](can.zh-CN.md)

`can_bind.sh` gives every CAN port a fixed name and brings it up at the right bitrate, so the drivers find the same device after every boot or replug.

## Interfaces

The Lookie drivers expect these names. Trio and Neo use the same set.

| Interface | Device | Bitrate |
| --- | --- | --- |
| `can_head` | Head | 1000000 |
| `can_torso` | Torso (lift or fold) | 1000000 |
| `can_left_arm` | Left arm | 1000000 |
| `can_right_arm` | Right arm | 1000000 |
| `can_chassis` | Chassis | 500000 |
| `can_chassis_imu` | Chassis IMU, not used by the Lookie drivers | 500000 |

## Quick start

On the Lookie computer the port mapping is already set in the script:

```bash
ros2 run lookie_bringup can_bind.sh
ip -br link show type can
```

All interfaces above should be listed as `UP`. The script asks for `sudo` only when it changes an interface. Names and bitrates do not survive a reboot or replug, so run it again after either.

## Use on another computer

The mapping inside the script matches the Lookie computer. On your own PC, copy the script and edit its `MAPPING`:

```bash
cp "$(ros2 pkg prefix lookie_bringup)/lib/lookie_bringup/can_bind.sh" ~/can_bind.sh
```

1. List the ports. Plug in one adapter at a time if you are unsure which is which.

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

   The scan also prints ready-to-paste `MAPPING` keys.

2. Put the keys into `MAPPING` with the names and bitrates from [Interfaces](#interfaces). Keep `:dev_id` when one address has several ports.

   ```bash
   declare -a MAPPING=(
       "0000:30:00.0:0x0=can_left_arm:1000000"
       "0000:30:00.0:0x1=can_right_arm:1000000"
       "1-2.3:1.0=can_chassis:500000"
   )
   ```

3. Preview the plan, then apply it.

   ```bash
   ~/can_bind.sh --list
   ~/can_bind.sh
   ```

A PCIe key follows the card slot. A USB key follows the physical USB port, so plug each adapter back into the same port.

## Options

| Option | Description |
| --- | --- |
| (none) or `--up [names]` | Rename, set bitrate and bring up all or the listed interfaces |
| `--bitrate N` | Use bitrate `N` for every interface brought up |
| `--down [names]` | Rename, then bring down all or the listed interfaces |
| `--rename` | Rename only |
| `--scan` | List CAN ports and their keys, read-only |
| `--list` | Show status and the rename plan, read-only |

`names` is a comma-separated list such as `can_head,can_torso`. Without `--bitrate`, the script uses the bitrate in `MAPPING`, then the bitrate already set on the interface.

## Troubleshooting

- **`not in MAPPING`**: the key printed by `--scan` is missing from `MAPPING`, or the adapter moved to another slot or USB port.
- **`shorthand key is ambiguous`**: the address has several ports; use the full `bus_addr:dev_id` key.
- **`could not be brought up directly`**: no bitrate is known; add `:bitrate` to the entry or pass `--bitrate`.
- **Driver reports a missing CAN device**: check `ip -br link show type can`; the name must match the interface table above exactly.
