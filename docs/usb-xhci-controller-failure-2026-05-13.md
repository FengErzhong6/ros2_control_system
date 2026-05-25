# USB xHCI Controller Failure Incident (2026-05-13)

## Summary

RealSense `initial_reset` firmware reset triggered an **xHCI USB controller-level crash**, causing systemic USB enumeration failure across multiple ports. Recovery requires reboot.

## Error Scale (14:09 - 14:26, 17 minutes)

| Metric | Count |
|--------|-------|
| `error -110` (ETIMEDOUT) | 62 |
| `unable to enumerate USB device` | 5 (ports 2-2, 2-4) |
| Affected ports | Bus 001: port 3, 4, 6, 11; Bus 002: port 2, 4 |

## Key kernel messages

```
usb 2-4: device descriptor read/8, error -110
usb usb2-port4: attempt power cycle
usb 2-4: new SuperSpeed USB device number 35
usb 2-4: device descriptor read/8, error -110  (x4)
usb usb2-port4: unable to enumerate USB device

usb 1-3: device descriptor read/64, error -110
usb 1-3: reset high-speed USB device number 3
usb 1-3: USB disconnect, device number 3
usb 1-3: new high-speed USB device number 36

hub 1-4:1.0: hub_ext_port_status failed (err = -110)
usb 1-4-port3: cannot disable (err = -110)
usb 1-6: clock source 5 is not valid, cannot use
usb 1-6: cannot get freq (v2/v3): err -110
usb 1-6: cannot set freq 48000 (v2/v3): err -110
usb 1-11: device descriptor read/64, error -110
```

## Failure Mechanism

```
RealSense firmware reset (14:09:47)
  └→ USB 3.2 device disconnect/reconnect
      └→ xHCI command ring fills with enumeration requests
          └→ Simultaneously handling active Orbbec isochronous streams
              └→ Commands time out (error -110)
                  └→ Controller retries → more timeouts → cascade
                      └→ Command ring blocked: new insertions can't be processed
                          └→ Device permanently invisible
```

## Root Cause

`error -110` = `ETIMEDOUT`: USB host controller sent a request, device didn't respond within timeout.

The xHCI controller's Command Ring is a hardware DMA queue. Once blocked:
1. Timed-out commands are not cleared from the ring
2. Ring pointer doesn't advance
3. New commands (including enumeration for new devices) can't be queued
4. `xhci_hcd` driver has no runtime mechanism to reset the command ring without dropping all connected devices

## Why RealSense became undetectable

USB 3.x enumeration requires:
1. Physical-layer link training (Rx/Tx equalization)
2. Read device descriptor (first 8 bytes)
3. Assign address
4. Read full descriptor, select configuration

Stuck at step 2 — controller can't even read 8 bytes from the device. Port power-cycle doesn't help because the controller command ring is still blocked.

## Why reboot is the only fix

- The command ring is a hardware structure, not resettable via `/sys/bus/usb`
- Unbinding `xhci_hcd` would drop ALL USB devices (including keyboard/mouse/network), effectively same as reboot
- Physical replug is ineffective: the controller never sees the new device

## Fix Applied

Added `initial_reset: false` to `cam_high` launch_arguments in:
`policy_deployment_statck/policy_deployment_bringup/config/recipes/default_policy_deployment.yaml`

This prevents `realsense2_camera_node` from triggering firmware reset at startup.

## Related Timeline

| Time | Event |
|------|-------|
| 14:03 | Startup test: all 3 hardware layers reached READY, warmup model loaded |
| 14:09:38 | Full startup: cam_high stuck "no image message" |
| 14:09:47 | RealSense found, "Resetting device..." triggered |
| 14:09:53 | After reset: "No RealSense devices were found!" |
| 14:10:02 | wait for device timeout → exit code 1 |
| 14:10:04 | Respawn attempt 2: Permission denied on /dev/video0-5 |
| 14:10:21 | Respawn attempt 3: same failure |
| 14:10:39 | First `unable to enumerate USB device` on port 2-2 |
| ~14:12 | One Orbbec camera disappears (port 2-2 device lost) |
| 14:20:21 | Standalone test: device /dev/video0 found but EBUSY |
| 14:20:21 | Retries cause device crash: "The device has been disconnected!" |
| 14:22:34 | Port 2-4 also fails enumeration |
| 14:23:16 | Port 2-4 `unable to enumerate USB device` |
| 14:25:39 | Bus 001 port power-cycle attempt, error spreads to Bluetooth dongle |
