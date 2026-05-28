# NightKite Pattern/Brightness Sweep

Preparation timestamp: 2026-05-28_061232

## Firmware-Derived Test Matrix

Brightness levels: 95, 127, 159, 191, 223, 255

Patterns:
- 1 rainbow
- 2 full_color
- 3 motion_brightness
- 4 runner_fixed
- 5 runner_reactive
- 6 runner_dual
- 7 heartbeat
- 8 ping_pong
- 9 comet_swarm
- 10 breath_storm
- 11 jerk_wave
- 12 yaw_spinner
- 13 yaw_spinner_circle
- 14 runner_dual_inverted
- 15 palette_beat_motion
- 16 pacifica_kite
- 17 twinkle_motion
- 18 fire_jet
- 19 noise_ring
- 20 pride_yaw
- 21 confetti_jerk
- 22 center_ripple

## Device Discovery

NightKite USB serial: /dev/cu.usbmodem1101

NightKite identity:
- name: NK-FD6181
- uid: 576B81A319FD6181
- firmware: 4.0.0-alpha.1
- hardware: pico2350-rm2-ble
- BLE service UUID: 4e4b4000-6e69-6768-746b-000000000001
- BLE RX write UUID: 4e4b4000-6e69-6768-746b-000000000002
- BLE TX notify UUID: 4e4b4000-6e69-6768-746b-000000000003

PowerMeter USB serial: /dev/cu.usbserial-210

## Preparation Commands

NightKite USB NK4:
- NK4 seq=10 cmd=set play_mode=manual autoplay=0 wireless_enabled=1 sync_enabled=0 pattern=1 brightness=95
- NK4 seq=11 cmd=status
- NK4 seq=12 cmd=ble_status
- NK4 seq=13 cmd=get section=wireless

PowerMeter serial:
- ?
- i 100
- r
- h
- log erase
- log new nk_pattern_brightness_prep
- log status
- mark prep_ready

PowerMeter header:

```csv
runtime_ms,interval_ms,voltage_v,current_ma,signed_power_w,sensor_power_w,discharged_mah,charged_mah,net_mah,discharged_wh,charged_wh,net_wh
```

Preparation state:
- PowerMeter interval set to 100 ms.
- PowerMeter counters reset.
- PowerMeter marker support confirmed.
- PowerMeter offline logging active at /powerlog.csv.
- NightKite set to manual, autoplay off, wireless on, sync off, pattern 1, brightness 95.
- No NightKite save command was sent.

Note: while NightKite USB was connected, PowerMeter current was negative, consistent with charge/input direction. Battery-only positive current check is pending after manual restart.

## Measurement Result

Run started: 2026-05-28T04:17:28.352Z
Run ended: 2026-05-28T04:31:32.299Z
Elapsed seconds: 843.9
Segments measured: 132 of 132
Completed: true
Abort reason: none
Latest PowerMeter voltage: 4.0200 V
Latest PowerMeter current: 78.750 mA

Generated files:
- raw_powerlog.csv
- segments.csv
- summary_per_segment.csv
- summary_pivot.csv
- run_notes.md

Post-run PowerMeter cleanup:
- Sent `mark host_summary_done`.
- Sent `log stop sweep_done`.
- PowerMeter reported `LOG active=0 full=1 file=/powerlog.csv size_bytes=491205 free_bytes=491520`.
- The host-side `raw_powerlog.csv` is complete; the ESP offline flash log filled after the sweep and was stopped.
