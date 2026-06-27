NightKite firmware bundle for NightKite Link SD card

Files:
- NK20_P2040.uf2   = firmware 2.0, Pico LiPo / RP2040
- NK30_P2040.uf2   = firmware 3.0, Pico LiPo / RP2040
- NK30_P2350.uf2   = firmware 3.0, Pico LiPo 2 / RP2350
- NK40D_P2040.uf2  = firmware 4.0-dev, Pico LiPo / RP2040
                      PlatformIO environment: pico2040
- NK40D_P2350.uf2  = firmware 4.0-dev, Pico LiPo 2 / RP2350, no RM2 radio
                      PlatformIO environment: pico2350
- NK40D_RM2BLE.uf2 = firmware 4.0-dev, Pico LiPo 2 / RP2350 with the wired
                      RM2 BLE breakout on GP17-GP20
                      PlatformIO environment: pico2350_rm2_ble

Sources:
- 2.0: git tag v2.0
- 3.0: git tag v3.0
- 4.0-dev: dev branch commit 6cc3312 (Stabilize audio sync pattern handling)
           Runtime version: 4.0.0-alpha.1

The current 4.0-dev builds include 27 patterns, Sync Beacon V1 compatibility,
V2 Audio Sync beacon receive support, AudioSyncState diagnostics and timeout,
the audio-reactive patterns 23-27, and stabilized pattern/sync loss handling.

NightKite Link lists every .uf2 file from /firmware and does not require these
exact filenames. In Firmware Update, select target RP2040 for NK20/NK30/NK40D
P2040 files. Select target RP2350 for NK30/NK40D P2350 and NK40D_RM2BLE files.
Use NK40D_RM2BLE only on the RP2350 controller with the wired RM2 breakout.
