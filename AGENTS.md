# NightKite Multi Repository Guidance

## Scope and sources of truth

- This repository owns NightKite Multi firmware only. Work inside it unless the user explicitly expands scope; never modify sibling `nightkite-link` or `nightkite-configurator` repositories.
- Do not commit or push unless explicitly requested. Preserve unrelated user changes and avoid broad formatting or generated-file churn.
- Treat `platformio.ini` as authoritative for supported boards, pins, feature flags, toolchain, and dependencies. Treat source as authoritative for current patterns, value ranges, protocol fields/UUIDs, beacon layouts, and persistent storage. Read those definitions before changing them; release binaries and prose documentation do not override them.
- Do not update bundled UF2 files, hardware assets, measurements, or release documentation unless the task requires it.

## Product and hardware invariants

- Preserve every hardware and feature variant defined in `platformio.ini`, including legacy and optional-feature builds.
- Board and RM2 pin assignments are physical wiring constraints, not freely interchangeable configuration. Change them only with explicit hardware direction.
- Preserve the two mirrored WS281x outputs, MPU6050 motion/orientation behavior, button control, manual/autoplay/sync modes, battery and USB behavior, calibration, diagnostics, and recoverable persistent configuration.
- Preserve every shipped pattern and its ID, name, enable/invert semantics, configuration ranges, and intended visual behavior unless the user requests a compatibility break. Motion-reactive inputs remain local even when animation timing is synchronized.
- Battery warning, brightness limiting, cutoff, hysteresis, USB-power recovery, and flash-wear protections are safety behavior. Do not weaken them or overwrite the saved brightness merely to apply a runtime limit.
- Do not add or enable Wi-Fi unless explicitly requested.

## Compatibility contracts

- The legacy USB CLI and USB NK4 machine protocol are public interfaces. Existing commands, arguments, accepted ranges, response prefixes, error codes, status fields, and destructive-action confirmations must remain backward compatible.
- NK4 responses must echo the request sequence and remain short, line-based, and machine-parseable. Machine mode must not emit prompts, banners, or unrelated debug text.
- BLE GATT, when compiled in, is an optional NK4 configuration/status/control transport. Preserve its UUIDs, newline framing, notification behavior, and USB fallback.
- Real-time controller sync uses compact binary BLE advertising beacons, separate from NK4/GATT. Controllers render locally; do not stream LED frames or make GATT, Wi-Fi, or an external controller mandatory at runtime.
- Preserve GATT/beacon advertising ownership and connection-priority behavior. A connected GATT client may pre-empt beacon operation; master and follower beacon modes must remain autonomous otherwise.
- Pattern IDs, beacon byte layouts/versioning/CRC rules, and accepted legacy beacon versions are wire contracts. Extend them compatibly; never silently reinterpret or renumber existing values.
- EEPROM addresses, magic/version values, masks, and stored meanings are storage contracts. Never reorder or reuse them. Any incompatible storage change needs explicit versioning, migration, validation, and a focused migration test.
- Preserve play-mode semantics: local autoplay runs only where intended; a sync master may autoplay and carry followers; a sync follower must not independently advance patterns.

## Implementation rules

- Prefer small, reviewable changes that preserve behavior. Trace all callers and fix shared root causes once; avoid speculative abstractions and large rewrites.
- Keep business logic transport-neutral. Reuse the response-writer/protocol helpers instead of adding direct `Serial.print()` calls outside transport or legacy-CLI code.
- Keep BLE/RM2 code optional behind existing build flags. Non-BLE builds must not acquire Bluetooth or Wi-Fi dependencies.
- Keep the main loop and radio paths non-blocking. Do not perform EEPROM/flash writes in timing-critical sync or render paths.
- Use `PatternClock` only where a shared time base is safe. Preserve local sensor/color inputs and the established pattern look; broad timing migrations require per-pattern classification and visual verification.
- Preserve useful diagnostics when changing boot, battery, protocol, radio, sync, persistence, or timing behavior.
- Do not upgrade platforms or dependencies unless explicitly requested or required by the task.

## Validation and handoff

- Run `git diff --check` for every change.
- For shared firmware, protocol, persistence, pattern, battery, or sync changes, build every environment defined in `platformio.ini`. For narrowly target-specific work, build the affected environment and relevant feature-disabled counterparts; BLE/RM2 work must include the BLE environment and non-BLE builds.
- Run the smallest relevant assert-based checks under `test/`; run all of them when shared helpers or contracts change. Add one focused regression check for new non-trivial logic.
- Hardware behavior cannot be proven by compilation. When affected and hardware is available, verify the legacy USB CLI and USB NK4; also verify BLE GATT for BLE work and master/follower lock, pattern/brightness following, master autoplay, and loss behavior for sync work. Test beacon sync without an active GATT connection.
- Never claim an interface or hardware path was tested when it was only built. Report unavailable hardware validation explicitly.
- Final handoff must list changed files, build/test/hardware results, and open TODOs. Do not commit or push unless requested.
