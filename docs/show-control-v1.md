# NightKite Multi: Show Control V1

Firmware-side contract for a subsequent NightKite Link / PC-gateway sender. No
sender, MIDI/OSC, Wi-Fi, extended advertisements or pixel streaming is implemented
here. Patterns remain IDs **1–27**. Show Control is a runtime output layer.

## Enable and radio ownership

Before flight, on the BLE/RM2 firmware:

```text
NK4 seq=1 cmd=set wireless_enabled=1
NK4 seq=2 cmd=show_control enabled=1
NK4 seq=3 cmd=save
NK4 seq=4 cmd=show_status
```

`show_control enabled=0|1` changes the receive preference immediately; the existing
`cmd=save` persists it together with `wireless_enabled`. Save before flight and
check `saved=1`. Without save, changes follow the ordinary unsaved-config semantics.
Repeating `enabled=1` preserves runtime state. `enabled=0` cancels the queue,
clock/dedupe history and output override. `show_status` works on every build;
enabling on non-BLE builds returns `unsupported / show_radio_unavailable`.
`get section=wireless` appends `show_control_enabled`. Standard `status` adds
`show_active`, `show_queue`; `caps` adds `show_control`. Existing fields and
`get section=show` (configuration) retain their meanings.

Config version **402** appends a 32-bit integer receive flag at EEPROM bytes
**188–191**, 0 disabled / 1 enabled. EEPROM size 224, magic, all earlier addresses
and meanings remain unchanged. Versions 400/401 migrate with show reception off,
ignoring formerly unused bytes; their wireless, identity and sync settings survive.
Only v400 receives the existing audio-pattern-mask migration. Invalid receive flags
normalize to off. Recovery uses the existing one-time config save path. Downgrading
to firmware that only understands v401 can reset extended settings through that
older firmware's existing unknown-version recovery; export settings before downgrade.

A reboot/brownout restores the **saved receive preference**, but never resumes a
show: overrides, clocks, queues, IDs, pending/front buffers and diagnostics start
empty. Config load/reset also clears execution state. No show event writes flash.
The sender must rebuild its pending image and re-establish show time after a
controller reboot; initial output follows normal saved local/sync configuration.

- Active autonomous master (`play_mode=sync`, `sync_enabled=1`, `sync_role=master`)
  owns V1 beacon TX. It does not receive Show Control, even when enabled.
- Active follower scans for the existing V1/V2 beacons. With show enabled it also
  processes Show V1 in that same scan, with no sender/master address binding.
- Otherwise show enabled + wireless enabled selects `radio_mode=show_receiver`:
  scan for Show V1 and group-matching Audio V2; keep local manual/autoplay runtime
  underneath. Plain V1/V2 pattern/brightness fields do not take control of a local
  controller. Use a follower when those fields should control the underlying state.
- Connected GATT retains priority. Scanning stops until disconnect. Already queued
  events can still execute; audio expires normally. Scanner modes suppress GATT
  advertising as the existing follower does. USB remains available for recovery.
- Disabling wireless, changing group, entering autonomous master operation or
  battery cutoff cancels show events and releases the override. Reboot clears the
  complete show session and loads the existing saved configuration.

This uses the existing passive scan parameters (60 ms interval / 30 ms window).
No simultaneous autonomous master-TX/show-RX time slicing is attempted. Preflight
configuration can use USB or GATT; neither is required during flight.

## Complete advertisement (31 bytes)

Legacy non-connectable advertising, company ID `0xFFFF` (existing experimental
NightKite ID). Offsets below are zero-based. All multibyte integers are **little
endian**, including the 24-bit target, event ID and CRC.

| Advertisement bytes | Meaning |
| --- | --- |
| 0–2 | `02 01 06`: Flags AD structure |
| 3–4 | `1B FF`: Manufacturer AD length 27 and type 0xFF |
| 5–6 | `FF FF`: company ID |
| 7–30 | 24-byte Show Control packet below |

| Packet offset | Size | Field |
| --- | --- | --- |
| 0–1 | 2 | Magic ASCII `NS` (`4E 53`), distinct from sync `NK` |
| 2 | 1 | Version `01` = SHOW_CONTROL_V1 |
| 3 | 1 | Bits 7–6 target kind; bits 5–0 command ID |
| 4–6 | 3 | Target value |
| 7–8 | 2 | Event ID, modulo 65536 |
| 9–12 | 4 | Sender monotonic show time in milliseconds, modulo 2^32 |
| 13–14 | 2 | Low 16 bits of absolute execution time on that same clock |
| 15–21 | 7 | Command parameters; unused bytes MUST be zero |
| 22–23 | 2 | CRC16-CCITT |

CRC: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.
Compute over **all 24 packet bytes with bytes 22–23 zeroed**; store result LE.
The AD framing/company ID are not covered. The payload length must equal 24;
unknown version, target kind, command, out-of-range parameters or nonzero reserved
parameters are rejected. Sync V1 (17 bytes) and V2 (22 bytes), their fields and
CRC-zeroing rules are unchanged. Their original codec is now isolated in
`src/protocol/SyncBeaconCodec.*` for host testing.

Independent example: SINGLE `ABC123`, event `0x1234`, sender time `0x12345678`,
execute at `0x12345A00`, segment image 42 / index 1 / start 15 / count 10 / blue:

```text
02 01 06 1B FF FF FF 4E 53 01 87 23 C1 AB 34 12 78 56 34 12 00 5A 2A 01 0F 0A 00 00 FF 73 85
```

## Targets

| Kind (bits 7–6) | Value | Match |
| --- | --- | --- |
| 0 ALL | Must be 0 | Every show-enabled receiver, independent of sync group |
| 1 GROUP | 1–255, upper two bytes zero | Existing configured `sync_group` |
| 2 SINGLE | 0–0xFFFFFF | Numeric hex value of existing six-character `short_id` |
| 3 | Reserved | Reject |

The short ID is the final six hex characters of the existing persistent device
UID (also advertised as `NK-xxxxxx`). It is a target, never a master identity.
Short-ID collisions are possible; check the fleet before using SINGLE targeting.
For A and C, send two SINGLE events with separate IDs and the **same absolute
execution time**. Use GROUP/ALL when applicable. Audio reception always uses the
configured sync group, even if the show command was targeted ALL or SINGLE.

## Clock, scheduling, retransmission

Reconstruct the absolute sender deadline as:

```text
delta = signed16(execute_low16 - low16(sender_ms))
execute32 = sender_ms + delta                  # modulo 2^32
```

Only `-250 <= delta <= 30000` ms is legal. Deadlines more than 30 s away must be
sent later. Do not encode a different 65536-ms epoch as if it were this one.
All time and event comparisons use wrap-aware differences, with intervals below
the corresponding signed half range.

Send fresh CLOCK packets before the first event (e.g. 10 Hz for one second) and
throughout a show (e.g. every 100–200 ms). CLOCK is the same compact format, command
9, ALL, event 0, execution low16 equal to sender low16, zero parameters. It updates
time only, uses no event slot and never changes output. Other valid targeted show
packets also provide clock samples. Audio phase is deliberately not the show clock:
it may reset at a pattern or beat change.

For a new forward-moving sender timestamp, the receiver measures
`offset = local_receive_ms - sender_ms - NK_SHOW_RX_COMPENSATION_MS`. It keeps the
smallest offset in each 2 s sampling window to reduce receive-delay jitter. A new
window tracks oscillator drift. The compile-time compensation defaults to 0 ms;
only tune it against measured gateway/RM2 latency. A controller can bootstrap from
an event alone, but fresh CLOCK warmup is required for useful multi-controller
alignment when an advertising controller has repeated an old payload.

**The sender must timestamp close to handing a new payload to its advertising
controller, and refresh timestamps regularly.** Legacy controller-generated
repeats of an unchanged payload are allowed: identical sender timestamps never
update the clock, and older event timestamps do not update it. A host/gateway stalled with an old payload cannot provide
fresh time. Air/driver latency is not knowable from the payload alone.

On sender replacement/reboot, send only freshly timestamped CLOCK packets for
**at least 3 seconds** before new events. After 2 seconds without a forward clock
sample, a different CLOCK timestamp can rebase the offset even if its value is
lower. Natural 32-bit wrap remains a forward sample. No session ID or controller
reset is required. Queued events retain their already frozen deadlines across a
rebase. Stop the previous advertiser completely: with no sender/session identity,
reordered traffic from two clock epochs cannot be distinguished reliably. Identical
stalled CLOCK advertisements never re-arm the clock. Warmup recovers the clock;
it does not cancel previously accepted commands (explicit show disable does).


On first acceptance, freeze the local deadline as `execute32 + estimated_offset`.
Later CLOCK samples and event retransmissions do not move an already queued event.
A refreshed retransmission keeps event ID, target, command, parameters and absolute
execution time; only sender time and CRC change. Stop retransmitting once the
execution time is more than 250 ms old. An unchanged advertisement repeated many
times still executes at most once.

There are **8 event slots** with no dynamic allocation or eviction. Events execute
in deadline order, equal deadlines in event-ID order. A full queue rejects new
events without marking them received, allowing retries after space is available.
The Link/PC owns the long timeline; these eight slots are only short lookahead.
Keep preparation events near their actual execution time instead of filling the
queue with an entire show. `show_full` counts capacity rejects; `show_stale` counts
out-of-horizon clock-mapping rejects. No acknowledgments are sent.

Dedupe is an equality cache of the **last 64 accepted/terminally late event IDs**,
plus all IDs still queued. There is no numeric-age rule, highest-ID lock, master ID
or session ID. Smaller unused IDs from a restarted/replacement sender are accepted;
65535→0 works normally. Repeats consume no slot and do not refresh cache position.
The same ID may be used again after 64 other remembered events and after its queued
event has left. Reusing an ID still in either set is a duplicate, even with changed
parameters; retransmission cannot modify an accepted event. This bounded cache is
not permanent replay protection: never retransmit an event after 64 subsequent IDs
have been accepted, and stop all retries 250 ms after its execution time.

**Sender startup: choose a random 16-bit initial event ID**, then increment modulo
65536 for each new event (including preparation). This minimizes collisions with
a receiver's recent cache after sender reboot; a collision is discarded until
its cached ID leaves the window. CLOCK event 0 is exempt from dedupe. IDs belong to
one coordinated show producer; none of this ordering/cache logic applies to
controller-sync sequences. Warm the restarted clock as described above.

An event at most **250 ms late** executes on the next loop pass; later arrivals or
events delayed longer in the queue are dropped and remembered. No catch-up replay.
PatternClock-based show patterns start with phase `now - original_local_deadline`,
so allowed late execution preserves intended phase. Events are consumed before
rendering the next frame. Normal frame budget is 120 FPS (~8.33 ms), plus loop,
radio and LED-transfer jitter; this is not a sub-millisecond timing guarantee.

## Commands and output authority

`p0..p6` denote packet bytes 15..21. Unlisted parameters must be zero.
Every command except CLOCK is scheduled and deduplicated, including preparation.

| ID | Command | Parameters | Effect at deadline |
| --- | --- | --- | --- |
| 1 | SET_PATTERN | p0 = 1..27 | Acquire pattern authority, run entry even for same ID; show phase begins at deadline |
| 2 | SET_BRIGHTNESS | p0 = 1..255 | Runtime brightness override; leaves selected output mode intact |
| 3 | SET_SOLID | p0 R, p1 G, p2 B, p3 brightness 0..255 | Static RGB across both logical strips; 0 retains/inherits runtime brightness |
| 4 | BLACKOUT | None | Explicit all-black output, independent of configured brightness |
| 5 | RELEASE | None | Return to current underlying output/brightness and invalidate pending image |
| 6 | CLEAR_PENDING | p0 image ID 0..255, p1 expected segment count 0..32 | Start black backbuffer of current logical length; clear received-index mask |
| 7 | SET_SEGMENT | p0 image ID, p1 index 0..31, p2 start, p3 count, p4 R, p5 G, p6 B | Change pending image only; nonzero count, within current logical LED range |
| 8 | APPLY_PENDING | p0 image ID | Copy a complete matching pending image to the visible static buffer atomically |
| 9 | CLOCK | None; special header rules above | Clock sample only, never an output event |

SET_PATTERN, SET_SOLID, BLACKOUT and APPLY_PENDING acquire output authority until
another output command or RELEASE. At acquisition they retain any show brightness,
or snapshot current underlying brightness. SET_BRIGHTNESS alone freezes only
brightness; underlying pattern selection can still follow sync. Local autoplay is
paused while any show override is active and restarts its interval at release.
Preparation alone does not acquire output authority or pause autoplay.

Underlying `currentPattern`, configured brightness, play mode and local clock are
not overwritten by show events. Existing pattern entry callbacks now initialize
render state without writing configuration. Controller-sync beacons continue to
update the underlying pattern, brightness and phase while show owns output; audio
V2 is still decoded. RELEASE therefore displays the current master state directly.
If a follower loses sync during override, its configured fallback-autoplay action
is deferred until release, allowing continued sync/audio reception in the meantime.
If the master returns before release, the pending fallback is cleared. Continue-local
and warning-only retain their existing behavior.

On a local controller, RELEASE resumes the underlying local selection/clock,
with clean pattern re-entry rather than restoring old LED pixels. Local button or
configuration edits during override can update that underlying selection. Existing
sensor/random/frame-driven patterns keep their behavior; a synchronized start is
not a promise that local motion or random effects produce identical pixels.

The final brightness limiter runs after show output and any pattern-specific
brightness effects. Critical battery caps remain at 95; lower runtime values remain
lower. Soft/emergency cutoff remains authoritative, blacks both physical outputs,
stops the radio and discards the show. USB recovery retains its existing path.
Battery/charging views retain display priority while active. Segment and solid RGB
bytes are unscaled 0..255 colors; final brightness scales
them once, equally on both strips. SET_SOLID p3=0 retains an existing show brightness
or snapshots `currentBrightness` on first output acquisition (the underlying
local/master setting, before pattern modulation or battery limiting). Later
sync brightness changes update the underlying value but not that snapshot. RELEASE
drops every show brightness override: followers use the latest received master
brightness, local controllers their current local brightness (including any local
edits during the show). Pattern-specific brightness behavior resumes normally.
Runtime brightness is never written into saved brightness; BLACKOUT does not change accepted config
brightness levels. Show reception suppresses periodic saves; explicit config save
and the existing safety cutoff save still save only ordinary configuration.

## Pending image semantics and strip addressing

One pending image and one visible static image, each at most 70 RGB entries. Logical
addresses are the existing contiguous firmware buffer, **not** fixed pin-buffer
offsets or a perimeter/ring mapping:

- `0 .. L-1`: strip 1, index 0 through L-1.
- `L .. 2L-1`: strip 2, index 0 through L-1.
- L is configured `strip_length` (10..35). Physical strip 2 still starts at offset
  35 in the physical buffer; unused physical pixels stay black.

To mirror a segment on both strips, send two segments at `start` and `L+start`.
No additional inversion is applied to static images. Segments may cross the
logical strip boundary. To make LEDs 0–7 red, 8–14 black and 15–24 blue, clear an
image expecting three segments, then set `(0,8,255,0,0)`, `(8,7,0,0,0)` and
`(15,10,0,0,255)` at segment indices 0,1,2. Send three more segments if the same
image should be mirrored on strip 2, and declare six expected segments instead.

Schedule CLEAR before all segments; schedule APPLY after all preparation. Each
segment index is accepted once for the current image. Overlaps use execution
order; prefer disjoint segments. Retransmissions of each preparation event must
reuse its original event ID. Missing CLEAR, wrong image ID, repeated segment index,
index >= declared count, out-of-range segments or changed strip length are rejected.
APPLY requires all indices `0..count-1`; otherwise visible output remains unchanged
and that Apply event is consumed. Repair with a new image transaction and new
Apply event. Zero segments produces a complete black image.

CLEAR and SET_SEGMENT never change the visible buffer. APPLY copies the whole
image before rendering; it leaves the pending image available until the next
CLEAR/RELEASE. RELEASE invalidates pending image but **does not cancel later queued
events**; this permits a sequence of show and autonomous sections. Explicit show
disable cancels the entire queue. Image IDs distinguish builds, not dedupe sessions;
do not recycle a build ID while its old commands are still in flight.

## Strict audio and autonomous sync contracts

Patterns 23–27 render only when V2 audio is valid and age <=500 ms. The central
`NK_AUDIO_FRESHNESS_TIMEOUT_MS` in `AudioPatternMath.h` is independent of the
1500 ms controller-sync loss timeout. At the slowest existing 5 Hz profile, 500 ms
allows two missing 200 ms frames plus 100 ms receive jitter margin; 10/20 Hz also
fit. Above 500 ms, the next render frame is black and filters reset. Identical
V2 payload repeats do not renew audio or revive expired data; a changed V2 payload
(e.g. incremented sequence/phase) is a new frame, with no sequence-order restriction.
V1 packets do not create or refresh an audio source; interleaved V1 does not discard a still-fresh
V2 source. Invalid/expired audio clears the logical LEDs and all smoothed audio
values. No synthetic spectrum, local beat, local PatternClock audio substitute or
old pixel animation remains. Reacquisition/entry initializes directly from current
received values and replaces pixel history on the first valid frame. Existing
valid-audio response, spatial design and local orientation colors are preserved.
Autoplay still selects 23–27 normally; without audio they remain black.

A follower accepts every valid pattern 1–27 from its sync group, even when locally
disabled. Mask edits do not evict a currently locked master's pattern. The local
mask still controls button cycling/autoplay. A sync master may autoplay; a locked
follower does not autoplay independently. Master sequence values may decrease or
restart, and no master UID/address pinning is performed. The existing persisted
`master_uid` compatibility field remains untouched and unenforced.

## Diagnostics

`show_status` is one ordinary NK4 response with the request's sequence. No unsolicited
show event logging occurs in timing/radio paths. All counters are runtime-only;
explicit session reset retains counters, reboot zeros them.

- Enable/authority: `show_supported`, `show_enabled`, `show_rx`, `show_active`,
  `show_output` (0 underlying, 1 pattern, 2 solid, 3 blackout, 4 buffer),
  `show_pattern` (selected pattern, or underlying in static mode), `show_brightness`.
- Events: `show_queue`, `show_last_event`, `show_last_cmd`, `show_executed`,
  `show_rejected`, `show_received`, `show_accepted`, `show_duplicates`, `show_stale`,
  `show_full`. Executed includes successful preparation; rejected includes
  incomplete/wrong-image commands. `show_stale` is a timing-horizon rejection,
  never numeric event-ID age. Received includes targeted CLOCK/retransmissions.
- Reception: `show_invalid`, `show_crc` (subset of invalid), `show_target_miss`.
  Malformed foreign/company traffic retains existing radio diagnostic handling.
- Timing: `show_late`, `show_dropped_late`, `show_clock_valid` (sample acquired,
  consult age for freshness), `show_clock_offset_ms`, `show_clock_age_ms`,
  `show_clock_samples`, `show_last_due_ms`, `show_lateness_ms`.
- Buffer: `show_pending`, `show_image`, `show_segments` (expected count),
  `show_segment_mask` (received indices, decimal uint32).

## Validation and remaining hardware checks

Run `python3 test/run_host_tests.py` for all assert checks (C++11, warnings as errors,
ASan/UBSan), then `pio run` for all three environments and `git diff --check`.
The integration harnesses compile selected actual `main.cpp` orchestration and
radio GAP dispatch functions against hardware doubles; they do not emulate the
MCU, BTstack or real LEDs. Radio state/advertising ownership and actual EEPROM
field I/O/migration are also exercised with hardware doubles. Audio checks include
loss/reacquisition between frames and stalled repeated advertisements.

Hardware checklist (not established by builds/host tests):

1. Legacy USB CLI and USB NK4: query/set/save/load, unchanged ranges and echoed seq;
   BLE GATT: notifications/newline framing, show enable then disconnect, USB recovery.
2. Two battery-powered controllers, no GATT: master autoplay through 1–27, follower
   mask only pattern 1 enabled. Check pattern/brightness/phase and no follower autoplay.
   Stop master, switch to a master with lower sequence; check takeover and all three
   configured sync-loss behaviors.
3. Send V2 plus interleaved Show packets: each of 23–27 renders, V1-only/startup and
   >500 ms audio loss black immediately; restart at very different audio levels.
   Test while show displays solid/blackout, then select an audio pattern and release.
4. CLOCK warmup, ALL/GROUP/SINGLE at one absolute deadline; measure both strips and
   multiple devices with a logic analyzer/camera. Repeat identical advertisements,
   refresh timestamps, reorder/drop packets, fill queue, test 250 ms late boundary.
5. Prepare disjoint segments across both strips. Check no intermediate display,
   incomplete-image rejection, matching image apply, strip lengths 10/25/35,
   release and explicit session reset. Reboot must not resume a show.
6. Measure clock drift/jitter over a full flight; tune receiver compensation only
   from measurements. Validate gateway timestamp refresh and legacy advertisement
   interval under interleaved audio/show traffic.
7. Controlled supply: low warning, critical cap under white/brightness 255,
   hysteresis, both cutoff paths and USB recovery. Check saved brightness unchanged.
8. Confirm master TX remains autonomous with show enable set, and GATT ownership
   is restored after leaving scanner mode. No Wi-Fi should start.

## Next task in nightkite-link

Implement the exact encoder/31-byte framing and CRC; fleet short-ID/group targeting;
a stable show clock with regularly refreshed CLOCK packets; event-ID allocation,
absolute deadlines, repetitions that preserve event identity, bounded sending rate
and deadline ordering; interleave unchanged Audio V2; build segment transactions
with image IDs/count/index and atomic APPLY; send RELEASE for autonomous sections;
provide preflight enable/save instructions, controller-reboot reconstruction and
diagnostics tooling. Randomize the initial event ID, increment modulo 65536, and
use a three-second fresh CLOCK warmup after sender restart. Keep fresh Audio V2
frames arriving well inside 500 ms; advance their sequence even for constant audio. Verify
fresh timestamp placement against the gateway BLE API and run the hardware checklist.
Do not reuse the controller-sync sequence counter as a master lock.
