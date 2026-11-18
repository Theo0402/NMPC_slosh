# Data dictionary, log schema version 1

Generated from `schema/fields.yaml` by `schema/gen_py.py`. Do not edit.

Every record is a tag (u16), a payload length (u32) and the payload, little-endian. A file starts with its magic, a u32 header length and the header as JSON. A truncated last record is the end of the trial.

## `event` (tag 1, `.bin`, since v1)

Operator markers, phase transitions, errors converted at the RT3 sites, and JSON blocks (the arm block read at start-up, a tool's summary).

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  | host monotonic time |
| `kind` | u16 | enum |  | 1 marker, 2 phase, 3 error, 4 info, 5 arm_block (JSON), 6 summary (JSON), 7 refusal |
| `text` | bytes | utf-8 |  |  |

## `wire_tx` (tag 2, `.bin`, since v1)

Every byte string sent to the arm, verbatim (LG4).

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  | host monotonic time just before the send |
| `port` | u16 | - |  | 10003 for commands |
| `data` | bytes | ascii |  |  |

## `wire_rx` (tag 3, `.bin`, since v1)

Every chunk received from the arm on the command port, verbatim, as the socket returned it (LG4). Fragmentation is visible.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  | host monotonic time just after the read |
| `port` | u16 | - |  |  |
| `data` | bytes | ascii |  |  |

## `state_raw` (tag 4, `.bin`, since v1)

Every Datasheet packet from port 10004 as bytes, header included (elfin_commands.md, PROTO p.185-188). The packet has no timestamp.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  | host monotonic arrival |
| `data` | bytes | - |  |  |

## `robot_state` (tag 5, `.bin`, since v1)

The parsed Datasheet, in SI. Groups the packet lacks have their valid bit cleared.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  | host monotonic arrival of the packet |
| `valid` | u16 | bits |  | bit0 q, bit1 qd, bit2 qdd, bit3 current, bit4 state fields |
| `q_rad` | f64[6] | rad | joint | Actual_Position J1-J6 |
| `qd_rad_s` | f64[6] | rad/s | joint |  |
| `qdd_rad_s2` | f64[6] | rad/s^2 | joint |  |
| `current_A` | f64[6] | A | joint |  |
| `robot_state` | i32 | enum |  | StateAndError.robotState (table [MISSING]) |
| `enabled` | u8 | bool |  |  |
| `paused` | u8 | bool |  |  |
| `moving` | u8 | bool |  |  |
| `blending_done` | u8 | bool |  |  |
| `in_pos` | u8 | bool |  |  |
| `error_axis` | i32 | - |  | Error_AxisID as reported (1-6 here, 0-5 in ReadRobotState [UNCLEAR]) |
| `error_code` | i32 | - |  |  |

## `arm_cmd` (tag 6, `.bin`, since v1)

One record per push (LG3). The degrees on the wire are in wire_tx.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `push_id` | u64 | - |  |  |
| `tick` | i64 | - |  | the tick (or trajectory sample) whose q_cmd this push carries |
| `q_cmd_rad` | f64[6] | rad | joint |  |
| `t_slot_ns` | i64 | ns |  | the push slot this push belongs to |
| `t_send_ns` | i64 | ns |  |  |

## `arm_ack` (tag 7, `.bin`, since v1)

One record per reply to a push.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `push_id` | u64 | - |  |  |
| `tick` | i64 | - |  |  |
| `status` | u16 | enum |  | 0 OK, 1 Fail, 2 unexpected reply, 3 no reply (timeout or closed) |
| `fail_code_valid` | u8 | bool |  |  |
| `fail_code` | i64 | - |  |  |
| `t_reply_ns` | i64 | ns |  |  |
| `reply_time_ns` | i64 | ns |  | t_reply_ns - t_send_ns |
| `late` | u8 | bool |  | the push was sent after its slot had passed |

## `frame` (tag 8, `.bin`, since v1)

One record per camera frame.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `frame_number` | u64 | - |  |  |
| `cam_timestamp_ticks` | u64 | camera ticks |  | raw; tick rate [MISSING] until measured |
| `trigger_counter` | u64 | - |  |  |
| `t_host_ns` | i64 | ns |  | host arrival |
| `exposure_s` | f64 | s |  |  |
| `gain_db` | f64 | dB |  |  |
| `dropped_count` | u32 | - |  | frames dropped since the previous record |
| `image_stored` | u8 | bool |  |  |
| `image_index` | u64 | - |  | index of the image record; meaningful only if image_stored |

## `sync` (tag 9, `.bin`, since v1)

LED commands timestamped at the tick, trigger pulses, electrode transitions.

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `t_host_ns` | i64 | ns |  |  |
| `kind` | u16 | enum |  | 1 led_on, 2 led_off, 3 trigger_pulse, 4 electrode |
| `value` | i32 | - |  |  |

## `image` (tag 1001, `.img`, since v1)

One stored image (codebase plan §5, image policy).

| Field | Type | Unit | Frame | Notes |
|---|---|---|---|---|
| `frame_number` | u64 | - |  |  |
| `roi_x_px` | u16 | px |  |  |
| `roi_y_px` | u16 | px |  |  |
| `roi_w_px` | u16 | px |  |  |
| `roi_h_px` | u16 | px |  |  |
| `full_frame` | u8 | bool |  |  |
| `compression` | u8 | enum |  | 0 none, 1 zstd |
| `pixels` | bytes | Mono8 |  |  |

