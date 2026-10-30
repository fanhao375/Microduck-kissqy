# Physical controller mapping, private capability v1

This page owns `padd`'s learned physical input mapping, persistence, calibration gate and local
voice state. The [console guide](../../console/README.zh-CN.md) owns operator instructions.
The `robotd` and existing `pad.input` APIs do not change.

## Sources and actual control

`padd` still chooses the controlling gamepad through the existing gilrs priority rules. A second,
nonblocking evdev reader follows that exact selected device node. The mapping reader is independent
of the optional read-only `/run/padd/pad.sock` subscriber stream, so a saved mapping continues to
drive when no monitor is connected. Default devices retain their gilrs action/axis path and
ordinary pad-loss sit behavior. Failure of the additional raw reader does not disable an otherwise
working default device when no saved profile or mapping gate exists.

For a saved profile, raw values produce the same normalized state returned by `mapped` and used
for actual movement, button edges, trigger/mouth input, modes and skills. Press/release events
inside one control tick retain both edges. A custom stick's calibrated deadzone is applied once.
Coordinates on the mapping wire are right/down positive, sticks `-1..1`, triggers `0..1`; Y is
converted once at the existing gilrs control boundary. Optional unbound controls stay inactive.

USB (`bus=3`) and Bluetooth (`bus=5`) hardware are eligible. The known
`Xbox 360 Controller (GameSir Nova 2 Lite)` bridge is also eligible, as its own explicit identity;
the grabbed `GameSir-Nova 2 Lite` node remains excluded by existing source selection. Other
sysfs virtual devices, UDP and `Microduck R17 Web Xbox` cannot acquire a mapping session.

Device IDs include bus/vendor/product and a stable 128-bit fingerprint of name, unique ID and
axis/button capabilities. Event-node numbers and current values do not affect the fingerprint.
USB/BT mode and changed capabilities therefore select distinct profiles when the kernel exposes
those differences. The GameSir bridge currently advertises a fixed USB Xbox identity and does
not expose its underlying physical transport: its profile cannot distinguish two otherwise
identical bridge instances or infer whether their upstream link was USB or Bluetooth.

## Local socket and bindings

On Linux, `--mapping-socket` defaults to `/run/padd/mapping.sock`; it is owned by `padd`, group
`robot`, mode `0660`. This is a private capability, independently versioned at `1`. It accepts one
newline-terminated JSON object and returns one JSON line per connection. Four concurrent clients,
eight queued requests, a 32,768-byte request limit, an absolute 800 ms input deadline and a
1,500 ms acknowledgment wait bound stalled clients. Only one eligible `begin` may spend the
300 ms robot-state proof budget in one control tick; invalid/unavailable/busy begins are rejected
before that proof.

Requests and replies:

```json
{"op":"status"}
{"op":"begin","device_id":"evdev-v1-..."}
{"op":"renew","token":"..."}
{"op":"preview","token":"...","bindings":{}}
{"op":"save","token":"...","name":"My controller","bindings":{}}
{"op":"cancel","token":"..."}
{"op":"reset","token":"..."}
```

Successful replies contain `{ok:true,version:1,status:...}`; only `begin` also returns a 64-hex
token from 32 operating-system random bytes. Errors contain
`{ok:false,version:1,code:string,error:string}`. Unknown request/binding fields are rejected.
Codes include `invalid_request`, `invalid_bindings`, `no_device`, `device_changed`, `busy`,
`not_stopped`, `invalid_token`, `persistence` and `unavailable`.

Button roles are `south/east/west/north`, `lb/rb`, `select/start`, `du/dd/dl/dr`, `l3/r3/home`.
Axes are `lx/ly/rx/ry/lt/rt`. The standard positions are independent of the lettering printed
on the physical controller. Bindings have these exact shapes:

```json
{"kind":"key","code":304}
{"kind":"axis_button","code":16,"value":-1}
{"kind":"axis","code":0,"min":-32768,"max":32767,"center":0,"invert":false,"deadzone":0.1}
```

Buttons accept key or digital hat directions; LT/RT accept key or axis; four sticks require
axes. A hat source must advertise `min=-1,max=1`; only directions `-1` and `1` are valid.
Axis ranges must lie within the selected device's advertised capability, stick centers must
be strictly inside their range, and deadzones must be finite within `0..0.5`. Duplicate keys,
directions or analog axis sources are refused; opposite directions of one hat may be used.
Saving requires Select, Start and four stick axes. Preview accepts partial mappings.

The exact `status` members are:

```text
device: null | {id,name,node,unique?,bus,vendor,product,
               axes:[{code,name,min,max,flat,fuzz,value}],buttons:[{code,name,pressed}]}
raw: null | {seq,buttons:{decimal_code:bool},axes:{decimal_code:i32},
             events:[{seq,kind,code,value}]}
mapped: null | {buttons:{role:bool},axes:{lx,ly,rx,ry,lt,rt}}
preview: null | same shape as mapped
profile: null | {schema:1,name,bindings:{role:binding}}
session: null | {phase:"learning"|"waiting_neutral",remaining_ms}
output_blocked: bool
```

Raw events retain at most the latest 256 key/axis events with monotonic sequence numbers over
the daemon lifetime. A client captures the current sequence before learning and only accepts
later events; if its cursor falls out of history it must start that capture again. Device
disconnect/replacement and `SYN_DROPPED` clear history, invalidate capture and require a fresh
kernel snapshot. Old-device samples cannot be used while the new node is being opened.

## Safety and session lifetime

Before accepting `begin`, the actual main loop independently requests `robot.busStatus` and two
fresh `robot.state` ticks through a separate, read-only connection. It requires explicit dynamic
`policy_enabled=false`, held state, zero requested/applied velocity and either completed HOME or
limp state; active policy, skill/homing labels, missing facts, stalled state and a non-advancing
clock are rejected. The two state timestamps must advance within 100 ms, and the whole response
read has an absolute 300 ms deadline, including partial-byte responses.

The main control gate runs before every pad edge, hold, skill, mode, shutdown, IMU head and
continuous-output path. Entering it clears velocity, body pose (`active:false`), head and mouth
targets and held Wheee sound once, and clears local modes, timers and pending edges. It then
atomically publishes the blocked voice state before acknowledging the session. A publication
failure still clears outputs first and prevents success. While blocked, the loop does not
pretend that the pad was lost and does not enter automatic sit logic.

Learning leases last five seconds and only explicit `renew` extends them. Preview updates only
the draft. Save, cancel, reset, expiry, mapping-reader loss, changed device and dropped reports
enter `waiting_neutral`. All keys must release, hats must center and the actual normalized sticks
and triggers must remain neutral for 300 ms before output resumes. Even a press/release or stick
pulse entirely inside one tick restarts that interval. Resumption discards old edges and hold
times; it does not enable the policy or perform an action. The web emergency stop remains a
separate external action. Studio must separately own its page session and prevent other web
actions; this gate independently protects native pad output.

`mapping-state.json` lives beside `mapping.sock`, owner `padd`, group `robot`, mode `0640`:

```json
{"version":1,"pid":123,"output_blocked":true,"device":{"id":"evdev-v1-...","node":"/dev/input/event7"},"profile":null}
```

`profile` is null or the full saved `{schema,name,bindings}` object. This file contains no token,
draft or raw values. It changes only when device, saved profile or gate changes, so the separate
voice reader can use the saved R3/Select binding and baseline its own edges without per-event
disk writes. Consumers must verify the live process and fail closed on malformed state.

## Persistence and errors

`--mapping-state-dir` defaults to `/var/lib/padd`. Both shipped units declare
`StateDirectory=padd`, making that directory writable under `ProtectSystem=strict`.
The `mappings.json` schema is `{schema:1,profiles:{device_id:profile}}`; at most 32 devices,
21 bindings per device, 128-byte names and 262,144 file bytes are accepted. Saves use a random,
exclusive `0600` temporary file, flush the file, atomically rename it and flush the directory.
Validation and write failures retain the live profile and learning lease. Reset removes only
the selected device's profile.

A missing file is first use. A corrupt, oversized, unknown-schema or unreadable existing file
is logged and ignored for live mapping, and is protected against subsequent save/reset overwrite.
The original file must be repaired or backed up externally and `padd` restarted before writes
are accepted again. This prevents an unsuccessful read from silently deleting another device's
saved configuration. Raw metadata is limited to 64 axes, 256 buttons and 256-byte name/unique
strings; an unreadable source with a possibly applicable saved profile keeps output paused.

## Software verification and remaining hardware acceptance

Rust tests cover raw AB swaps, axis swap/inversion/center/deadzone, analog/digital triggers,
hats, actual control boundary and safe output parameters, leases/neutral/old edges, bounded raw
history/drop/replacement, save/reload/reset, identity isolation, corrupt file protection, actual
Linux permission denial and slow socket/robot replies. Unix-socket fixtures exercise the real
mapping server and robot proof. `padd/tests/process_smoke.py` runs the actual compiled daemon
against a temporary read-only robot fixture only when `/dev/input/event*` is absent; it checks
the new capability, unchanged raw tap and absence of movement/skill requests.

`mapping::native::tests::cross_language_fixture` is opt-in through
`PAD_MAPPING_FIXTURE_DIR`; it uses synthetic raw metadata/events and the production mapping
socket for the Studio Python contract probe. It does not open a hardware device or robot socket.
ARM64 compilation and QEMU CLI smoke validate the target executable, not hardware behavior.
Different physical controller models, actual USB/BT reconnects and the GameSir bridge still
require separate bench acceptance; no real robot was accessed or deployed for this change.
