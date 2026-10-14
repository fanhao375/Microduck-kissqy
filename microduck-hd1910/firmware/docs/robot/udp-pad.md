<!-- Added 2026-10 by fanhao375 (microduck-replica): UDP pad ([netpad]); the port of Pollen Robotics microduck 0.17.1 udp-pad (Apache-2.0); see IMU-MODULE.md at the repository root. -->
# Drive with a pad over UDP

A program on the same network sends the state of a gamepad — sticks, triggers, buttons — as UDP
datagrams, and the robot does with it exactly what it does with a Bluetooth pad: the same mapping,
the same modes, the same holds on Start and Select. It is for a pad that is not near the robot, or
not Bluetooth at all — a laptop with a pad in its USB port.

This is upstream's UDP pad (Pollen microduck 0.17, `docs/robot/udp-pad.md` there), ported onto this
0.15.1 / control.43 (V167) tree. The wire format is upstream's byte for byte, so the client is upstream's
own `duckctl udp-pad`. What differs is on the robot: upstream reads it in a second daemon,
`netpadd`; here it is a second **source** inside `padd`, read through the R17 mapping this `padd`
already has (`padd/src/udp.rs`). There is no `netpadd` unit to start.

**It is a quick, unauthenticated LAN tool.** Anyone on that network who can reach the port can
drive the robot, stand it up, make it go limp and power it off. Turn it on only on a network you
trust, and only while you use it.

## Turning it on

Off by default. On this fork, one command does it on the robot (hold or hang the duck: it relaxes
the robot first):

```bash
sudo bash /opt/robot/feetech-ft5-r5/service.sh netpad on              # port 4210, timeout 250 ms
sudo bash /opt/robot/feetech-ft5-r5/service.sh netpad on 4210 250     # or name them
sudo bash /opt/robot/feetech-ft5-r5/service.sh netpad off             # back to the Bluetooth pad
```

It checks the numbers against both `robotd.toml` and `robotd.saved.toml` (each against its own
`[safety] deadman_ms`), that this `padd` knows `[netpad]`, that `padd.service` allows `AF_INET` and,
for `on`, that robotd has finished its four startup steps (padd waits for them); writes `[netpad]`
into the files that change (a `<name>.netpad-previous` copy of each first); restarts `padd` alone and
reads its new process's journal for the line below. If `on` does not bring that line, the files this
run wrote are put back and `padd` restarted. `off` keeps the port and timeout already in the files
and is never rolled back: it is the way back to the Bluetooth pad.

By hand it is this section in `/etc/robot/feetech-ft5/robotd.toml` (and `robotd.saved.toml`):

```toml
[netpad]
enabled = true
port = 4210        # upstream's default, which duckctl assumes
timeout_ms = 250   # silence after which the client is gone; must be below [safety] deadman_ms
```

then `sudo systemctl restart padd`. The journal says which source it took:

```
netpad on — reading the pad from UDP (duckctl udp-pad <this robot's IP>); the Bluetooth and web pads are not read until [netpad] enabled = false
```

**While it is on, the Bluetooth pad and the console's web pad are not read at all** — `padd` reads
one source. `enabled = false` (or removing the section) and the same restart go back.

**Get the two numbers right.** With `enabled = true`, a `timeout_ms` that is 0 or not below
`[safety] deadman_ms`, or a `port` of 0, makes the whole file invalid — for `robotd` built from
this tree too, which then refuses to start and names the key in its journal, exactly as upstream
does. `padd` treats a file it cannot read as netpad off. A `robotd` from before this change ignores
the section with a warning. If `robotd` refuses to start over it, fix or remove `[netpad]` in both
files and run `sudo bash /opt/robot/feetech-ft5-r5/service.sh reconnect` (it restarts robotd, then
padd, and the GameSir bridge only where that is enabled).

`padd.service` must allow `AF_INET` (`RestrictAddressFamilies=… AF_INET`, in both
`padd/systemd/padd.service` and `packaging/padd.service`). A unit installed before this change
needs a drop-in, or the bind fails with `EAFNOSUPPORT` and `padd` restarts every five seconds.
The fork's installer (`deploy/imu-module-install.sh`) writes this one and removes it on rollback;
drop-ins add to the list, so `AF_INET` alone is enough:

```ini
# /etc/systemd/system/padd.service.d/zzzz-replica-netpad.conf
[Service]
RestrictAddressFamilies=AF_INET
```

## Driving from a laptop

With [uv](https://docs.astral.sh/uv/) installed, on Windows, macOS or Linux:

```bash
uvx duckctl@0.17.1 udp-pad 192.168.1.42
```

Pin the version: the datagram is version 1 and this build reads only that. Plug a pad into the
laptop first; with no pad, `duckctl` sends nothing and the robot times it out. If pushing the stick
forward walks the robot backwards, add `--invert-y`. Without an address `duckctl` looks for the
robot over Bluetooth, which needs `btd` on the robot — give the address.

On the laptop, a VPN or proxy in TUN mode (Clash and the like) can capture LAN traffic: if the
panel never shows the robot connected, exempt the robot's subnet or switch TUN off.

## What the robot does

- One client drives at a time. Another one sending meanwhile is told *busy* (at most once a second)
  and ignored until the first has been quiet `timeout_ms`.
- Every pad datagram is answered with a 24-byte status (anything that is not one gets no answer),
  and the client that drives gets the robot's health once a second — battery, temperatures, loop
  rate, posture, the last request the pad asked for and the robot refused — which `duckctl` draws.
- A client's first datagram presses nothing: buttons it already holds wait for their release, as
  a Bluetooth pad's do when it connects. The stick clicks are not on the wire, so the R17 L3
  roller test is Bluetooth-only.
- If `padd` itself is held up past `timeout_ms` — a slow answer from `robotd` — the datagrams that
  queued meanwhile keep the client connected, but their sticks read as centred until one is heard
  as it lands: they may be a client's last words from before it left. Until then the holds —
  Select's shutdown, X's resend, the D-pad mode holds — keep their time without firing, and in
  head or body-pose mode only the stop is sent, not a pose to centre.
- The client silent for `timeout_ms`: nothing more is sent, and `robotd`'s deadman stops any
  walking. Still gone a second later, the robot asks where the pad went (a questioning quack) and
  sits down — `sit_toggle`, D-pad down's default — unless it is sitting already or the policy is not
  driving. It greets the pad when it comes back. A drop-out shorter than that second is silent.
  This is the same for a Bluetooth pad that goes away (`padd/src/loss.rs`).
