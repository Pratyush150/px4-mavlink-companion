# MAVLink link troubleshooting: a field guide

Written for the situation where the vehicle is on the bench, the companion
computer is plugged in, and nothing works. Symptom first, then the causes in
the order they are actually worth checking, then the fix.

Two conventions used throughout:

- **PX4** parameters look like `MAV_0_CONFIG`, `SER_TEL1_BAUD`.
- **ArduPilot** parameters look like `SERIAL1_PROTOCOL`, `SR1_EXTRA1`.
  The `SERIALn` / `SRn` index follows the *serial port*, not the MAVLink
  channel you think you are on. USB is usually `SERIAL0`.

Start here:

```bash
python3 tools/mavdiag.py --port auto          # ranked diagnosis, human output
python3 tools/mavdiag.py --port auto --json   # same thing for CI or a log
```

To see what each fault looks like before you meet it in the field, run the
diagnostic against the built-in simulator (no hardware, no SITL):

```bash
python3 tools/mavdiag.py --sim px4 --fault frozen --duration 14
# faults: link-drop, stale-attitude, frozen, gps-loss, battery-sag, time-backwards
# --fault-at N injects it N seconds in, so you see a healthy link go bad
```

---

## 1. No heartbeat at all

Nothing arrives. `mavdiag` says "no HEARTBEAT". This is the most common
starting point and almost never a code problem.

| Cause | How to confirm | Fix |
|---|---|---|
| Wrong device | `ls -l /dev/serial/by-id/` shows what is actually attached | Use the `by-id` path, not `ttyACM0` (see section 7) |
| Wrong baud | Raw bytes arrive but never frame: `timeout 3 cat /dev/ttyUSB0 \| xxd \| head` shows garbage that never repeats a pattern | Probe: `MavLink(...).probe_baud()`, or try 57600, 921600, 115200 in that order |
| Port is not configured for MAVLink | Autopilot parameter check | **PX4**: `MAV_0_CONFIG` / `MAV_1_CONFIG` must select the port; `MAV_x_MODE = 2` (Onboard) for a companion. **ArduPilot**: `SERIALn_PROTOCOL = 2` (MAVLink2) and `SERIALn_BAUD` set to match |
| TX/RX swapped | Nothing at any baud, on any tool | FC TX -> companion RX, FC RX -> companion TX |
| No common ground | Intermittent garbage, works when you touch the frame | Run a ground wire between the boards; a USB shield is not a ground |
| Charge-only USB cable | `dmesg \| tail` shows nothing on plug-in | Use a data cable |
| Not permitted to open the port | `PermissionError: /dev/ttyACM0` | `sudo usermod -aG dialout $USER`, then log out and back in |
| Something else holds the port | `sudo fuser -v /dev/ttyACM0` | Close QGroundControl / Mission Planner / MAVProxy, or stop your own service |
| Linux serial console owns the UART | Only on a Pi's built-in `/dev/serial0` | `raspi-config` -> Interface -> Serial: login shell **no**, hardware **yes** |
| Nothing is powered | No lights, no enumeration | Check that the FC is powered independently of the companion |

Network links fail differently:

| Cause | Fix |
|---|---|
| Wrong direction | `udp:0.0.0.0:14540` **binds and waits**. If the other side expects you to speak first, use `udpout:<host>:<port>` |
| Wrong port | PX4 SITL: `14540` for offboard APIs, `14550` for a GCS. ArduPilot SITL: TCP `5760` |
| A router is in the path with no endpoint for you | Add an endpoint in `mavlink-router.conf` or MAVProxy's `--out` |
| Firewall | `sudo tcpdump -n -i any udp port 14540` -- if you see traffic here but not in your app, it is a bind/firewall issue |

---

## 2. Heartbeat arrives but nothing else does

The link is fine. Nobody has asked for the data.

| Stack | Cause | Fix |
|---|---|---|
| **PX4** | PX4 streams a fixed set per `MAV_x_MODE` and nothing more | Request each message with `MAV_CMD_SET_MESSAGE_INTERVAL`. `mavbridge.rates.RateManager` does this. Set `MAV_x_MODE = 2` (Onboard) on the companion port |
| **PX4** | Requests are per-connection and are lost on reconnect | Re-request after every reconnect. `MavLink.receive_loop` gives you the hook |
| **ArduPilot** | Stream rates are zero on that port | `SRn_EXTRA1` (ATTITUDE), `SRn_POSITION` (GLOBAL_POSITION_INT), `SRn_EXT_STAT` (SYS_STATUS, GPS_RAW_INT), `SRn_RC_CHAN` (RC_CHANNELS), `SRn_EXTRA2` (VFR_HUD), `SRn_RAW_SENS` (IMU) |
| **ArduPilot** | You set `SR1_*` but you are on USB | USB is `SERIAL0`, so the parameters are `SR0_*` |
| **Both** | A GCS is also connected and set its own rates | Re-request your intervals every 10-30 s rather than once at startup |
| **Both** | You asked for a message the firmware does not emit | Check the `COMMAND_ACK`: `UNSUPPORTED` means the command is not implemented; on ArduPilot before 4.0 fall back to `REQUEST_DATA_STREAM` (`RateManager.fall_back_to_legacy()`) |

`mavbridge.watchdog` distinguishes "never arrived" (`STREAM_MISSING`) from
"arrived and then stopped" (`STREAM_STALE`), because the first is a
configuration problem and the second is a link problem.

---

## 3. Telemetry freezes after N seconds

Three different faults look identical from a distance. Tell them apart before
you start changing things.

| What you see | What it is | Fix |
|---|---|---|
| Heartbeat stops too | Link is down. Cable, radio, power, or the FC rebooted | Section 1, and check `dmesg` for USB disconnects |
| Heartbeat continues, one stream stops | That stream's rate request was lost or overridden | Section 2. Re-request intervals periodically |
| Heartbeat continues, packets keep arriving at full rate, but the **contents never change** | Frozen data: something in the path is replaying the last packet -- a router with a stuck endpoint, a radio buffer, or a serial driver handing back stale bytes after a brownout | Power-cycle the FC and watch whether `time_boot_ms` restarts. If it only happens under load, you are saturating the link (section 5) |
| Everything catches up in a burst after a pause | Link saturated, packets queued in the radio | Section 5 |

The third row is the one a plain heartbeat check cannot see, and the reason
`mavbridge.watchdog` tracks payload timestamps and not just arrival times.

Timestamps going **backwards** is a separate signal:

- A jump back to near zero from a large value = the autopilot rebooted.
  Check power, and on a Pi check the USB current limit.
- Small, repeated regressions = two sources on one link. Two vehicles, a SITL
  and a real FC on the same UDP port, or duplicate MAVLink system ids
  (`SYSID_THISMAV` on ArduPilot, `MAV_SYS_ID` on PX4). Filter on system id or
  give each source its own port.

---

## 4. Garbled bytes, framing errors, CRC failures

You are getting data but it does not parse, or it parses intermittently.

| Cause | How to confirm | Fix |
|---|---|---|
| Baud mismatch | `xxd` output is dense random bytes with no `0xFD`/`0xFE` start markers at regular spacing | Match the baud on both sides; probe if unsure |
| Parity/stop-bit mismatch | Rare, but happens with USB-serial bridges configured elsewhere | MAVLink is 8N1 everywhere; make sure nothing set 8E1 |
| Flow control on one side only | Works at 57600, fails at 921600, worse when the link is busy | Enable RTS/CTS on both sides **and wire the two extra pins**: ArduPilot `BRD_SERn_RTSCTS`, PX4 `SER_TELn_CTS`/`RTS`. If you cannot wire them, disable flow control on both sides and lower the baud |
| Voltage-level mismatch | Occasional corruption, works only over short wires | FC UARTs are 3.3 V. Do not connect a 5 V adapter without a level shifter |
| Long or unshielded wires next to ESCs | Corruption correlates with throttle | Shorten, route away from power wiring, twist the pair |
| MAVLink 1 vs 2 confusion | Some messages parse, newer ones do not | Force MAVLink 2: ArduPilot `SERIALn_PROTOCOL = 2`, PX4 `MAV_PROTO_VER = 2` |

---

## 5. Rates are lower than requested / telemetry lags then bursts

You asked for 50 Hz attitude on a telemetry radio. You are not going to get it.

The arithmetic, which is worth internalising:

- 8N1 serial spends **10 bits per byte** (1 start + 8 data + 1 stop).
  57600 baud = **5760 bytes/s**, not 7200.
- A MAVLink 2 frame costs **12 bytes of overhead** on top of the payload,
  so a 28-byte `ATTITUDE` is 40 bytes on the wire.
- ATTITUDE at 50 Hz = 2000 B/s. That is already a third of a 57600 link
  before anything else, and a SiK radio delivers well under its nominal byte
  rate once error correction and the half-duplex duty cycle are accounted for.

```python
from mavbridge.rates import RateRequest, estimate_bandwidth, check_link_budget, COMMON_LINKS
est = estimate_bandwidth([RateRequest("ATTITUDE", 50), RateRequest("GLOBAL_POSITION_INT", 20)])
print(check_link_budget(est, COMMON_LINKS["sik57600"]).warnings)
```

Fixes, in order of effectiveness:

1. Ask for less. Cut ATTITUDE and GLOBAL_POSITION_INT first; they dominate.
2. Move the high-rate work to a wired UART between FC and companion at
   921600 with flow control, and keep the radio for supervision only.
3. Raise the radio's air data rate (SiK `AIR_SPEED`) -- but that costs range,
   and both radios must match.
4. Do not run parameter downloads or mission transfers on a saturated link and
   then wonder why telemetry stalled.

---

## 6. Works over USB, fails on TELEM1/TELEM2

Extremely common, and the two paths differ in ways that matter.

| Difference | Consequence |
|---|---|
| USB CDC-ACM ignores the baud setting entirely | Your "921600" works on USB and means nothing; on a UART it must match exactly |
| USB is usually a different serial index | ArduPilot: USB is `SERIAL0`, so `SR0_*` and `SERIAL0_PROTOCOL` apply, not `SR1_*`. PX4: USB is its own MAVLink instance |
| TELEM ports often default to a GCS-oriented message set | **PX4**: set `MAV_x_MODE = 2` (Onboard) on the companion port. **ArduPilot**: set the `SRn_*` rates for that port |
| TELEM ports may have flow control expectations | ArduPilot `BRD_SERn_RTSCTS = 0` to disable if you have not wired RTS/CTS |
| USB provides power; a TELEM port does not | The companion must have its own supply |
| 3.3 V logic on TELEM, 5 V-tolerant USB | Level-shift if your adapter is 5 V |

Checklist for moving from USB to TELEM2 on ArduPilot: `SERIAL2_PROTOCOL = 2`,
`SERIAL2_BAUD = 921` (or 57), `BRD_SER2_RTSCTS = 0`, `SR2_*` rates non-zero.
On PX4: `MAV_1_CONFIG = TELEM2`, `MAV_1_MODE = 2`, `SER_TEL2_BAUD = 921600`.
Reboot the FC after changing `*_CONFIG` or `SERIALn_PROTOCOL` -- neither stack
applies those live.

---

## 7. `/dev/ttyACM0` becomes `/dev/ttyACM1` after a reboot

`ttyACM*` and `ttyUSB*` numbers are assigned in USB enumeration order. Boot
with a modem attached, replug in a different order, or have the FC re-enumerate
after a reboot, and your service opens the wrong device and waits forever for a
heartbeat that will never come.

Fix: use the stable name.

```bash
ls -l /dev/serial/by-id/
# usb-ArduPilot_Pixhawk1-1M_1E0033000751393034373338-if00 -> ../../ttyACM0
```

```python
from mavbridge.link import discover_serial_ports
for port in discover_serial_ports():
    print(port.score, port.kind, port.path, port.reason)
```

`mavbridge` prefers `by-id` paths, then `ttyACM*`, then `ttyUSB*`, and breaks
ties deterministically so a service picks the same port on every boot. If you
have two identical flight controllers, add your own udev rule keyed on the
USB serial number and give each a name you choose.

---

## 8. USB power brownout on a Raspberry Pi

Symptoms: the link drops every few minutes, `dmesg` shows USB resets or
`over-current change`, timestamps restart from zero, or the whole Pi reboots.

| Check | Command / fix |
|---|---|
| Did the 5 V rail dip? | `vcgencmd get_throttled` -- anything non-zero means it did |
| USB kernel events | `dmesg -T \| grep -i -E 'usb\|over-current\|reset'` |
| Powering the Pi from the FC's 5 V rail | Don't. Give the Pi its own regulator sized for its peak draw, not its average |
| Powering the FC from the Pi's USB | Also don't; USB current limits are low and shared |
| Cheap BEC shared with servos | Servo current spikes brown out the companion. Separate rails |
| Battery sag under load | See section 9 -- a tired pack browns out everything downstream |

A brownout that resets the FC shows up in `mavbridge` as an
`AUTOPILOT_REBOOT` event (timestamps jumping back to near zero), not as a
plain link drop -- which is how you tell it apart from a bad cable.

---

## 9. Battery looks fine but the vehicle behaves badly

`battery_remaining` is an estimate, and on a pack with no current sensor it is
close to fiction. Voltage **sag under load** is the honest early signal:

- Resting at 25.0 V (6S), dropping to 22.0 V at 40 A is 0.5 V/cell of sag --
  a tired pack, an undersized pack, or a bad connector.
- `mavbridge.telemetry.BatteryMonitor` tracks the resting reference and
  reports `sag_v` and an estimated pack internal resistance.
- If the companion computer shares that pack through a BEC, its brownouts will
  look like link drops to you. Section 8.

Note also that `SYS_STATUS.current_battery` is `-1` when there is no current
sensor. `mavbridge` reports `None`, not `-0.01 A`.

---

## 10. A ground station is connected, so the companion misbehaves

| Symptom | Cause | Fix |
|---|---|---|
| Cannot open the port at all | The GCS holds the USB port exclusively | Close it, or connect the GCS through a router instead |
| Streams change rate or stop when the GCS connects | The GCS set its own stream rates on the shared link | Re-request your intervals periodically; prefer separate ports for GCS and companion |
| Commands are accepted but appear to do nothing | Two components are commanding the vehicle | Give your companion a distinct component id (191, `MAV_COMP_ID_ONBOARD_COMPUTER`) and keep the GCS on 190 |
| Duplicate system ids | Two things claim to be system 1 | `SYSID_THISMAV` (ArduPilot) / `MAV_SYS_ID` (PX4) must be unique per vehicle |

The right architecture on a shared link is a router (`mavlink-router` or
MAVProxy) with one endpoint per consumer, rather than several processes
fighting over one serial port.

---

## 11. Works in SITL, fails on hardware

| Difference | What bites you |
|---|---|
| SITL links are fast and lossless | Rates that work over UDP on localhost saturate a 57600 radio. Budget with `mavbridge.rates` |
| SITL has a perfect GPS immediately | Real vehicles need a fix and an EKF that has converged; arming and position modes are refused until then |
| SITL has no serial layer | Baud, flow control, TX/RX and grounds are all untested until you are on hardware |
| SITL timing is generous | An offboard loop that is "fast enough" in SITL may miss PX4's 2 Hz setpoint floor on a loaded Pi. Stream at 20 Hz and watch the deadman |
| SITL never browns out | Section 8 |
| SITL parameters are defaults | Your real vehicle has `SRn_*` / `MAV_x_MODE` set by whoever configured it last |

---

## 12. Offboard / GUIDED will not engage

| Cause | Stack | Fix |
|---|---|---|
| Setpoints were not already streaming | **PX4** | PX4 requires setpoints at >2 Hz **before** the mode change, and drops out if they stop for ~0.5 s. Stream first, wait ~1 s, then request OFFBOARD. `OffboardController.engage()` does this |
| Setpoint stream stalls mid-flight | **PX4** | Offboard-loss failsafe triggers. Keep the stream at 20 Hz; the `mavbridge` deadman raises instead of silently sending a stale setpoint |
| No position estimate | Both | Position setpoints need a valid local position. Indoors, use velocity or body-frame setpoints, or supply an external position source |
| Not armed | Both | Arm first, and decode the `COMMAND_ACK` -- `FAILED` almost always has a `STATUSTEXT` explaining which pre-arm check failed |
| RC override | **ArduPilot** | Moving the sticks can knock the vehicle out of GUIDED depending on configuration |
| Wrong mode number | Both | PX4 uses `MAV_CMD_DO_SET_MODE` with main mode 6; ArduPilot uses the vehicle's own mode table (GUIDED is 4 on copter, 15 on plane) |
| Command rejected as `TEMPORARILY_REJECTED` | Both | Something is not ready yet -- usually the EKF or the setpoint stream. Retry, do not force |

Reproduce the failure offline before you take it to the flight line:

```bash
python3 examples/offboard_square.py --sim --skip-prestream   # PX4 refuses
python3 examples/offboard_square.py --sim                    # correct sequence
```

---

## Quick reference: parameters people get wrong

| Goal | PX4 | ArduPilot |
|---|---|---|
| Enable MAVLink on a port | `MAV_x_CONFIG` | `SERIALn_PROTOCOL = 2` |
| Set the baud | `SER_TELn_BAUD` | `SERIALn_BAUD` |
| Companion-oriented message set | `MAV_x_MODE = 2` (Onboard) | set `SRn_*` rates |
| Force MAVLink 2 | `MAV_PROTO_VER = 2` | `SERIALn_PROTOCOL = 2` |
| Flow control | `SER_TELn_CTS` / `RTS` | `BRD_SERn_RTSCTS` |
| System id | `MAV_SYS_ID` | `SYSID_THISMAV` |
| Attitude stream rate | `MAV_CMD_SET_MESSAGE_INTERVAL` | `SRn_EXTRA1` |
| Position stream rate | `MAV_CMD_SET_MESSAGE_INTERVAL` | `SRn_POSITION` |

Changes to `*_CONFIG` and `SERIALn_PROTOCOL` require a flight-controller
reboot on both stacks.
