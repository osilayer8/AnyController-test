## Supported controllers

| Controller | Input | Status |
|---|---|---|
| Switch Pro Controller (`057e:2009`) | USB | Driver, works |
| DualShock 4 (`054c:05c4`, `054c:09cc`) | USB | Driver, works |
| DS4 v1 (`054c:05c4`) | USB | Driver, works |
| DS4 v2 (`045e:028e`) | USB | Driver, works |
| Switch Joy-Con (L/R) | USB | Driver, works |
| Xbox 360 / XInput (`045e:028e`) | USB cable or 2.4GHz dongle | Driver, works |
| 8BitDo wireless receiver, X-input mode (`2dc8:3106`) | USB dongle | Recognised, untested |

## Requirements

- jailbroken PS5 (or ASR fork) on firmware 13.60 with a trusted payload
- payload file named `anycontroller.elf` (see the [how-it-works](docs/how-it-works.svg) diagram)
- [ps5-payload-dev](https://github.com/ps5-payload-dev) toolchain (SDK v0.45, clang/ld.lld) — see [Building](#building)

## Usage

```sh
# send the payload to the PS5, then it watches USB and registers every
# supported device as a virtual DualSense for the system
python3 send_payload.py <PS5_IP> anycontroller.elf
```

## Button mapping

| Button | Switch Pro | DualShock 4 | Xbox | 8BitDo in X-input mode |
|---|---|---|---|---|
| Cross / A | X | Square | A | A |
| Circle / B | B | Cross | B | B |
| Square / X | X | Square | X | X |
| Triangle / Y | Y | Triangle | Y | Y |
| L1 / L2 | L1 / ZL | L1 / L2 (digital) | LB / L2 | LB / L2 |
| R1 / R2 | R1 / ZR | R1 / R2 (digital) | RB / R2 | RB / R2 |
| L3 / R3 | L / R stick click | L3 / R3 | LT / RT click | LT / RT click |
| D-pad | D-pad | D-pad | D-pad | D-pad |
| Options / Start | OPTIONS / START | OPTIONS / START | START / BACK | START / BACK |
| Create / Select | MINUS / PLUS | MINUS / PLUS | SELECT / CREATE | SELECT / CREATE |
| PS / Home | PS button | PS button | Guide | Guide |

Analog sticks and triggers keep their native direction and range. Battery
and rumble are not supported yet; the light bar is not supported at all.

## Building

Dependencies: the [ps5-payload-dev](https://github.com/ps5-payload-dev) toolchain
(verified with SDK v0.45, clang 18.1.3, ld.lld) and the payload SDK at
`/tmp/sdk/ps5-payload-sdk`.

```sh
bash build_ps5.sh        # make clean && make, prints "Send with: python3 send_payload.py <PS5_IP> anycontroller.elf"
python3 send_payload.py <PS5_IP> anycontroller.elf
```

## How it works

<p align="center">
  <img src="docs/how-it-works.svg" width="100%" alt="Controller, then USB access through ugen, then a driver, then the /dev/hid virtual pad, then the PS5 sees a DualSense">
</p>

- **Virtual DualSense:** `/dev/hid` accepts three ioctls (add device, insert input, delete device) that libScePad uses for virtual pads, and it doesn't check who is calling. A payload can register a pad the kernel treats as a real DualSense. Interface discovered by [FGG-XSense](https://github.com/FGGstore/FGG-XSense). See [`src/vpad.c`](src/vpad.c).
- **USB access:** the PS5 has no `uhid` device. Controllers appear only as FreeBSD `ugen` nodes (`/dev/usb/B.A.E`). AnyController identifies devices by their USB descriptor, detaches Sony's HID driver, claims the interface and reads reports from the interrupt endpoints. See [`src/transport/usb.c`](src/transport/usb.c).
- **Switch Pro Controller:** over USB it drops off the bus every second or so until the host sends the USB handshake and the "no timeout" command. The driver sends the same sequence as Linux's `hid-nintendo`, then enables full 0x30 reports and the motion sensor. See [`src/controllers/switch_pro.c`](src/controllers/switch_pro.c).
- **Xbox 360 / XInput:** standard 20-byte input reports. The driver sends the player-LED command on connect, which also makes multi-mode dongles stay in X-input mode instead of cycling. Adapters can pad the report past 20 bytes and can re-enumerate under their own vendor id once a pad links (an 8BitDo receiver becomes `2dc8:3106`), so the driver accepts any message of 20 bytes or more and is registered under both ids. See [`src/controllers/xbox360.c`](src/controllers/xbox360.c).
- **DualShock 4:** over USB the pad streams 64-byte input report `0x01` at ~250 Hz with no handshake needed, so the driver only unpacks the d-pad hat — sticks and triggers already arrive in PlayStation convention. See [`src/controllers/ds4.c`](src/controllers/ds4.c).
- **Brief drops:** if a controller drops off USB, its virtual DualSense is kept for 3 seconds. A quick reconnect then resumes on the same pad, without a disconnect or user re-pick.

### Adding a controller

Each controller type is a `ControllerDriver` (see [`include/controller.h`](include/controller.h)): a VID/PID, an optional `setup` step for handshakes, and a `parse_input` that turns raw reports into a `NormalizedInput`. Register it in [`src/controller.c`](src/controller.c).

## Debugging

AnyController logs to the kernel log. With a klog server running on the console (port 3232) you get a line for each step of the chain, which is the fastest way to see what a new device is doing:

```
[AnyController] usb 0.3: no driver for 2dc8:3107
[AnyController] usb 0.5: opened Xbox 360 Controller (045e:028e)
[AnyController] slot 0: Xbox 360 Controller 045e:028e -> vpad handle=14877453
[AnyController] slot 0: report 20 bytes, type 00 len 14: 00 14 00 00 00 00 00 80 ...
[AnyController] slot 0: buttons 0110: l1 up triggers 0/0
[AnyController] slot 0: 751 reports/5s (20..20 bytes), 0 unparsed, 0 submit failures
[AnyController] slot 0: USB lost, holding DualSense for 3000 ms
[AnyController] slot 0: identity 045e:028e -> 2dc8:3106 (8BitDo receiver), same pad
[AnyController] slot 0: 8BitDo receiver resumed on vpad handle=14877453
```

What each line is for:

- `usb ...: no driver for aaaa:bbbb` — a device the payload cannot use. When a wireless adapter is plugged in, this is where its identity shows up: an 8BitDo receiver reports `2dc8:3107` until a controller is paired to it, and that id is deliberately not registered because nothing is behind it yet.
- `opened ...` / `slot N: <name> vid:pid` — a device matched a driver, plus the interface and endpoints it got (`usb N.M: IN ep1, OUT ep2`).
- `report N bytes, type xx len yy` — printed once per distinct report format per connection. A new layout appearing mid-session means a pad was paired, swapped or re-enumerated under a new id.
- `cannot parse N-byte report` — reports arrive but the matched driver does not understand them. This is what "connected but the pad does nothing" looks like.
- `buttons xxxx: ...` — the first 60 button transitions of a connection, with the analog trigger values, so a mapping can be checked without watching the game.
- `reports/5s (20..32 bytes), N unparsed` — health line: rate, report length range and how many reports the driver dropped.
- `USB lost` / `did not come back` — the 3-second grace and its outcome.

If a device does not work, the lines worth reporting are the `no driver` / `opened` line, the `report ...` line and the first `cannot parse` lines.

## Not yet supported

- Bluetooth controllers
- Rumble from games, and the light bar
- Motion controls (gyro)
- Other controller families (Xbox One/Series, generic HID gamepads)
- The Create button bit is unverified

## Credits

- [FGG-XSense](https://github.com/FGGstore/FGG-XSense): discovery of the `/dev/hid` virtual pad interface
- [ps5-payload-dev](https://github.com/ps5-payload-dev): the payload SDK, ELF loader and klog server
- [dekuNukem/Nintendo_Switch_Reverse_Engineering](https://github.com/dekuNukem/Nintendo_Switch_Reverse_Engineering): Switch controller protocol
