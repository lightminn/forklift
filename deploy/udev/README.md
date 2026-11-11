# udev rule for the MCU link (USB CDC)

`99-forklift-mcu.rules` matches the firmware's USB identity (ST VCP `0483:5740`, product string starting with
`forklift-mcu `). It does three things:

- tells ModemManager to leave the port alone (`ID_MM_DEVICE_IGNORE=1`) — its AT probes would otherwise reach the
  MCU's frame decoder (they are thrown away by the CRC, but they cost the link);
- names the port `/dev/forklift-mcu` (and `/dev/forklift-mcu-<serial>`, the MCU's UID, when several are plugged in);
- lets the `dialout` group open it.

The rule is a host-side premise of the firmware's link design (`docs/design/2026-11-12-weact-h743-board-firmware.md`
section 5). The other two live in `tools/real_mcu_link.py`: DTR is never lowered after the open, and every opening
writes one `0x00` delimiter before its first frame.

## Install (laptop and Jetson; needs sudo — run it yourself)

From the repository root:

```bash
sudo install -m 644 deploy/udev/99-forklift-mcu.rules /etc/udev/rules.d/
sudo udevadm control --reload
sudo udevadm trigger --subsystem-match=tty --subsystem-match=usb
```

Your user must be in `dialout` (`groups`; else `sudo usermod -aG dialout $USER` and log in again).

## Check (with the MCU plugged in)

```bash
ls -l /dev/forklift-mcu                       # a link to /dev/ttyACM<n>
udevadm info -q property /dev/forklift-mcu | grep -E 'ID_MM_DEVICE_IGNORE|ID_MODEL|ID_SERIAL_SHORT'
# ID_MM_DEVICE_IGNORE=1 must be listed
```

Then the tools take `--device /dev/forklift-mcu` (USB CDC ignores `--baud`; the UART7 alternative uses its adapter's
path with `--baud 460800`).

## When the link goes silent

There is no automatic USB reset (lead decision 2026-11-12): if the MCU's USB stack stops, the truck stops by command
expiry and the teleop bridge's console says so (session row: no STATE — cycle the main power). After the power cycle,
check `reset_cause` in the first STATE (`tools/bringup.py record`).
