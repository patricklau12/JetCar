# Jetson-IO Header UART Boot Entry Check

A fresh Jetson can show the correct 40-pin header pin labels inside `jetson-io.py` and still fail to drive the rover over `/dev/ttyTHS1`.

## Important Runtime Check

After using Jetson-IO, do not stop at the menu screen.

You must save/apply the header configuration and reboot, then confirm that the boot config actually changed.

Check:

```bash
grep -n . /boot/extlinux/extlinux.conf 2>/dev/null
grep -n . /boot/firmware/extlinux/extlinux.conf 2>/dev/null
```

## What The Working Board Looks Like

A working board may show something like:

- `DEFAULT JetsonIO`
- a `LABEL JetsonIO` section
- a `MENU LABEL Custom Header Config: ...`
- an `OVERLAYS ...` line

That means the Jetson is actually booting the Jetson-IO-generated hardware config.

## What The Non-Working Board Looks Like

If the file only shows something like:

- `DEFAULT primary`
- only a `LABEL primary` section
- no `LABEL JetsonIO`
- no `OVERLAYS ...` line for the custom config

then the Jetson-IO header change was not applied at boot, even if the Jetson-IO menu screen looked correct.

## Why This Matters

For JetCar, the rover control path uses the Jetson header UART and expects `/dev/ttyTHS1` to be electrically routed to the header pins used for the rover serial wiring.

If the board is still booting the default entry instead of the Jetson-IO custom entry, the menu can look correct while the header UART is still not active in the way the rover wiring expects.

## Fix

1. Run Jetson-IO again:

```bash
sudo /opt/nvidia/jetson-io/jetson-io.py
```

2. Set the 40-pin header for the desired UART/camera config.
3. Save/apply the configuration.
4. Reboot.
5. Re-run the `extlinux.conf` check above.
6. Confirm you now see the `JetsonIO` boot entry and related overlay lines.

## JetCar Serial Retest

After the boot entry is updated, re-test rover serial control:

```bash
cd /home/orin/JetCar
source .venv/bin/activate
python scripts/wave_rover_serial.py --port /dev/ttyTHS1 raw --json '{"T":1,"L":0.30,"R":0.30}'
sleep 1
python scripts/wave_rover_serial.py --port /dev/ttyTHS1 stop
```
