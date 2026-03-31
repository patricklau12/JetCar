# Wave Rover UART Wiring with Jetson Orin Nano

This page explains the basic serial wiring between the **WAVE ROVER** controller and the **Jetson Orin Nano Developer Kit**.

This should be mentioned clearly in the project docs because many beginner problems come from wiring mistakes, not code mistakes.

---

## Important idea first

For this project, the Jetson is **not** supposed to drive the wheel motors directly from GPIO.

Instead:
- the Jetson runs the vision and control logic
- the Jetson sends **serial JSON commands** to the rover controller
- the rover controller board actually drives the motors

So the connection is mainly a **UART serial connection**, not a direct motor-control GPIO connection.

---

## What should be documented

At minimum, the docs should mention:

- which Jetson header pins are used for UART
- that TX and RX must be crossed
- that GND must be shared
- that this is serial communication, not direct motor driving
- that Linux serial permission issues may need the `dialout` group

---

## Common Jetson UART pin reminder

A commonly used UART wiring reminder for this project is:

- **pin 6** = GND
- **pin 8** = TX
- **pin 10** = RX

That means the usual connection rule is:

- Jetson **GND** -> rover **GND**
- Jetson **TX** -> rover **RX**
- Jetson **RX** -> rover **TX**

This is the classic UART rule:

**TX goes to RX, and RX goes to TX**

Do not connect TX to TX or RX to RX.

---

## Why beginners get confused

Beginners often hear "GPIO pins" and assume every connection is the same kind of GPIO usage.

That is misleading here.

Yes, the UART pins are on the Jetson header, but the important concept is:

- these pins are being used as a **serial port**
- the Jetson is sending command data
- the rover controller interprets the command and drives the motors

So the project is using header pins for **communication**, not raw motor control.

---

## The basic software side

The rover controller is sent JSON lines such as:

```json
{"T":1,"L":0.16,"R":0.16}
```

That means the Jetson needs access to the correct serial device, commonly something like:

```bash
/dev/ttyTHS1
```

The exact port can vary by setup, so it is good to check available devices.

---

## Useful checks on the Jetson

List likely serial devices:

```bash
ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

Check Linux group membership:

```bash
groups
```

If needed, add the current user to `dialout`:

```bash
sudo usermod -aG dialout $USER
```

Then log out and log back in, or reboot.

---

## What to tell students very clearly

A good beginner note is:

> The WAVE ROVER should be controlled over UART serial. The Jetson does not directly power or PWM the motors from its header. Make sure GND is shared, TX/RX are crossed, and the correct serial device is available in Linux.

---

## Good troubleshooting checklist

If the rover does not respond:

1. check GND is connected
2. check TX and RX are crossed
3. check the correct Jetson header pins were used
4. check the serial device exists in Linux
5. check the user has permission to access the serial device
6. check the rover power and controller are actually on

---

## Best practice for the repo docs

Yes, this should absolutely be mentioned in the docs.

The best places are:

- `README.md` in a short wiring reminder section
- the brand-new Jetson / first setup path
- the motor calibration chapter
- a dedicated UART wiring page like this one

That way beginners see it early, before they waste time debugging Python for a wiring problem.
