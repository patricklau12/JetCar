# Hardware Checklist

Use this checklist before students arrive.

## Core Hardware

- Jetson Orin Nano with stable power supply
- enough free storage space
- RGB camera connected firmly
- rover chassis powered and charged
- serial wiring between Jetson and rover confirmed
- USB keyboard, mouse, and display ready for first setup

## Pre-Workshop Software Check

- Repo is cloned in `/home/orin/JetCar`
- `git pull` has been run recently
- Python environment installs without errors
- the web panel can start
- one browser can open `http://127.0.0.1:8765`

## Camera Check

- live RGB image appears
- mask preview appears
- correct camera source is selected: `usb` or `csi`

## Motor Check

- serial status shows ready
- `Stop` works immediately
- manual `Forward`, `Back`, `Left`, and `Right` all respond while the rover is on a stand

## Calibration Check

- `forward` minimum is saved
- `back` minimum is saved
- `left` minimum is saved
- `right` minimum is saved
- calibration was done on the actual venue floor or a similar surface

## Safety

- keep the rover on a stand during first motor tests
- use a physical kill method if available
- do not start floor testing until camera, serial, and calibration all work
