# Compare The Local `wave_rover_serial.py` Against The Repo Version

A common trap during bring-up is assuming that the local script on an older working Jetson is identical to the script in the GitHub repo.

## Why This Matters

The repo version of `scripts/wave_rover_serial.py` only supports:

- `stop`
- `forward`
- `backward`
- `left`
- `right`
- `raw`

It does **not** support options such as:

- `--listen`
- `--duration`
- `feedback`
- `echo_on`

So if an older working Jetson accepts those options, then it is running a different local script than the repo copy.

That means:

- the old board may not actually be using the same code path as the fresh Jetson
- a behavior difference may be caused by a local script modification, not by Jetson UART hardware alone

## Check On Both Jetsons

```bash
cd /home/orin/JetCar
sha256sum scripts/wave_rover_serial.py
python scripts/wave_rover_serial.py -h
grep -nE 'listen|duration|feedback|echo_on' scripts/wave_rover_serial.py
```

## What To Look For

- If the SHA differs, the files are not identical.
- If the help output differs, the files are not identical.
- If the old Jetson script contains `listen`, `duration`, `feedback`, or `echo_on`, then the old board is using a local custom script.

## Fastest A/B Test

If the old script is different, copy it from the old board to the new board and test the exact same command path.

Example:

```bash
scp orin@OLD_JETSON_IP:/home/orin/JetCar/scripts/wave_rover_serial.py /tmp/wave_rover_serial.old.py
scp /tmp/wave_rover_serial.old.py orin@NEW_JETSON_IP:/home/orin/JetCar/scripts/wave_rover_serial.py
```

Then on the new Jetson:

```bash
cd /home/orin/JetCar
source .venv/bin/activate
python scripts/wave_rover_serial.py -h
```

If the new board starts working after copying the old local script, the issue was code-path mismatch rather than Jetson UART setup.
