# Brand-New Jetson Orin Nano: First Boot Guide

This guide is for the person who has a **brand-new Jetson Orin Nano Developer Kit** and has not even booted it once.

This is the "do not assume anything" version.

It explains:
- what hardware to prepare
- how to get the Jetson to its first Ubuntu desktop
- what to do right after first boot
- how to get it ready for the JetCar repo

---

# Before you start

This guide assumes you are using the **Jetson Orin Nano Developer Kit** style setup.

The simplest beginner path is:

1. prepare a microSD card on another computer
2. boot the Jetson from that microSD card
3. complete the Ubuntu first-boot wizard
4. connect to the internet
5. do basic checks
6. then move on to the JetCar repo setup

If NVIDIA changes the exact image name, JetPack version label, or download button text, always trust the **current official NVIDIA getting-started page** over any screenshot or older tutorial.

---

# 1. What you need

Prepare these items first:

## Required
- Jetson Orin Nano Developer Kit
- official or known-good power supply for the dev kit
- microSD card from a reliable brand
- one computer to prepare the microSD card
- monitor
- HDMI or DisplayPort cable, depending on your setup
- USB keyboard
- USB mouse

## Strongly recommended
- wired Ethernet connection for first setup
- a second computer so you can read docs while setting up the Jetson
- a label or notebook to write down the Jetson username, password, and IP address

---

# 2. The easiest first-boot path

For a complete beginner, the easiest path is usually:

## Option A — official prebuilt microSD image

This is the easiest path for most students.

Use the official NVIDIA Jetson Orin Nano Developer Kit getting-started page and download the official microSD image or follow the current official flashing workflow for the dev kit.

Then write that image to the microSD card from your normal computer.

## Option B — SDK Manager on a host computer

This is the more advanced path.

Use this only if you already know you need SDK Manager, or the official guide for your current JetPack release tells you to use it.

For classroom or first-time workshop use, **Option A is usually easier**.

---

# 3. Prepare the microSD card

On your normal computer:

1. go to the official NVIDIA Jetson Orin Nano getting-started page
2. download the current official image or follow the official flashing instructions for the dev kit
3. use the recommended flashing tool from the official instructions
4. write the image to the microSD card
5. safely eject the microSD card

## Very important beginner note

Do not just copy the downloaded file onto the microSD card like a normal file.

You must **flash / image** the card using the proper tool so the card becomes bootable.

---

# 4. Insert the card and connect the hardware

With power disconnected:

1. insert the prepared microSD card into the Jetson
2. connect the monitor
3. connect keyboard and mouse
4. connect Ethernet if available
5. connect the power supply last

Why power last?
Because it avoids confusion during setup and reduces the chance of hot-plug mistakes while you are still learning the ports.

---

# 5. First power-on

Now power on the Jetson.

What you should expect:
- the board powers up
- the display shows the NVIDIA / Ubuntu boot process
- after some time, the first-boot setup wizard appears

## If it feels slow
First boot can take a while.
Do not panic immediately if it is not instant.

---

# 6. Complete the first-boot Ubuntu wizard

The first-boot wizard usually asks for things like:
- language
- keyboard layout
- time zone
- username
- computer name
- password
- network settings

## What to do
Pick simple values that you will remember.

For example:
- username: `orin`
- hostname: something obvious like `jetson-orin-nano`

Write down:
- username
- password
- hostname

Do not rely on memory.

---

# 7. Get to the desktop and verify the basics

Once the first-boot wizard is done and the desktop appears, check these basics:

## A. Internet
Make sure the Jetson can access the internet.

For beginners, wired Ethernet is easiest.

## B. Terminal
Open a terminal and confirm it works.

## C. Storage
Make sure the system looks normal and the boot media is visible.

## D. Mouse / keyboard / display
Make sure the basic desktop session is stable before doing anything complicated.

---

# 8. The first terminal commands to run

After you reach the desktop, open a terminal and run:

```bash
whoami
hostname
pwd
```

This confirms:
- which user you are logged in as
- what the machine name is
- where you are in the filesystem

Then check the Jetson software info with commands appropriate to the installed image.

Two commonly useful checks are:

```bash
cat /etc/nv_tegra_release
```

and/or

```bash
dpkg-query -W nvidia-jetpack
```

If one of these does not work on your image, do not panic.
The exact package layout can vary by release.

The main goal is to confirm the board really booted into the NVIDIA Jetson software stack and not some random unsupported image.

---

# 9. Update carefully

Once the internet works, do a basic package refresh:

```bash
sudo apt update
```

You may also choose to do:

```bash
sudo apt upgrade -y
```

But for beginners, keep this rule in mind:

**Do not do random system modifications before you confirm the Jetson already boots and behaves normally.**

If you are following a class or workshop image, it is often safer to confirm the expected JetPack setup first, then update with intention.

---

# 10. Create a working folder for projects

A clean beginner habit is to keep projects under the home folder.

For example:

```bash
cd /home/$USER
mkdir -p Projects
```

For the JetCar repo, the current docs assume:

```bash
cd /home/orin
```

and then clone the project there.

So if you are using the username `orin`, the repo path becomes:

```bash
/home/orin/JetCar
```

---

# 11. Ready the Jetson for serial projects

Because JetCar uses serial communication to the rover controller, this is a good early check.

List likely serial devices:

```bash
ls /dev/ttyTHS* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

You may also need your user in the `dialout` group:

```bash
sudo usermod -aG dialout $USER
```

If you do this, log out and log back in, or reboot.

---

# 12. Reboot once before serious work

After first boot, network setup, and any user-group changes, a clean reboot is a good idea.

```bash
sudo reboot
```

After reboot, confirm again:
- desktop appears normally
- internet works
- terminal works
- your username is correct

---

# 13. Now the Jetson is ready for the JetCar repo

Once the board boots normally and the desktop works, you can move to the project setup.

For the JetCar repo, the clone step is:

```bash
cd /home/orin
git clone https://github.com/patricklau12/JetCar.git JetCar
cd /home/orin/JetCar
```

Then continue with the repo setup guide and notebook `00_package_install_and_check.ipynb`.

---

# 14. Common beginner mistakes

## Mistake 1 — copying the image file to the card instead of flashing it
Wrong.
You must use the proper image-writing workflow.

## Mistake 2 — forgetting to insert the microSD card before power-on
Then the board may not boot the way you expect.

## Mistake 3 — bad power supply
A weak or wrong power supply causes confusing failures.

## Mistake 4 — changing too many things before confirming first boot works
First confirm the board boots normally.
Then customize.

## Mistake 5 — forgetting username or password
Write them down.

## Mistake 6 — assuming UART / serial permissions will just work
Check device nodes and `dialout` access early.

---

# 15. If the Jetson does not boot

Work through these in order:

1. check the power supply
2. check the monitor cable and display input source
3. remove and re-seat the microSD card
4. reflash the microSD card from scratch
5. re-read the current official NVIDIA getting-started page for your exact dev kit and release

Do not jump straight into complicated debugging if basic boot media preparation may be wrong.

---

# 16. The simplest student summary

If someone says, "I have a brand-new Orin Nano and know nothing," the shortest safe answer is:

1. use the official NVIDIA getting-started page
2. prepare the official boot media correctly
3. finish the first Ubuntu setup wizard
4. connect internet
5. reboot once
6. clone JetCar
7. run the JetCar setup docs from notebook `00`

That is the safest beginner path.