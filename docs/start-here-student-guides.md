# Start Here: Student Guides

This page is the first doc GitHub visitors should follow.

If you are new to Jetson, robot cars, or this repo, start with these pages.

They are ordered from the most basic setup step to the more advanced project concepts.

---

## Chapter 0. If your Jetson Orin Nano is brand new

Read this first:

- [Brand-New Jetson Orin Nano: First Boot Guide](./brand-new-jetson-orin-nano-first-boot.md)

Use this if you have not even completed the first Ubuntu boot wizard yet.

It explains:
- how to prepare the boot media
- how to do the first power-on
- what to check after the first desktop appears
- how to get ready for the JetCar repo

---

## Chapter 1. If the Jetson already boots and you want the project workflow

Read this next:

- [Command Workflow](./command-workflow.md)

Use this to understand:
- which command to run first
- what each chapter does
- what order to follow
- when you are ready to move on

---

## Chapter 2. If you want the chapter-by-chapter learning version

Read this next:

- [Notebook Chapters for Students](./notebook-chapters-for-students.md)

Use this to understand:
- what each learning chapter does
- what you should learn from each chapter
- how the old notebook flow maps to the project

---

## Chapter 3. If you want to understand how the line-following algorithm works

Read this next:

- [Line Following Explained for Students](./line-following-explained-for-students.md)

Use this to understand:
- what the mask is
- what a contour is
- why the lower image region matters most
- how shift and angle become movement decisions
- how the Jetson sends serial motor commands to the rover

---

## Chapter 4. If you want the more structured maintainer or teacher versions

These pages are more formal and a bit more detailed:

- [Notebook Chapters Guide](./notebook-chapters-guide.md)
- [How JetCar Works: Line Following, Masking, and Motor Communication](./how-it-works-line-following-and-control.md)

These are useful for:
- teaching assistants
- workshop leaders
- maintainers
- anyone writing new docs or improving the notebooks

---

## Chapter 5. Supporting docs

After reading the pages above, you can continue with the existing quickstart and repo docs:

- [Beginner Quickstart](./beginner-quickstart.md)
- [Hardware Checklist](./hardware-checklist.md)

---

## Recommended order for a complete beginner

1. Chapter 0: first-boot guide
2. Chapter 1: command workflow
3. Chapter 2: notebook chapters for students
4. Chapter 3: line following explained for students
5. then start command chapter `00`

That order is usually the least confusing path.

---

## Real clone command for this repo

If you need the actual clone command, use:

```bash
cd /home/orin
git clone https://github.com/patricklau12/JetCar.git JetCar
cd /home/orin/JetCar
```
