# Core System 2026  
_Core System Mars Rover UdegSpace 2026”_

<p align="center">
   <img src="docs/images/MarsRover.svg" alt="Mars Rover logo" height="200">
</p>

---

## 1 · Purpose

This repository holds the **hardware, firmware and documentation** that make up the
“Core System” of the 2026 rover Maya:

- Real-time Main Control Unit (MCU)  
- Navigation sensors (GNSS, IMU, barometer)  
- Jetson-based compute interface  
- Motor driver / RoboClaw connectors  
- Mini-PCIe Wi-Fi HaLow socket  
- CAD / PCB data and reference PDFs  

Over time this repo will also host the **ROS 2 workspace** that connects the core
system to the rest of the rover stack.

---

## 2 · Repo layout

Current top-level layout:

| Path                       | Contents                                                |
|----------------------------|--------------------------------------------------------|
| `kukulcan/hardware/`       | KiCad project for the MCU board, symbols, footprints, 3D models, generated outputs |
| `kukulcan/firmware/upy/`   | MicroPython firmware (BSW drivers and App UI) for sensors and display |
| `src/`                     | ROS 2 workspace root (will contain core ROS 2 packages) |
| `docs/`                    | Datasheets, design notes, reference PDFs              |
| `CONTRIBUTING.md`          | Contribution and workflow guidelines                   |
| `LICENSE`                  | Project license                                        |

As the project grows, additional firmware projects (PlatformIO, ESP-IDF, etc.)
will live under `kukulcan/firmware/<name>/`, and ROS 2 packages will live under
`src/<package_name>/`.

---

## 3 · Branch workflow (quick view)

```text
main      ← protected, tagged releases only
develop   ← integration / beta code

sw/*      ← short-lived software feature branches
hw/*      ← short-lived hardware feature branches
misc/*    ← docs, CI, repo-wide chores
````

Rules of thumb:

* **main**

  * Only updated via fast-forward from `develop` at known-good milestones.
  * Tag releases here (e.g. `v0.1.0-beta`).

* **develop**

  * Always buildable.
  * Integration branch for Kukulcan hardware + firmware + ROS 2 workspace.

* **feature branches (`sw/*`, `hw/*`, `misc/*`)**

  * Always branch from `develop`.
  * Squash-merge into `develop` when done, then delete the branch.

---

## 4 · Getting started (repo)

Clone and switch to the integration branch:

```bash
git clone git@github.com:MarsRoverUdeGSpace/Core_System_2026.git
cd Core_System_2026

# For day-to-day work
git checkout develop
```

For now:

* Hardware is under `kukulcan/hardware/MCU/` (open `MCU.kicad_pro` in KiCad).
* MicroPython firmware is under `kukulcan/firmware/upy/`.

ROS 2 packages will later be added under `src/` and built with `colcon` from the
repo root.

---

## 5 · Roadmap (high-level)

* [x] Normalize repo layout (`docs/`, `kukulcan/{hardware,firmware}`, `src/`)
* [x] Integrate MCU hardware design (KiCad)
* [x] Integrate uPy firmware (GNSS, IMU, BMP, OLED UI)
* [ ] Integrate ESP32 / PlatformIO core I/O firmware
* [ ] Seed ROS 2 packages in `src/` for core system bring-up
* [ ] Tag first stable beta and advance `main`

````

