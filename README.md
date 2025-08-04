# Core System 2026  
_Core System UdeGSpace Mars Rover [name]_

![repo-banner](Docs/Images/core_banner.png) <!-- optional -->

---

## 1 · Purpose

This repository holds **all hardware, firmware and documentation** that make up the
“Core System” of the 2026 rover:

* Real-time Main Cotrol Unit  
* Navigation sensors (GNSS, IMU, Barometer)  
* Jetson Nano interface
* RoboClaw Connectors
* Mini-PCIe Wi-Fi HaLow socket  
* CAD / PCB data and reference PDFs  


---

## 2 · Repo layout

| Folder | What’s inside |
|--------|---------------|
| `Hardware/` | KiCad projects, 3-D models, BOMs |
| `Software/` | Firmware (`rover-coreio`), MicroPython prototypes (`uPy`), sensor drivers |
| `Docs/` | Datasheets, block diagrams, specs |
| `LICENSE` | Project licence (MIT by default) |

---

## 3 · Branch workflow (quick view)

```text
main      ← protected, tagged releases only
develop   ← integration / peer-reviewed code
hw/*      ← hardware feature branches   (e.g. hw/mpcie-socket)
sw/*      ← software feature branches   (e.g. sw/imu-driver)
misc/*    ← docs, CI, repo-wide chores

