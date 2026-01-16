# Repository Guidelines

## Project Structure & Module Organization

- `kukulcan/hardware/MCU/`: KiCad project files (`MCU.kicad_*`) and board exports (e.g., `MCU.stl`).
- `kukulcan/hardware/Libraries/`: shared footprints and 3D models.
- `kukulcan/hardware/mfr/`: manufacturing outputs (Gerbers, drills, job file, fab zip, BOM/CPL under `mfr/pcba/`).
- `kukulcan/hardware/MCU/jlcpcb/`: JLCPCB exports (`gerber/`, `production_files/`).
- `kukulcan/firmware/upy/`: MicroPython firmware (drivers and UI).
- `src/`: ROS 2 workspace root (packages will live under `src/<package_name>/`).
- `docs/`: datasheets, design notes, reference PDFs, and images.
- `build/`, `install/`, `log/`: generated artifacts (do not edit by hand).

## Build, Test, and Development Commands

- `idf.py build`: compile-check for ESP-IDF firmware (use within the relevant firmware subproject).
- `idf.py -p /dev/ttyUSB0 flash`: flash ESP-IDF firmware to hardware.
- `colcon build`: (future) ROS 2 workspace build from repo root once packages exist.

If a module ships its own README, prefer those commands over generic ones.

## Coding Style & Naming Conventions

- Branches: `sw/*` for software, `hw/*` for hardware, `misc/*` for docs/CI chores.
- ROS 2 packages should follow `src/<package_name>/` and standard ROS 2 naming.
- Keep commits small and focused; align changes with the branch scope.

## Testing Guidelines

- No repo-wide test runner yet; validate per subsystem.
- Firmware: use `idf.py build` for compile checks and run on target hardware when applicable.
- Hardware: verify updates in KiCad; share artifacts or screenshots in PRs if they affect schematics/PCB.

## Commit & Pull Request Guidelines

- Commit messages follow Conventional Commits with scoped types, e.g.:
  - `feat(sw-can): add FDCAN filter`
  - `fix(hw-mcu): correct USB D+ routing`
  - `docs(imu): add calibration guide`
- Keep the history clean: separate logical changes (design edits vs. exports), and squash before merging to `develop`.
- For manufacturing updates, prefer a dedicated commit like `hw(mfr): update MCU RevA gerbers`.
- Open PRs against `develop` and use squash-merge on approval.
- PRs should include: a short description, linked issue (if any), and test/validation notes.
- Use GitHub CLI when possible: `gh pr create --base develop --fill`.

## Security & Configuration Tips

- Avoid committing build artifacts; keep them in `build/` and `log/`.
- Manufacturing exports in `kukulcan/hardware/mfr/` and `kukulcan/hardware/MCU/jlcpcb/` are tracked artifacts; regenerate when the PCB changes and label revs consistently (e.g., `MCU_revA_*`).
