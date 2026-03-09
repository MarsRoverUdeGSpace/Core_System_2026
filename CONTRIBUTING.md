# Contributing

Use `develop` as the integration branch. Keep `main` for tagged stable releases.

## Branches

- `sw/*` for software and firmware work
- `hw/*` for schematics, PCB, BOM, and fabrication outputs
- `misc/*` for docs, CI, and repository maintenance

## Workflow

1. Branch from `develop`
2. Keep the change small and focused
3. Validate the affected subsystem locally
4. Open a PR against `develop`
5. Squash-merge after review

## Commits

Use Conventional Commits when possible:

- `feat(kukulcan-fw): add encoder publisher`
- `fix(hw-mcu): correct USB net label`
- `docs(repo): refresh release-readiness docs`

## Validation expectations

- Firmware: include `pio run` or equivalent build result and on-device notes when hardware was used
- ROS 2: include `colcon build` results for affected packages
- Hardware: include KiCad verification notes and regenerate tracked manufacturing outputs when the PCB changes

## Pull requests

Each PR should include:

- What changed
- How it was validated
- Hardware revision involved, if relevant
- Any follow-up work intentionally left for later beta releases

## CLI shortcuts

```bash
git switch develop
git switch -c sw/example-change
git status
git add <files>
git commit -m "feat(scope): summary"
git push -u origin sw/example-change
gh pr create --base develop --fill
```
