# CONTRIBUTING — Core System 2026  
_Effective Git + GitHub CLI + Gitk workflow_


main        ← **stable releases** (protected, tagged)
develop     ← **integration** (CI + 1 review)
hw/\*        ← hardware draft branches
sw/\*        ← software draft branches
misc/\*      ← docs / CI / repo chores

````

---

## 1 · Observe before you change 📋

```bash
git status                               # staged / dirty files
git log --graph --oneline --decorate -6  # recent history
gitk --all &                             # full GUI graph
````

---

## 2 · Real example — add `led_blink.cpp` to **Software/rover-coreio**

### 2-A  Create a software feature branch

```bash
git checkout develop
git pull
git switch -c sw/led-blink-demo
```

### 2-B  Code + local test

```bash
# add the example
nano Software/rover-coreio/examples/led_blink.cpp

idf.py build                    # compile check
idf.py -p /dev/ttyUSB0 flash    # run on board
```

### 2-C  Stage & commit (Conventional Commit)

```bash
git add Software/rover-coreio/examples/led_blink.cpp
git commit -m "feat(sw-examples): add LED blink demo to rover-coreio"
```

### 2-D  Visual sanity-check

```bash
git log --graph --oneline --decorate --all -4
```

### 2-E  Push & open PR with **GitHub CLI**

```bash
git push -u origin sw/led-blink-demo
gh pr create --base develop --fill        # uses commit msg for title/body
```

---

## 3 · Review loop

```bash
gh pr diff               # view patch
gh pr checkout <#>       # build branch locally
idf.py build             # run tests
gh pr review <#> --approve -b "Works on DevKit 👍"
```

Authors amend fixes:

```bash
git commit --amend
git push --force-with-lease
```

---

## 4 · Integrate: squash-merge → **develop**

```bash
gh pr merge <#> --squash --delete-branch
```

CI re-runs on **develop**; field-test on rover if needed.

---

## 5 · Baseline: fast-forward **develop → main** + tag

```bash
git checkout main
git merge --ff-only develop
git push origin main

git tag -a v0.2.0 -m "Add LED blink demo example"
git push origin v0.2.0
```

---

## 6 · Quick command reference

| Task                | Command                                            |
| ------------------- | -------------------------------------------------- |
| Stage / dirty files | `git status`                                       |
| Compact graph       | `git log --graph --oneline --decorate --all`       |
| GUI graph           | `gitk --all &`                                     |
| New branch          | `git switch -c sw/<feature>`                       |
| Diff vs develop     | `git diff develop...HEAD`                          |
| Push & track        | `git push -u origin <branch>`                      |
| Open PR             | `gh pr create --base develop --fill`               |
| List PRs            | `gh pr list`                                       |
| Approve             | `gh pr review <#> --approve`                       |
| Squash-merge        | `gh pr merge <#> --squash --delete-branch`         |
| Fast-forward main   | `git checkout main && git merge --ff-only develop` |

---

## 7 · Commit type cheat-sheet

| type    | purpose          | example                               |
| ------- | ---------------- | ------------------------------------- |
| `feat`  | new capability   | `feat(sw-can): add FDCAN filter`      |
| `fix`   | bug / regression | `fix(hw-mcu): correct USB D+ routing` |
| `docs`  | docs only        | `docs(imu): add calibration guide`    |
| `ci`    | CI / docker      | `ci(build): enable kicad-cli drc`     |
| `chore` | house-keeping    | `chore: bump ESP-IDF 5.3`             |
| `hw`    | mech / BOM only  | `hw(bom): replace 10 µF cap`          |

Stay on-topic per branch, keep commits minimal, and the history stays easy to read—and bisect.

---

