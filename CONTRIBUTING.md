# CONTRIBUTING — Core_System_2026

main ← stable releases (protected)
develop ← integration (CI + 1 review)
hw/* ← hardware draft branches
sw/* ← software draft branches
misc/* ← docs / CI / repo chores

---

## How to contribute

```bash
# 1. Keep local clone fresh
git checkout develop
git pull

# 2. Create a draft branch (pick the right prefix)
git switch -c hw/<feature-name>     # or sw/<feature>  misc/<feature>

# 3. Work → commit → push
git push -u origin <branch>

# 4. Open a Pull-Request into **develop**
#    CI must pass & at least one reviewer approves

