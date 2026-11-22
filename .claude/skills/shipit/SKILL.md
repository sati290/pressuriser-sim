---
name: shipit
description: Merge this branch into main: opens a pull request, checks it and merges it. Typing /shipit is your approval.
disable-model-invocation: true
---

# /shipit

The owner typing `/shipit` approves the branch and instructs you to open a PR **and merge it into
`main`**. Don't ask again unless a step says to stop. There is no CI: the checks are the local ones
in `CLAUDE.md`.

1. **Branch**
   - On `main`: nothing to ship, stop.
   - Commit uncommitted work (never `.env`, `shots/`, `node_modules/`) and push.
   - `git fetch origin main`; if behind, merge `origin/main` (no rebase, no force-push). Conflicts: stop.
   - No commits that `main` lacks: stop.

2. **Checks**, on the commit to be merged. Skip a check you already ran and that passed in this
   session if none of the files it covers have changed since (a merge from `main` counts as a change).
   - `npm test` must pass.
   - If `index.html` changed vs `origin/main`: `node tools/screenshot.mjs` (cloud:
     `--chrome /opt/pw-browsers/chromium`); every line `errors=none`, scrollWidth = viewport width;
     glance at the wide shots.
   - If `common.js`, `sim.js` or `control.js` changed: `docs/physics.md`, `CLAUDE.md` and the page's
     notes must still match. Fix small gaps on the branch; flag bigger ones in the PR.
   - Any failure: stop and report. Push anything committed here.

3. **PR** (GitHub MCP tools): reuse an open PR from this branch, else create one. Body: what changed
   and why, checks and results (or that they were already run on this code), anything flagged, the
   preview `https://raw.githack.com/sati290/pressuriser-sim/<head-sha>/index.html`, and the PR
   attribution lines from the system reminder.

4. **Merge**: PR head must be the checked commit (if it moved, back to 2). Method `merge`. If GitHub
   refuses, stop and report; never force-push or bypass protection.

5. **Report**: PR link, merge SHA, checks, and
   `https://raw.githack.com/sati290/pressuriser-sim/main/index.html` (may take a few minutes).
