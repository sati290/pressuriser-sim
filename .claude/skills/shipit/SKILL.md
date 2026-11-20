---
name: shipit
description: Ship the current branch to main. The owner invoking /shipit approves the change; open a pull request, run the project's checks, then merge it into main. Use only when the owner types /shipit.
---

# /shipit: approve, open a PR, check, merge into main

The owner typing `/shipit` is their approval of the work on the current branch and their explicit
instruction to open a pull request **and merge it into `main`**. Do not ask for confirmation again
unless a step below says to stop.

The repository has no CI: the checks are the local ones in `CLAUDE.md`. They must pass on the exact
commit that gets merged.

## Steps

1. **Branch state**
   - Refuse on `main` itself: say there is nothing to ship from `main` and stop.
   - Commit any uncommitted changes that belong to the work (never `.env`, `shots/`, `node_modules/`),
     with a normal descriptive message, then push the branch (`git push -u origin <branch>`).
   - `git fetch origin main`. If the branch is behind `main`, merge `origin/main` into it
     (no rebase, no force-push). On conflicts, stop and tell the owner.
   - If the branch has no commits that `main` lacks, say so and stop.

2. **Checks** (on the commit you will merge, after any merge from `main`)
   - `npm test`: must pass. If it fails, stop and report the failing tests; do not merge.
   - If `index.html` changed relative to `origin/main` (CSS, markup or drawing), run the screenshot
     tool: `node tools/screenshot.mjs` (in a cloud session with the pre-installed browser:
     `node tools/screenshot.mjs --chrome /opt/pw-browsers/chromium`). Every line must show
     `errors=none` and no horizontal overflow (scrollWidth equal to the viewport width). Look at the
     wide screenshot(s) for anything obviously broken. If something fails, stop and report.
   - If `common.js`, `sim.js` or `control.js` changed, check that `docs/physics.md`, `CLAUDE.md` and the
     notes section in `index.html` still describe the model (CLAUDE.md asks for these to stay in sync).
     Fix small omissions in a commit on the branch and re-run the checks; flag anything bigger in
     the PR description instead of blocking.
   - Push again if anything was committed in this step.

3. **Pull request** (GitHub MCP tools; load them with ToolSearch if they are deferred)
   - Reuse an open PR from this branch into `main` if one exists; otherwise create one.
   - Title: a short summary of the change. Body: what changed and why (from the commits since
     `main`), the checks run with their results (test count, screenshot widths/themes), anything
     flagged in step 2, and the preview link
     `https://raw.githack.com/sati290/pressuriser-sim/<head-sha>/index.html`.
     Mirror a PR template if the repository has one. End with the PR attribution lines from the
     session's system reminder.

4. **Merge**
   - Make sure the PR head is the commit you checked; if it moved, go back to step 2.
   - Merge with a merge commit (method `merge`, as earlier PRs in this repo were merged).
   - If GitHub refuses (conflicts, branch protection, permissions), stop and report the reason;
     never force-push to `main` or bypass protection.

5. **Report**
   - The PR link, the merge commit SHA, the checks that ran, and the live link
     `https://raw.githack.com/sati290/pressuriser-sim/main/index.html` (raw.githack may take a few
     minutes to pick up the new `main`).
   - After merging, the working branch's PR is merged: further work on that branch name restarts
     from the new `main` (see the session's git instructions).
