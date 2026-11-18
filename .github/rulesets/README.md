# GitHub rulesets

`main-protection.json` is a ready-to-import repository ruleset for the
`AGi-robot/*` repositories. Once `main` is the default branch, it protects
`main` against deletion, force-pushes, and direct pushes — all changes must
come through a pull request (0 approvals required, so solo merges work).

> `OppaAI/AGi` is intentionally excluded for now while its contents are being
> stripped into the per-component org repos. The old, disabled
> "Cannot commit to main" ruleset there can be left as-is.

## Per-repository setup (web UI)

Do this for each `AGi-robot/*` repo: `shell`, `memory`, `cypher`, `telepathy`,
`canvas`, `imaging`, `scorecard`, `augmented-reality`.

1. **Bring `main` up to date:** open a PR with base `main`, compare `dev`,
   review, and merge. (`shell` had no `main`, so it was created from `dev`
   and needs no PR.) `scorecard`'s `main` has 1 commit not in `dev` —
   resolve that conflict in the PR if GitHub flags it.
2. **Make `main` the default:** **Settings → Branches → Default branch** (the
   pencil icon) → select `main` → **Update**.
3. **Protect `main`:** **Settings → Rules → Rulesets → New ruleset → Import a
   ruleset** → upload `main-protection.json` → review (Active, targets
   `refs/heads/main`) → **Create**.
4. Optionally delete any old disabled rulesets targeting `dev`.

After switching the default, existing local clones keep tracking `dev`; run
`git fetch && git checkout main` locally, and update any CI or docs that
assume `dev` is the default base.
