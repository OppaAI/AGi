# GitHub rulesets

`dev-protection.json` is a ready-to-import repository ruleset that protects the
`dev` branch against deletion, force-pushes, and direct pushes (changes must
come through a pull request).

## Import into a repository

1. On GitHub, open the repository → **Settings** → **Rules → Rulesets**
2. Click **New ruleset** → **Import a ruleset**
3. Upload `dev-protection.json`
4. Review the imported settings (enforcement: **Active**, target: `dev`) and click **Create**

Repeat for each repository that needs it (`OppaAI/AGi` and all `AGi-robot/*` repos).

## Note on the old ruleset

`OppaAI/AGi` has an existing ruleset named **"Cannot commit to main"** created
directly in GitHub settings. It is **disabled** and targets `main`, not `dev`
— so it currently protects nothing. You can either delete it or edit it to
target `refs/heads/dev` and set enforcement to Active. To reuse its settings in
other repos, open it in the GitHub UI, use the **⋯ → Export** option to get a
JSON file, then import that file as above.
