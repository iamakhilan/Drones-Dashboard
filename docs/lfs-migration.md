# Git LFS migration plan for AeroVIO

The dashboard bundles sizeable binary/data assets (trajectory CSVs, the PCD
map, PNG imagery). `.gitattributes` carries the matching LFS patterns but they
are **commented out** — enabling LFS on files already committed requires a
history rewrite. Follow these steps on a maintenance branch, with all
contributors aware.

## 1. Install & initialize

```bash
git lfs install
```

## 2. Rewrite history for existing blobs

```bash
git lfs migrate import --include="data/*.csv,data/*.pcd,*.png" --everything
```

This replaces matching blobs in every commit with LFS pointers and rewrites
all commit hashes. Inspect the result:

```bash
git lfs ls-files --all
```

## 3. Uncomment the `.gitattributes` patterns

Enable (uncomment) these lines in `.gitattributes`:

```
data/*.csv filter=lfs diff=lfs merge=lfs -text
data/*.pcd filter=lfs diff=lfs merge=lfs -text
*.png filter=lfs diff=lfs merge=lfs -text
```

## 4. Push (force required after migrate)

```bash
git push --force-with-lease origin main
git push --force-with-lease --tags
```

## 5. Notify collaborators

Every clone must re-clone or run:

```bash
git fetch --all
git reset --hard origin/main
git lfs pull
```

## Notes

- GitHub enforces a hard 100 MB file limit and LFS bandwidth quotas — check
  repository Settings → Billing before large migrations.
- `real_data.js` and `docs/` assets are small text and should **not** be moved
  to LFS (dataset values are consumed as text by the dashboard).
