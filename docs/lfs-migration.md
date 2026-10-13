# Git LFS migration

The `.gitattributes` in this repo marks `data/*.csv`, `data/*.pcd`, and `*.png`
as LFS candidates (commented out). This document describes the one-time
migration to move those assets from regular Git objects into Git LFS.

> ⚠️ Migration rewrites history. Coordinate with all contributors, push a
> backup branch first, and have everyone re-clone afterwards.

## 1. Prerequisites

```bash
# Install git-lfs (once per machine)
# Debian/Ubuntu:  sudo apt install git-lfs
# macOS:          brew install git-lfs

git lfs install
```

## 2. Enable the patterns

Uncomment the LFS lines in `.gitattributes`:

```gitattributes
data/*.csv filter=lfs diff=lfs merge=lfs -text
data/*.pcd filter=lfs diff=lfs merge=lfs -text
*.png     filter=lfs diff=lfs merge=lfs -text
```

## 3. Migrate existing history

```bash
# Backup first
git branch backup/pre-lfs
git push origin backup/pre-lfs

# Rewrite history so the matching files become LFS pointers
git lfs migrate import --include="data/*.csv,data/*.pcd,*.png" --everything

# Verify the working tree still checks out real content
git lfs pull
git lfs ls-files          # should list the CSVs, PCD, and PNGs
git lfs fsck              # integrity check of local LFS objects
```

## 4. Force-push the rewritten branches

```bash
git push --force-with-lease origin main
# repeat --force-with-lease for any other long-lived branches/tags
```

## 5. Verify

```bash
# Pointer files, not blobs, should be in the tree
git show HEAD:data/synthetic_orbit_estimated_trajectory.csv | head -3
# Expected:
#   version https://git-lfs.github.com/spec/v1
#   oid sha256:...
#   size ...

git lfs env               # confirm endpoint and auth
```

Every contributor (and CI) must then run `git lfs install` and use
`git lfs pull` after cloning. If LFS is unavailable, the working files are
small pointer stubs — do not commit them back as-is.

## Rollback

If anything goes wrong before others have pulled:

```bash
git reset --hard backup/pre-lfs
git push --force-with-lease origin main
```
