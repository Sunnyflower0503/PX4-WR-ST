# Firmware Repository Setup

This repository uses Git submodules. Clone and update it with the pinned
submodule commits so that the firmware source matches the validated revision.

## First clone

```bash
git clone --branch STHITL --recurse-submodules https://github.com/Sunnyflower0503/PX4-WR-ST.git
cd PX4-WR-ST
git submodule status --recursive
```

If the repository was cloned without submodules:

```bash
git submodule sync --recursive
git submodule update --init --recursive
```

## Update an existing clone

```bash
git switch STHITL
git pull --ff-only --recurse-submodules
git submodule sync --recursive
git submodule update --init --recursive
```

Submodules are intentionally pinned by commit. Do not run
`git submodule update --remote` for a reproducible firmware build, because that
selects newer upstream revisions instead of the revisions recorded here.

## Verify the checkout

The following command must finish without a missing mapping or missing commit
error:

```bash
git submodule status --recursive
```

A leading space means the submodule is at the recorded commit. A leading `-`
means it has not been initialized, `+` means a different commit is checked out,
and `U` means a merge conflict must be resolved.

The repository only records committed submodule content. Local edits made
inside a submodule are not included when the parent repository is pushed; they
must be published in an accessible submodule repository and then referenced by
an updated gitlink in this repository.
