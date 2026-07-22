# CI/CD Pipeline Setup — er4 Python wrapper

This document explains how the automated `.pyd` build works and how to set it up
from scratch. Companion documents go deeper on specific parts:

- `PREFLIGHT.md` — **run this before the first build.** Verifies the runner.
- `RUNNER_SETUP.md` — installing and configuring the self-hosted Windows runner.
- `LOCAL_BUILD.md` — building the `.pyd` by hand, without CI (the ground truth).
- `SECURITY.md` — how the build is gated, and how to strengthen the gate.
- `ADDING_PYTHON_VERSIONS.md` — supporting more than Python 3.11.

This pipeline is a sibling of the one in `cl384_python`; the structure is
deliberately the same so that fixing one teaches you how to fix the other.

---

## 1. What this pipeline does

When an authorised user pushes a version tag to **er4CommLib**, the pipeline:

1. Verifies the tag was pushed by an approved user (actor-check).
2. Tells the **er4-python** repo to start a build (cross-repo dispatch).
3. On a self-hosted Windows machine: checks out er4CommLib **at that tag** and
   builds it **from source**, then installs it to a per-run prefix.
4. Builds the pybind11 wrapper against that freshly-built commlib, producing
   `er4_python_wrapper.pyd`.
5. Bundles the `.pyd` with its runtime DLLs (`MPSSE.dll`, `FTD2XX.dll`) and
   smoke-tests that Python can import it.
6. Publishes a GitHub Release whose name and notes state exactly which
   er4CommLib version produced it.

Unlike the cl384 pipeline there is **no Opal Kelly / FrontPanel dependency**, so
the bundle is self-contained: nothing has to be obtained separately by the user.

---

## 2. The repositories

| Repo | Role |
|---|---|
| **er4CommLib** | The C++ library. Tagging it triggers everything. |
| **er4-python** | Merged wrapper-builder + release host. Builds and publishes the `.pyd`. |

> **Merge note:** `er4-python-creator` (the builder) and `er4-python` (the
> user-facing repo) merge into a single repo, `er4-python`. The builder's
> `main.cpp` + `CMakeLists.txt` live at the repo root; releases are published
> here. See section 7 if you have not merged yet.

---

## 3. Architecture at a glance

```
   developer pushes tag  0.18.0
            |
            v
  ┌────────────────────────┐
  │  er4CommLib repo        │
  │  trigger-wrapper-build  │   actor-check: is pusher authorised?
  │  (GitHub-hosted)        │   if yes -> repository_dispatch
  └──────────┬──────────────┘
             │  event: commlib-tagged  { commlib_ref: 0.18.0 }
             v
  ┌────────────────────────────────────────────┐
  │  er4-python repo                            │
  │                                             │
  │  job: build   (SELF-HOSTED Windows)         │
  │    1. checkout commlib @ 0.18.0             │
  │    2. cmake build+install commlib (per-run) │
  │    3. cmake build wrapper  -> .pyd          │
  │    4. bundle .pyd + MPSSE.dll + FTD2XX.dll  │
  │    5. smoke test: python -c "import ..."    │
  │    6. upload zip as workflow artifact       │
  │                                             │
  │  job: publish (hosted Linux, needs: build)  │
  │    7. download every artifact               │
  │    8. ONE release "commlib 0.18.0"          │
  └────────────────────────────────────────────┘
```

The build/publish split is what makes adding Python 3.12 later a one-line
change: each matrix leg uploads its own zip, and publish attaches them all to a
single release.

---

## 4. One-time setup checklist

### 4.1 Set up the self-hosted runner
Follow `RUNNER_SETUP.md`. If the runner is already serving cl384_python, you
mostly need to (a) add `ER4COMMLIB_PATH` as a **system** variable and (b) make
sure the runner is visible to the er4-python repo — see PREFLIGHT §5.

### 4.2 Create the two tokens

**`COMMLIB_PAT`** — lets the build read er4CommLib's source.
1. GitHub → Settings → Developer settings → Fine-grained personal access tokens.
2. Repository access: only `er4CommLib`. Permissions: **Contents → Read**.
3. Store on **er4-python** → Settings → Secrets and variables → Actions, named
   `COMMLIB_PAT`.

**`DISPATCH_TOKEN`** — lets er4CommLib start er4-python's build.
1. New fine-grained token. Repository access: only `er4-python`.
2. Permissions: **Contents → Read and write**.
3. Store on **er4CommLib**, named `DISPATCH_TOKEN`.

> Why PATs and not the built-in `GITHUB_TOKEN`? It cannot read a different repo,
> and it cannot trigger workflows in a different repo. Both directions here are
> cross-repo.

> Do not reuse the cl384 tokens. They are scoped to cl384_python and will 403.

### 4.3 Drop in the workflow files
- `build-and-release.yml` → **er4-python** at `.github/workflows/`.
- `trigger-wrapper-build.yml` → **er4CommLib** at `.github/workflows/`.
  (It ships in this repo under `trigger/` purely for delivery; it does nothing
  here.)

Then set `AUTHORISED_ACTOR` in the trigger workflow — it is a deliberate
`CHANGE_ME_github_username` placeholder.

### 4.4 The tag scheme
The trigger fires on tags matching `[0-9]*.[0-9]*.[0-9]*` — i.e. `0.18.0`, no
`v` prefix. The build reads the real version from commlib's `CMakeLists.txt`
regardless, so the tag text only needs to *trigger*, not to carry the version.

**The branch is irrelevant.** A tag points at a commit; checkout resolves the
tag directly. If you tag a commit whose tree has no `CMakeLists.txt`, the build
fails loudly at the verify step rather than silently building something else.
Tag commits on `development` (or wherever the buildable tree lives).

---

## 5. Running it

### First run: test, don't tag
See PREFLIGHT's closing section. Use `workflow_dispatch` with ref `development`.

### Real run
1. The authorised user pushes a tag on er4CommLib:
   ```
   git tag 0.18.0
   git push origin 0.18.0
   ```
2. The trigger workflow runs, passes the actor-check, dispatches.
3. er4-python's build runs on the self-hosted runner.
4. A release named **er4_python (commlib 0.18.0)** appears on er4-python with
   the zipped bundle attached.

If the pusher is not the authorised user, the trigger job fails at the
actor-check and nothing is built.

---

## 6. What a consumer downloads

A zip containing:
- `er4_python_wrapper.pyd` — the module (`import er4_python_wrapper`).
- `MPSSE.dll`, `FTD2XX.dll` — FTDI runtime, must sit beside the `.pyd`.
- `BUILD_MANIFEST.txt` — records the commlib version and Python target.

They also need the FTDI D2XX USB driver installed on the machine — that is a
driver, not a redistributable DLL, and is covered in the README.

---

## 7. Merging the two old repos

`er4-python` is the repo that survives (it holds the user-facing README, the
examples, the docs). `er4-python-creator` folds into it.

```bash
# in a clone of er4-python
git remote add creator <url-of-er4-python-creator>
git fetch creator
git merge creator/main --allow-unrelated-histories
# resolve path clashes, commit
```

Expect one clash: **both repos have a `Readme.md`**. Keep er4-python's (the
substantial user-facing one) and discard the creator's three-line version; its
content is superseded by these docs.

The builder files (`main.cpp`, `CMakeLists.txt`, `include/`) end up at the repo
root. After merging there is exactly one cross-repo link left:
er4CommLib → er4-python.

You can delete `cl4_python.pro` at this point — the qmake build is superseded by
CMake, and leaving two build systems around invites someone to fix the wrong one.

---

## 8. Troubleshooting quick table

| Symptom | Likely cause | Fix |
|---|---|---|
| Trigger job fails at actor-check | pusher != authorised user | expected; or you left the `CHANGE_ME` placeholder |
| Dispatch step 403 / 404 | `DISPATCH_TOKEN` missing/wrong scope | recreate PAT with Contents:write on er4-python |
| Build never starts, job stays "Queued" | runner not registered to *this* repo, or offline, or label mismatch | PREFLIGHT §5 |
| Checkout of commlib fails 404 | `COMMLIB_PAT` missing/wrong scope | needs Contents:read on er4CommLib |
| CMake can't find ftdi_utils / MPSSE / FTD2XX | env vars not visible to the service | make them **system** vars, restart runner service |
| `er4commlibConfig.cmake not at ...` | commlib installed to a different layout | read the listing the step prints, adjust `$cfg` |
| "No .pyd produced" | wrapper build failed upstream | read the Build wrapper step log |
| Required DLL not found | DLL not in the searched folders | adjust `DLL_SEARCH` in the workflow |
| Smoke test fails, `DLL load failed` | companion DLL missing, or wrong Python bitness/minor | PREFLIGHT §3 and §4 |
| Release published but empty | artifact upload/download name mismatch | check the `bundle-*` pattern |
