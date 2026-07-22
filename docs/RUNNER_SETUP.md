# Self-hosted Windows runner setup

The `.pyd` build needs MSVC 2022 and the FTDI dependencies, so it runs on a
self-hosted Windows machine rather than a GitHub-hosted runner.

> **If you already run the cl384_python build on this machine**, most of this is
> done. You need only: add `ER4COMMLIB_PATH` as a *system* variable (§2),
> restart the runner service (§4), and make the runner visible to the er4-python
> repo (§3, the org-level note).

---

## 1. Machine prerequisites

- Windows 10+, 64-bit.
- Visual Studio 2022 with the "Desktop development with C++" workload (MSVC v143).
- CMake ≥ 3.21 on `PATH`.
- Python 3.11.7, 64-bit, with headers and `libs\python311.lib`.
- pybind11 headers.
- er4CommLib's dependencies: ftdi_utils, libMPSSE, FTD2XX.
- Git.

---

## 2. Environment variables

| Variable | Points at | Notes |
|---|---|---|
| `ER4COMMLIB_PATH` | an installed er4CommLib prefix | must contain `cmake\er4commlibConfig.cmake`; typically `C:\ElemLibraries\er4CommLib`. **Overridden per-run by CI.** |
| `PYTHON_3_11_7_PATH` | Python 3.11.7 root | needs `include\` and `libs\python311.lib` |
| `PYBIND_11_PATH` | pybind11 root | headers only |
| `FTDI_UTILS_PATH` | ftdi_utils install | |
| `LIBMPSSE_PATH` | libMPSSE root | source of `MPSSE.dll` |
| `FTD2XX_PATH` | ftd2xx root | source of `FTD2XX.dll` |

Match the trailing-slash convention er4CommLib's own CMakeLists expects — set
them the same way you set them for a working local build. If a local build
works, copying those exact values is correct.

`ER4COMMLIB_PATH` is only used by CI as a *fallback shape*; the workflow
overrides it to a per-run prefix. You still set it here so local/manual builds
on this machine work.

### ⚠️ Critical: make them SYSTEM variables

The runner runs as a **Windows service** under its own account. Services do
**not** see per-user environment variables. If you set these only in your own
user profile, CI fails with "cannot find ftdi_utils / python311.lib" **even
though building by hand on the same machine works**. This is the single most
common self-hosted gotcha.

1. Start → "Edit the system environment variables" → Environment Variables.
2. Add each under **System variables** (the lower box), not User variables.
3. OK.
4. **Restart the runner service** (§4) so it picks them up.

`docs/PREFLIGHT.md` §1 has a one-liner that reports machine-vs-user for every
variable at once. Run it.

---

## 3. Register the runner

1. GitHub → **er4-python** → Settings → Actions → Runners → New self-hosted
   runner → Windows.
2. Follow the shown commands on the build machine:
   ```powershell
   mkdir C:\actions-runner ; cd C:\actions-runner
   # (download command shown by GitHub)
   .\config.cmd --url https://github.com/Elements-SRL/er4-python --token <TOKEN>
   ```
3. When `config.cmd` asks for **labels**, add: `windows,e384-build`
   (`self-hosted` is added automatically). The workflow targets
   `[self-hosted, windows, e384-build]`, so these must match.

### Sharing one runner between cl384_python and er4-python

A repo-level runner serves exactly one repository. If your existing runner is
registered to cl384_python, er4-python's jobs will queue forever.

The clean fix is an **organisation-level runner**: Org → Settings → Actions →
Runners → New runner, then grant both repositories access via a runner group.
One machine, both pipelines, one set of labels.

The `e384-build` label is kept as-is even though this is the er4 pipeline —
renaming it would mean re-labelling a runner that cl384_python also targets.
It identifies the machine, not the project.

---

## 4. Run it as a service

```powershell
cd C:\actions-runner
.\svc.cmd install
.\svc.cmd start
```

Useful later:
```powershell
.\svc.cmd status
.\svc.cmd stop
.\svc.cmd start     # run this after changing system env vars
```

After any change to system environment variables, **stop and start the service**
so it re-reads the environment.

---

## 5. Security note

If er4-python is **public**, do not let the build workflow run on pull requests
from forks — a fork's PR could run arbitrary code on your machine. The provided
workflow triggers only on `repository_dispatch` and `workflow_dispatch`, neither
of which a fork can invoke, so you are covered as written. If you later add a
`pull_request` trigger, restrict it or move that workflow to a hosted runner.
See `SECURITY.md`.

---

## 6. Quick validation

Work through `PREFLIGHT.md` — it is exactly this, as runnable checks.
