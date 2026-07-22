# Preflight — run these before the first CI build

Every check below has failed for somebody at least once. Each takes seconds.
Do them in order; stop at the first failure and fix it before continuing.

Checks 1–4 run **on the self-hosted runner machine**. Checks 5–7 are on GitHub.

---

## 1. The env vars exist AT SYSTEM LEVEL

This is the single most common self-hosted failure: the build works when you run
it by hand, and fails in CI, because the runner service runs under its own
account and cannot see *user* environment variables.

In PowerShell on the runner:

```powershell
foreach ($v in 'ER4COMMLIB_PATH','PYTHON_3_11_7_PATH','PYBIND_11_PATH',
               'FTDI_UTILS_PATH','LIBMPSSE_PATH','FTD2XX_PATH') {
  $m = [Environment]::GetEnvironmentVariable($v,'Machine')
  $u = [Environment]::GetEnvironmentVariable($v,'User')
  '{0,-22} machine={1,-6} user={2}' -f $v, ($(if($m){'SET'}else{'--'})), ($(if($u){'SET'}else{'--'}))
}
```

**Every one must show `machine=SET`.** A var that is `user=SET` but
`machine=--` is exactly the trap. Fix via Start → "Edit the system environment
variables" → Environment Variables → **System variables** (the lower box).

`ER4COMMLIB_PATH` is new for this pipeline — the others already exist from the
e384 setup. Set it to your machine-wide install, e.g.
`C:\ElemLibraries\er4CommLib`. CI overrides it per-run; this value is for local
builds.

**After changing any system var, restart the runner service:**

```powershell
cd C:\actions-runner
.\svc.cmd stop
.\svc.cmd start
```

---

## 2. The config package is where CMake will look for it

```powershell
Test-Path "$env:ER4COMMLIB_PATH\cmake\er4commlibConfig.cmake"
```

Must print `True`. If your install put it somewhere else (e.g. directly in the
prefix root rather than a `cmake` subfolder), the "Build + install commlib" step
in the workflow will fail its verification with a listing of where the file
actually landed — adjust the `$cfg` path there to match.

---

## 3. Python is 64-bit and is 3.11.7

```powershell
& "$env:PYTHON_3_11_7_PATH\python.exe" -c "import sys; print(sys.version); print(sys.maxsize > 2**32)"
```

Must print a `3.11.7` version and `True` (True = 64-bit). A 32-bit Python or a
different minor version produces a `.pyd` that will not import.

Also confirm the link library exists:

```powershell
Test-Path "$env:PYTHON_3_11_7_PATH\libs\python311.lib"
```

---

## 4. The shippable DLLs are findable

The workflow searches `FTDI_UTILS_PATH`, `LIBMPSSE_PATH`, `FTD2XX_PATH`, and the
commlib install `bin` for each DLL. Confirm both exist somewhere in that set:

```powershell
foreach ($dll in 'MPSSE.dll','FTD2XX.dll') {
  $hit = @($env:FTDI_UTILS_PATH,$env:LIBMPSSE_PATH,$env:FTD2XX_PATH) |
    Where-Object { $_ } |
    ForEach-Object { Get-ChildItem $_ -Recurse -Filter $dll -ErrorAction SilentlyContinue } |
    Select-Object -First 1
  if ($hit) { "$dll -> $($hit.FullName)" } else { "$dll -> NOT FOUND" }
}
```

If either says NOT FOUND, add the containing folder to `DLL_SEARCH` in
`build-and-release.yml`.

---

## 5. The runner is online and correctly labelled

GitHub → **er4-python** → Settings → Actions → Runners.

The runner must show **Idle** (green) and carry all three labels:
`self-hosted`, `windows`, `e384-build`.

> The same physical runner serves both cl384_python and er4-python. A runner is
> registered per-repository by default, so if it is currently registered only to
> cl384_python it will **not** pick up er4-python jobs. Either register it to
> er4-python as well, or (better) register it at the **organisation** level and
> grant both repos access. If jobs sit forever in "Queued", this is why.

---

## 6. Both secrets exist, on the right repos

| Secret | Lives on | Scope needed |
|---|---|---|
| `COMMLIB_PAT` | **er4-python** | Contents: **Read** on `Elements-SRL/er4CommLib` |
| `DISPATCH_TOKEN` | **er4CommLib** | Contents: **Write** on `Elements-SRL/er4-python` |

Two distinct tokens, on two different repos, pointing opposite directions. Do
not reuse the cl384 tokens — those are scoped to cl384_python and will 403.

The built-in `GITHUB_TOKEN` cannot do either job: it cannot read a different
repo, and it cannot trigger a workflow in a different repo.

---

## 7. Repo names in the workflows are right

- `build-and-release.yml` → `COMMLIB_REPO: Elements-SRL/er4CommLib`
- `trigger-wrapper-build.yml` → the dispatch URL ends `/Elements-SRL/er4-python/dispatches`
- `trigger-wrapper-build.yml` → `AUTHORISED_ACTOR: CHANGE_ME_github_username`

That last one is a deliberate placeholder. If you leave it, every tag push fails
the actor-check — loudly, which is the safe direction, but nothing will build.

---

## First run: use workflow_dispatch, not a tag

Do **not** cut a real tag for the first run. Instead:

er4-python → Actions → **Build and release .pyd** → Run workflow → commlib ref
`development` → Run.

This exercises the whole build path without involving er4CommLib and without a
tag in the wild. It will still publish a release (named after whatever version
`development`'s CMakeLists declares) — delete that release afterwards if you
don't want it.

If it goes green and the zip contains a `.pyd` that imports, the tag path will
work too: the only difference is where `commlib_ref` comes from.
