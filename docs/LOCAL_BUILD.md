# Building the `.pyd` by hand (no CI)

This is the ground-truth procedure: how to build `er4_python_wrapper.pyd`
manually on a Windows machine. The CI pipeline does exactly these steps in an
automated way. If CI ever breaks, this is how you reproduce and diagnose it.

---

## 1. Requirements

- Windows 10+.
- Visual Studio 2022, "Desktop development with C++" workload (MSVC v143).
- CMake ≥ 3.21.
- Python **3.11.7** (64-bit) with headers and `libs/python311.lib`.
- pybind11 (headers only).
- er4CommLib's dependencies: `ftdi_utils`, and the FTDI runtime (`MPSSE`,
  `FTD2XX`).

The wrapper requires **C++20**. Despite the historical `cl4_python.pro`, Qt is
not used — plain CMake + MSVC is enough.

---

## 2. Environment variables

Set these (see `RUNNER_SETUP.md` for the full table). For a manual build in one
shell you can set them just for that session:

```powershell
$env:PYTHON_3_11_7_PATH = "C:\path\to\Python311"
$env:PYBIND_11_PATH     = "C:\path\to\pybind11"
$env:FTDI_UTILS_PATH    = "C:\path\to\ftdi_utils-install"
$env:LIBMPSSE_PATH      = "C:\path\to\libMPSSE"
$env:FTD2XX_PATH        = "C:\path\to\ftd2xx"
# ER4COMMLIB_PATH is set below, after we install commlib.
```

---

## 3. Build and install er4CommLib from source

The wrapper does `find_package(er4commlib REQUIRED CONFIG)`, so commlib must be
**installed** (not merely built) to a prefix that contains
`cmake/er4commlibConfig.cmake`.

```powershell
cd C:\src\er4CommLib          # a checkout at the desired tag
cmake -B build -G "Visual Studio 17 2022" -A x64 `
      -DCMAKE_BUILD_TYPE=Release `
      -DCMAKE_INSTALL_PREFIX="C:\src\er4commlib-install"
cmake --build build --config Release --target install
```

Then point the wrapper at what you just installed:

```powershell
$env:ER4COMMLIB_PATH = "C:\src\er4commlib-install"
```

> ⚠️ commlib's `CMakeLists.txt` forces the install prefix to `ER4COMMLIB_PATH`
> if you don't pass one — which is how installs end up in the machine-wide
> `C:\ElemLibraries\er4CommLib`. Passing an explicit `-DCMAKE_INSTALL_PREFIX`
> as above keeps things predictable. **Clear `ER4COMMLIB_PATH` before this step
> if it is already set**, or the explicit prefix may be overridden.
>
> For everyday local work, building against the machine-wide
> `C:\ElemLibraries\er4CommLib` is fine. For anything you intend to *ship*, use
> a per-build prefix so you know exactly which commlib you linked.

---

## 4. Build the wrapper → `.pyd`

```powershell
cd C:\src\er4-python           # the merged wrapper repo
cmake -B build -G "Visual Studio 17 2022" -A x64 -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release
```

The output is `er4_python_wrapper.pyd` under `build\` (typically
`build\Release\`). The module name comes from the
`PYBIND11_MODULE(er4_python_wrapper, m)` macro in `main.cpp`, so in Python you
`import er4_python_wrapper`.

---

## 5. Assemble a usable bundle

The `.pyd` alone is not runnable — it needs its companion DLLs beside it (or on
`PATH`). Put these in one folder:

- `er4_python_wrapper.pyd`
- `MPSSE.dll`
- `FTD2XX.dll`

Do **not** ship `main.o` — it is a compiler intermediate, not a runtime file.

You also need the FTDI D2XX **driver** installed on the machine (version
2.12.36.20 x64, from ftdichip.com or via EDR4). That is a system driver, not
something you bundle.

---

## 6. Smoke test

```powershell
cd <the bundle folder>
python -c "import er4_python_wrapper as m; print(m.__file__)"
```

If it imports without a `DLL load failed` error, the companion DLLs are in the
right place.

`DLL load failed while importing er4_python_wrapper` almost always means:
- a companion DLL is missing from the folder / PATH, or
- you are running a different-bitness Python (must be 64-bit), or
- a different Python minor version than the `.pyd` was built against (3.11).

CI runs this exact check before publishing, so a green build means the artifact
imported successfully on the runner.

---

## 7. Mapping to CI

| Manual step here | CI step in `build-and-release.yml` |
|---|---|
| checkout commlib at a tag | "Checkout er4CommLib source" |
| build+install commlib (§3) | "Configure commlib" + "Build + install commlib" |
| set `ER4COMMLIB_PATH` | done inline via `env:` on the wrapper steps |
| build wrapper (§4) | "Configure wrapper" + "Build wrapper" |
| assemble bundle (§5) | "Assemble bundle" |
| smoke test (§6) | "Smoke test import" |

If CI produces something your manual build doesn't (or vice versa), compare
step-by-step against this table — the difference is almost always an environment
variable the service can't see (system vs user), or a stale machine-wide commlib
being picked up locally that CI correctly ignores.
