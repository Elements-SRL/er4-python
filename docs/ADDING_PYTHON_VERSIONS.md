# Supporting more Python versions

Today the wrapper targets **CPython 3.11** only. A `.pyd` is tied to one Python
minor version and one architecture, so "also support 3.12" means "produce a
second `.pyd` built against 3.12".

The **workflow is already structured for this**: build and publish are separate
jobs, the build job already has a `strategy.matrix`, and publish already
collects every artifact into one release. Adding a version is therefore mostly a
config change plus one source edit.

---

## 1. What still has to change

**The CMakeLists.** This is the one piece deliberately left pinned. It currently
hardcodes `PYTHON_3_11_7_PATH` and links `python311.lib` by name — because that
is the configuration proven to build and import. Making it version-agnostic is
required before a matrix can work, but it is untested here, so it was not done
pre-emptively.

Replace the Python block with:

```cmake
# --- Python -----------------------------------------------------------------
# The workflow selects which Python by putting the desired python.exe first on
# PATH; find_package picks it up from there.
find_package(Python COMPONENTS Development REQUIRED)
target_link_libraries(er4_python_wrapper PUBLIC Python::Python)
```

and delete both the `PYTHON_3_11_7_PATH_CMAKE` include-directory entry and the
final `target_link_libraries(... python311.lib)` line. Keep the pybind11
include directory.

**Verify this locally before touching CI.** Build once with the new CMakeLists
against 3.11 and confirm the `.pyd` still imports. If `find_package(Python)`
cannot locate your install, that is a runner-configuration problem to solve on
the bench, not in a workflow run.

---

## 2. Make each Python version available on the runner

Install every version you want (64-bit, with dev headers/libs). Then arrange for
the right one to be first on `PATH` per matrix leg:

- Install each under a known root and prepend its folder to `PATH` in a workflow
  step keyed off the matrix value; or
- Use `actions/setup-python` (works on self-hosted if the runner can reach the
  Python download hosts).

---

## 3. Extend the matrix

In `build-and-release.yml`, the matrix already exists:

```yaml
    strategy:
      fail-fast: false
      matrix:
        include:
          - python: '3.11'
            py_tag: 'cp311'
          - python: '3.12'          # <-- add
            py_tag: 'cp312'
```

Then add a step at the top of the job to select the interpreter:

```yaml
      - name: Select Python ${{ matrix.python }}
        run: |
          $root = "C:\python\${{ matrix.python }}"   # adjust to your layout
          echo "$root;$root\Scripts" | Out-File -FilePath $env:GITHUB_PATH -Append -Encoding utf8
```

And update the smoke-test step, which currently resolves the interpreter via
`PYTHON_3_11_7_PATH`, to just call `python` from `PATH`.

Everything else already keys off `matrix.py_tag`: the artifact name, the zip
name, and the manifest.

---

## 4. One release, all versions

Already handled. Each build leg uploads `bundle-cp311`, `bundle-cp312`, … and
the publish job downloads them all with `pattern: bundle-*` and
`merge-multiple: true`, attaching every zip to a single release.

One thing to check when you add a leg: the publish job reads
`needs.build.outputs.commlib_version`. With a matrix, a job output comes from
whichever leg finished last — harmless here, because every leg builds the same
commlib ref and therefore reports the same version. If that ever stops being
true, move the version extraction into its own single job.

---

## 5. What the user sees

Release assets become:

```
er4_python-commlib-0.18.0-cp311.zip
er4_python-commlib-0.18.0-cp312.zip
```

The README should then tell users to pick the zip matching their Python minor
version, and note that mixing them produces `DLL load failed`.
