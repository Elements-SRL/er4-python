# Build Authorisation & Security

This pipeline can publish releases automatically, so it matters *who* is allowed
to set it off. This document explains the implemented gate (actor-check), its
honest limits, and the stronger gate you can add later.

---

## 1. What is implemented: the actor-check

In `er4CommLib/.github/workflows/trigger-wrapper-build.yml`, the first step
compares the user who pushed the tag against an allow-listed username:

```yaml
- name: Verify authorised actor
  env:
    AUTHORISED_ACTOR: CHANGE_ME_github_username
  run: |
    if [ "$GITHUB_ACTOR" != "$AUTHORISED_ACTOR" ]; then
      echo "::error::User '$GITHUB_ACTOR' is not authorised..."
      exit 1
    fi
```

If anyone else pushes a tag, the job fails here: no dispatch, no build, no
release.

For more than one user, switch to a membership test:

```bash
case " alice bob carol " in
  *" $GITHUB_ACTOR "*) echo "authorised" ;;
  *) echo "::error::not authorised"; exit 1 ;;
esac
```

---

## 2. Honest limits of the actor-check

`github.actor` tells you which account performed the push. That is an
**authentication** signal, not a tamper-proof **authorisation** control.

Good for:
- Stopping the wrong teammate from accidentally cutting a release.
- Zero friction — fully automatic, no clicks.

Does **not**:
- Defend against a **compromised account**. With the authorised user's
  credentials, `github.actor` reads as that user.
- Provide any cryptographic guarantee of intent.

For an internal library workflow this is usually the right amount of control.

---

## 3. Stronger gate (not implemented): required reviewers

GitHub **Environments** let you require a named human to approve a job before it
runs — real authorisation, a pause and a click, regardless of who triggered it.

1. **er4-python** → Settings → **Environments** → New environment, e.g. `release`.
2. Enable **Required reviewers**, add the approver(s).
3. In `build-and-release.yml`, attach the job:

   ```yaml
   jobs:
     build:
       runs-on: [self-hosted, windows, e384-build]
       environment: release        # <-- add this line
   ```

The two gates compose: actor-check filters who can *trigger*, the environment
reviewer controls who can *approve*.

### Trade-off
One manual click per release. That is the point — it converts an automatic
pipeline into a gated one. Use it when an unattended release would be
unacceptable.

---

## 4. Self-hosted runner exposure

Whoever can make the runner execute a workflow can run code on that machine.

- The provided workflow triggers only on `repository_dispatch` and
  `workflow_dispatch`. Neither can be invoked by a fork's pull request, so a
  public fork cannot execute code on your runner **as written**.
- If you later add a `pull_request` trigger to any workflow on this runner,
  restrict it (same-repo branches only, or require approval for first-time
  contributors), or move it to a hosted runner.
- **This machine may also serve cl384_python.** A weakness introduced in either
  repo's workflows exposes the same physical machine and the same vendor SDKs.
  Review both when changing triggers.

---

## 5. Token hygiene

| Token | Lives on | Scope |
|---|---|---|
| `COMMLIB_PAT` | er4-python | Contents:**read** on er4CommLib only |
| `DISPATCH_TOKEN` | er4CommLib | Contents:**write** on er4-python only |

Keep both fine-grained and minimally scoped. Rotate if either may have leaked.
Do not reuse the cl384 pipeline's tokens — separate scopes mean a leak of one
does not compromise the other.
