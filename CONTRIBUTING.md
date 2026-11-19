# Contributing to ferroute

Thanks for helping! ferroute is licensed under AGPL-3.0-only, with commercial licenses available
(see [COMMERCIAL.md](COMMERCIAL.md)).

## Licensing of contributions (lightweight CLA)

By submitting a contribution you agree that (1) it is licensed under AGPL-3.0-only, and
(2) you grant the copyright holder the right to relicense your contribution under other
terms, including commercial licenses as described in [COMMERCIAL.md](COMMERCIAL.md). You confirm
you have the right to make the contribution. No DCO sign-off line is required.

## Workflow

Host requirements: `make` and `docker` only.

```sh
make dev-image    # build the toolchain image once
make lint         # rustfmt check + clippy -D warnings
make test         # nextest + doctests
make e2e          # route testdata boards, run ferroute DRC on the output
make kicad10-drc  # independent KiCad 10 DRC of routed boards
make audit        # cargo-deny + cargo-audit
make hooks        # install the commit-msg hook
```

`make NATIVE=1 <target>` runs on the host toolchain instead of Docker.

## Quality gates

A PR must pass: formatting, clippy with no warnings, tests on Linux/macOS/Windows, cargo-deny
(licenses must be AGPL-compatible), the DRC regression suite (routed boards must have zero DRC
violations and no routing invariant regressions against `bench/baseline.json`), and the KiCad 10
`kicad-cli` DRC lane. The KiCad 11 preview lane is informational.

## Commit messages and releases

We use [Conventional Commits](https://www.conventionalcommits.org/): `type(scope): subject` with
types `feat fix perf refactor test docs ci build chore revert style`. Scopes are free-form.
PR titles and every commit in a PR are linted. Use `!` or a `BREAKING CHANGE:` footer for breaking changes.

Releases are automatic on merge to `main`: `feat` bumps minor, `fix`/`perf` bump patch, breaking
changes bump minor while the version is < 1.0 (major afterwards). `docs`, `test`, `chore`, `ci`
etc. alone do not trigger a release.
