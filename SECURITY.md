# Security policy

## Supported versions

Only the latest released minor version of ferroute receives security fixes.

## Reporting a vulnerability

Please do **not** open a public issue. Report privately through GitHub's
["Report a vulnerability"](https://github.com/vanhanit/ferroute/security/advisories/new) form
on the repository's Security tab.

Include the affected version, a description, and a reproducer (a minimal `.kicad_pcb` if relevant).
You can expect an acknowledgement within 7 days and a fix or mitigation plan within 30 days.

Areas of particular interest: parsers for KiCad files (`ferroute-kicad`), the IPC layer
(`ferroute-ipc`), and anything that writes output files.
