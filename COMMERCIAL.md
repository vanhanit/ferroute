# Licensing: open source (AGPL-3.0) and commercial

ferroute is **dual-licensed**.

## 1. Open source — GNU AGPL-3.0-only

You may use, study, modify and redistribute ferroute free of charge under the
[GNU Affero General Public License v3.0](LICENSE). In short, if you distribute ferroute
or a modified version — **or let users interact with it over a network** (for example as
part of a web service, SaaS routing backend, CI-as-a-service, or a hosted EDA tool) — you
must:

* keep the copyright notices, the [`NOTICE`](NOTICE) file and the licence text;
* license your modifications and the combined work under AGPL-3.0;
* offer the **complete Corresponding Source** of the version you run or distribute to
  every user, including network users (AGPL-3.0 §13 "Remote Network Interaction").

Using the unmodified `ferroute` binary to route your own boards (including commercial
board designs) does **not** impose any licence on your PCB designs: the boards you route
are your output, not a derivative work of ferroute.

## 2. Commercial licence

A commercial licence from the copyright holder is required if you want to:

* embed ferroute (or parts of it, e.g. the router, DRC oracle or IPC client crates) in a
  proprietary / closed-source product;
* offer ferroute-based functionality over a network **without** publishing your source
  code under the AGPL;
* redistribute modified versions without the AGPL's source-disclosure obligations;
* obtain warranties, indemnification or support terms not offered by the AGPL.

The commercial licence grants these rights for a fee and can include support and custom
development.

**Contact:** reach the copyright holder via the GitHub profile
[@vanhanit](https://github.com/vanhanit), or open an issue titled
*"Commercial licensing inquiry"* in this repository (do not post confidential details in
public issues — ask for a private channel instead).

## 3. Contributions

To keep dual licensing possible, contributions are accepted under the terms in
[`CONTRIBUTING.md`](CONTRIBUTING.md): you license your contribution under AGPL-3.0-only
**and** grant the copyright holder a perpetual, irrevocable right to relicense it,
including under commercial terms.

*This file is an informal summary, not legal advice. The AGPL-3.0 text in `LICENSE`
and any signed commercial agreement are authoritative.*
