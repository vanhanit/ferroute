#!/usr/bin/env python3
"""Gate on a `kicad-cli pcb drc --format json` report. Stdlib only; run with `python3 -I`.

Fails (exit 1) when the report contains any error-severity violation or any unconnected
item (unless --allow-unconnected). Schematic-parity findings are ignored: the boards have
no schematic. Exit 2 on unreadable input.
"""
import argparse
import json
import sys


def severity(item):
    return str(item.get("severity", "error")).lower()


def describe(item):
    where = ", ".join(
        "(%s, %s)mm" % (p.get("pos", {}).get("x"), p.get("pos", {}).get("y"))
        for p in item.get("items", [])
        if isinstance(p, dict)
    )
    return "%s: %s %s" % (item.get("type", "?"), item.get("description", ""), where)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("report", nargs="+", help="kicad-cli DRC JSON report(s)")
    ap.add_argument("--allow-unconnected", action="store_true",
                    help="do not fail on unconnected_items")
    ap.add_argument("--max-print", type=int, default=20, help="max findings to print per category")
    args = ap.parse_args(argv)

    failed = False
    for path in args.report:
        try:
            with open(path, encoding="utf-8") as fh:
                data = json.load(fh)
        except (OSError, ValueError) as exc:
            print("ERROR: cannot read %s: %s" % (path, exc), file=sys.stderr)
            return 2

        violations = [v for v in data.get("violations", []) if severity(v) == "error"]
        warnings = [v for v in data.get("violations", []) if severity(v) != "error"]
        unconnected = data.get("unconnected_items", [])
        parity = data.get("schematic_parity", [])  # intentionally ignored

        print("%s: %d error violation(s), %d other violation(s), %d unconnected item(s), "
              "%d schematic-parity item(s) ignored"
              % (path, len(violations), len(warnings), len(unconnected), len(parity)))
        for v in violations[: args.max_print]:
            print("  VIOLATION " + describe(v))
        bad_unconnected = [] if args.allow_unconnected else unconnected
        for u in bad_unconnected[: args.max_print]:
            print("  UNCONNECTED " + describe(u))
        if violations or bad_unconnected:
            failed = True
    print("FAIL" if failed else "OK")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
