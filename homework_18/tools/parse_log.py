#!/usr/bin/env python3
"""Розбір лога пристрою (ДЗ18).

Доводить, що формат рядка стану придатний не лише для очей, а й для скрипта.

    python3 tools/parse_log.py log.txt
    python3 tools/parse_log.py < log.txt

Виводить: кількість рядків, фактичний період (мін/сер/макс за мітками часу),
діапазони ax/ay/az, скільки разів був i2c=err.
"""

import re
import sys
from statistics import mean

PAIR = re.compile(r"(\w+)=(-?[\d.]+|\w+)")


def parse(stream):
    rows = []
    for raw in stream:
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        fields = dict(PAIR.findall(line))
        if "t" not in fields or "ax" not in fields:
            continue
        try:
            rows.append(
                {
                    "t": int(fields["t"]),
                    "ax": float(fields["ax"]),
                    "ay": float(fields["ay"]),
                    "az": float(fields["az"]),
                    "servo": int(fields.get("servo", 0)),
                    "mode": fields.get("mode", "?"),
                    "i2c": fields.get("i2c", "?"),
                }
            )
        except ValueError:
            continue
    return rows


def main():
    src = open(sys.argv[1], encoding="utf-8", errors="replace") if len(sys.argv) > 1 else sys.stdin
    rows = parse(src)

    if not rows:
        print("Жодного рядка стану не знайдено.")
        return 1

    dts = [b["t"] - a["t"] for a, b in zip(rows, rows[1:])]
    print(f"рядків стану : {len(rows)}")
    if dts:
        print(f"період, мс   : min={min(dts)}  avg={mean(dts):.1f}  max={max(dts)}")
    for axis in ("ax", "ay", "az"):
        vals = [r[axis] for r in rows]
        print(f"{axis}, g        : min={min(vals):+.2f}  max={max(vals):+.2f}")
    print(f"серво, град  : min={min(r['servo'] for r in rows)}  max={max(r['servo'] for r in rows)}")
    print(f"i2c=err      : {sum(1 for r in rows if r['i2c'] != 'ok')}")
    print(f"режими       : {', '.join(sorted({r['mode'] for r in rows}))}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
