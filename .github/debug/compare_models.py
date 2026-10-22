"""Compare two Gurobi MPS models (and their hint files) by variable and row names.

Temporary debugging aid for the platform-dependent runtime of
VSSGenMBInfoSolver.Default7. Prints a Markdown report to stdout.

Usage: compare_models.py <name_a> <a.mps> <a.hnt> <name_b> <b.mps> <b.hnt>
"""

import sys


def read_mps(path):
    rows, cols, ints = [], [], set()
    coef, rhs, bounds, qc, gencons = {}, {}, {}, {}, []
    section, qc_row, is_int, last_col = "", None, False, None
    with open(path, encoding="ascii") as f:
        for line in f:
            if not line.strip() or line.startswith("*"):
                continue
            if not line[0].isspace():
                parts = line.split()
                section = parts[0]
                qc_row = parts[1] if section == "QCMATRIX" else None
                continue
            p = line.split()
            if section == "ROWS":
                rows.append((p[1], p[0]))
            elif section == "COLUMNS":
                if len(p) >= 3 and p[1] == "'MARKER'":
                    is_int = p[2] == "'INTORG'"
                    continue
                if p[0] != last_col:
                    cols.append(p[0])
                    last_col = p[0]
                    if is_int:
                        ints.add(p[0])
                for i in range(1, len(p) - 1, 2):
                    coef[(p[0], p[i])] = float(p[i + 1])
            elif section == "RHS":
                for i in range(1, len(p) - 1, 2):
                    rhs[p[i]] = float(p[i + 1])
            elif section == "BOUNDS":
                bounds[(p[0], p[2])] = float(p[3]) if len(p) > 3 else None
            elif section == "QCMATRIX":
                qc[(qc_row, p[0], p[1])] = float(p[2])
            elif section == "GENCONS":
                gencons.append(" ".join(p))
    return {"rows": rows, "cols": cols, "ints": ints, "coef": coef, "rhs": rhs,
            "bounds": bounds, "qc": qc, "gencons": gencons}


def read_hnt(path):
    hints = {}
    with open(path, encoding="ascii") as f:
        for line in f:
            p = line.split()
            if line.startswith("#") or len(p) < 2:
                continue
            hints[p[0]] = float(p[1])
    return hints


def diff_values(title, a, b, limit=15):
    keys_a, keys_b = set(a), set(b)
    only_a, only_b = keys_a - keys_b, keys_b - keys_a
    diffs = []
    for k in keys_a & keys_b:
        va, vb = a[k], b[k]
        if va != vb:
            d = abs(va - vb) if va is not None and vb is not None else float("inf")
            diffs.append((d, k, va, vb))
    diffs.sort(key=lambda x: -x[0])
    print(f"**{title}**: {len(keys_a)} vs {len(keys_b)} entries, "
          f"{len(only_a)} only in A, {len(only_b)} only in B, {len(diffs)} differ")
    for k in sorted(only_a, key=str)[:5]:
        print(f"- only in A: `{k}` = {a[k]!r}")
    for k in sorted(only_b, key=str)[:5]:
        print(f"- only in B: `{k}` = {b[k]!r}")
    if diffs:
        rel = max(d / max(abs(va), abs(vb), 1e-300) for d, _, va, vb in diffs
                  if va is not None and vb is not None)
        print(f"- max abs diff {diffs[0][0]!r}, max rel diff {rel!r}")
        for d, k, va, vb in diffs[:limit]:
            print(f"- `{k}`: {va!r} vs {vb!r} (diff {d!r})")
    print()


def order_report(title, a, b):
    if a == b:
        print(f"**{title} order**: identical ({len(a)})\n")
        return
    same_set = set(a) == set(b)
    pos_b = {n: i for i, n in enumerate(b)}
    moved = sum(1 for i, n in enumerate(a) if pos_b.get(n) != i)
    first = next(i for i, (x, y) in enumerate(zip(a, b)) if x != y) if len(a) == len(b) else None
    print(f"**{title} order**: differs, same names: {same_set}, "
          f"{moved} of {len(a)} at a different position")
    if first is not None:
        print(f"- first difference at {first}: `{a[first]}` vs `{b[first]}`")
        print(f"- A[{first}:{first + 6}] = {a[first:first + 6]}")
        print(f"- B[{first}:{first + 6}] = {b[first:first + 6]}")
    print()


def main():
    name_a, mps_a, hnt_a, name_b, mps_b, hnt_b = sys.argv[1:7]
    a, b = read_mps(mps_a), read_mps(mps_b)
    print(f"### Model comparison: A = {name_a}, B = {name_b}\n")
    order_report("Row", [r for r, _ in a["rows"]], [r for r, _ in b["rows"]])
    order_report("Column", a["cols"], b["cols"])
    diff_values("Row senses", {r: s for r, s in a["rows"]}, {r: s for r, s in b["rows"]})
    diff_values("Integer columns", {c: 1.0 for c in a["ints"]}, {c: 1.0 for c in b["ints"]})
    diff_values("Matrix coefficients", a["coef"], b["coef"])
    diff_values("Right-hand sides", a["rhs"], b["rhs"])
    diff_values("Bounds", a["bounds"], b["bounds"])
    diff_values("Quadratic coefficients", a["qc"], b["qc"])
    gen_a, gen_b = sorted(a["gencons"]), sorted(b["gencons"])
    print(f"**General constraints**: identical as a set: {gen_a == gen_b}, "
          f"identical in order: {a['gencons'] == b['gencons']}\n")
    diff_values("Hints", read_hnt(hnt_a), read_hnt(hnt_b))


if __name__ == "__main__":
    main()
