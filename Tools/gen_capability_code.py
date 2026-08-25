#!/usr/bin/env python3
"""Generate the function-browser code database for the capability
dashboard (Docs/CapabilityChecklist.html).

Extracts EVERY function and struct (with nested methods) from the
capability library sources, computes the intra-library call graph,
and splices the result as JSON between the FNCODE markers inside the
dashboard's script block. Rerun after editing library sources:

    python Tools/gen_capability_code.py

Sources covered (the library + the pinned engine-mirror exports):
  - Source/Solver/SolverCapability.h   (everything)
  - Source/Solver/SolverStrafe.h       (everything)
  - Source/Solver/SolverMove.cpp       (the Fn:: export bodies, PLUS
    the real mirror implementations they forward to and MoveTick
    itself, grouped under the display prefix Move::)
  - Source/Solver/SolverAir.cpp        (Air::WishInputs only)

The certification suites (CmdCap* in SolverLab.cpp) are run logs,
not library code, and are deliberately not embedded.
"""

import json
import os
import re
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
HTML = os.path.join(ROOT, "Docs", "CapabilityChecklist.html")

MARK_BEGIN = "// ==== FNCODE-BEGIN"
MARK_END = "// ==== FNCODE-END"

CTRL_KEYWORDS = {
    "if", "for", "while", "switch", "return", "sizeof", "else",
    "do", "catch", "case", "new", "delete", "static_assert",
}

FUNC_START = re.compile(
    r"^\s*(?:inline\s+|static\s+|constexpr\s+)*"
    r"[A-Za-z_][\w:<>,&\*\s]*?[\s\*&]"
    r"([A-Za-z_]\w*)\s*\(")
STRUCT_START = re.compile(r"^\s*struct\s+([A-Za-z_]\w*)\s*\{")
NS_START = re.compile(r"^\s*namespace\s+([A-Za-z_]\w*)\s*\{")


def strip_noise(line):
    """Remove string/char literals and // comments for brace/paren
    counting and call scanning."""
    out = []
    i, n = 0, len(line)
    while i < n:
        c = line[i]
        if c == '/' and i + 1 < n and line[i + 1] == '/':
            break
        if c == '"' or c == "'":
            q = c
            i += 1
            while i < n:
                if line[i] == '\\':
                    i += 2
                    continue
                if line[i] == q:
                    break
                i += 1
            out.append('""' if q == '"' else "''")
            i += 1
            continue
        out.append(c)
        i += 1
    return "".join(out)


class Entry(object):
    def __init__(self, qual, kind, path, line0, line1, code, parent):
        self.qual = qual          # e.g. CapP2P::SolveFixedN
        self.kind = kind          # 'fn' | 'struct' | 'method'
        self.path = path          # repo-relative source path
        self.line0 = line0        # 1-based first line (incl. comment)
        self.line1 = line1        # 1-based last line
        self.code = code
        self.parent = parent      # struct qual for methods, else None
        self.calls = []
        self.called_by = []


def leading_comment_start(lines, idx):
    """First line index of the contiguous // block directly above
    lines[idx] (returns idx if none)."""
    j = idx
    while j - 1 >= 0 and lines[j - 1].strip().startswith("//"):
        j -= 1
    return j


def capture_signature(lines, idx):
    """From a candidate function-start line, find where the body '{'
    opens. Returns (body_open_idx, is_declaration)."""
    depth = 0
    j = idx
    while j < len(lines):
        s = strip_noise(lines[j])
        for ch in s:
            if ch == '(':
                depth += 1
            elif ch == ')':
                depth -= 1
            elif ch == '{' and depth == 0:
                return j, False
            elif ch == ';' and depth == 0:
                return j, True
        j += 1
    return len(lines) - 1, True


def capture_block(lines, open_idx):
    """From the line holding the opening '{', return the index of the
    line holding its matching '}'."""
    depth = 0
    j = open_idx
    while j < len(lines):
        s = strip_noise(lines[j])
        for ch in s:
            if ch == '{':
                depth += 1
            elif ch == '}':
                depth -= 1
                if depth == 0:
                    return j
        j += 1
    return len(lines) - 1


def parse_source(text, relpath, ns_filter=None, name_filter=None):
    """Line parser over one file. ns_filter: only emit entries whose
    top capability namespace is in the set. name_filter: only emit
    entries whose short name is in the set."""
    lines = text.split("\n")
    entries = []
    # scope stack: (kind 'ns'|'struct'|'skip', name, entered_depth)
    stack = []
    depth = 0
    i = 0
    while i < len(lines):
        line = lines[i]
        s = strip_noise(line)
        at_code_scope = all(k[0] != "skip" for k in stack) and (
            not stack or stack[-1][0] in ("ns", "struct"))

        m = NS_START.match(line)
        if m:
            stack.append(("ns", m.group(1), depth))
            depth += s.count("{") - s.count("}")
            i += 1
            continue

        if at_code_scope:
            m = STRUCT_START.match(line)
            if m:
                sname = m.group(1)
                close = capture_block(lines, i)
                c0 = leading_comment_start(lines, i)
                qual = qual_name(stack, sname)
                code = "\n".join(lines[c0:close + 1])
                ent = Entry(qual, "struct", relpath, c0 + 1,
                            close + 1, code, None)
                if want(ent, ns_filter, name_filter, stack):
                    entries.append(ent)
                # descend into the struct to pick up methods
                stack.append(("struct", sname, depth))
                depth += s.count("{") - s.count("}")
                i += 1
                continue

            m = FUNC_START.match(line)
            if m and m.group(1) not in CTRL_KEYWORDS \
                    and "operator" not in line:
                fname = m.group(1)
                open_idx, is_decl = capture_signature(lines, i)
                if is_decl:
                    i += 1
                    depth += s.count("{") - s.count("}")
                    continue
                close = capture_block(lines, open_idx)
                c0 = leading_comment_start(lines, i)
                parent = None
                kind = "fn"
                if stack and stack[-1][0] == "struct":
                    parent = qual_name(stack[:-1], stack[-1][1])
                    kind = "method"
                qual = qual_name(stack, fname)
                code = "\n".join(lines[c0:close + 1])
                ent = Entry(qual, kind, relpath, c0 + 1, close + 1,
                            code, parent)
                if want(ent, ns_filter, name_filter, stack):
                    entries.append(ent)
                i = close + 1
                continue

        depth += s.count("{") - s.count("}")
        # pop scopes whose block ended
        while stack and depth <= stack[-1][2]:
            stack.pop()
        i += 1
    return entries


def qual_name(stack, name):
    parts = [nm for kind, nm, _ in stack if nm != "Solver"]
    parts.append(name)
    return "::".join(parts)


def want(ent, ns_filter, name_filter, stack):
    if name_filter is not None:
        short = ent.qual.split("::")[-1]
        if short not in name_filter:
            return False
    if ns_filter is not None:
        top = ent.qual.split("::")[0]
        if top not in ns_filter:
            return False
    return True


def extract_named(text, relpath, names, prefix):
    """Targeted extraction: find each named function's DEFINITION in
    a file too irregular for the scoped parser (declarations and call
    sites are skipped by requiring a body)."""
    lines = text.split("\n")
    entries = []
    for name in names:
        pat = re.compile(
            r"^\s*(?:static\s+|inline\s+)*"
            r"[A-Za-z_][\w:<>,&\*\s]*?[\s\*&]"
            + re.escape(name) + r"\s*\(")
        for i, line in enumerate(lines):
            if not pat.match(line):
                continue
            open_idx, is_decl = capture_signature(lines, i)
            if is_decl:
                continue
            close = capture_block(lines, open_idx)
            c0 = leading_comment_start(lines, i)
            code = "\n".join(lines[c0:close + 1])
            entries.append(Entry(prefix + "::" + name, "fn",
                                 relpath, c0 + 1, close + 1, code,
                                 None))
            break
    return entries


def build_call_graph(entries):
    by_qual = {e.qual: e for e in entries}
    # short-name -> quals (for same-namespace and dot-method calls)
    by_short = {}
    for e in entries:
        by_short.setdefault(e.qual.split("::")[-1], []).append(e.qual)
    qual_call = re.compile(r"([A-Za-z_]\w*(?:::[A-Za-z_]\w*)+)\s*\(")
    bare_call = re.compile(r"(?<![\w:.>])([A-Za-z_]\w*)\s*\(")
    dot_call = re.compile(r"[\w\)\]]\.\s*([A-Za-z_]\w*)\s*\(")
    for e in entries:
        body = "\n".join(strip_noise(l) for l in e.code.split("\n"))
        ns = "::".join(e.qual.split("::")[:-1])
        found = []
        for m in qual_call.finditer(body):
            q = m.group(1)
            if q in by_qual and q != e.qual:
                found.append(q)
            else:
                # e.g. CapAir::KernelTick matched with full path
                tail = q.split("::")
                hit = False
                for k in range(1, len(tail)):
                    cand = "::".join(tail[k - 1:])
                    if cand in by_qual and cand != e.qual:
                        found.append(cand)
                        hit = True
                        break
                # ::Solver::X forwards resolve to the Move:: mirror
                if not hit:
                    cand = "Move::" + tail[-1]
                    if cand in by_qual and cand != e.qual:
                        found.append(cand)
        for m in bare_call.finditer(body):
            nm = m.group(1)
            if nm in CTRL_KEYWORDS:
                continue
            cand = (ns + "::" + nm) if ns else nm
            if cand in by_qual and cand != e.qual:
                found.append(cand)
            elif "Move::" + nm in by_qual \
                    and "Move::" + nm != e.qual:
                found.append("Move::" + nm)
        for m in dot_call.finditer(body):
            nm = m.group(1)
            quals = by_short.get(nm, [])
            methods = [q for q in quals
                       if by_qual[q].kind == "method"]
            if len(methods) == 1 and methods[0] != e.qual:
                found.append(methods[0])
        seen = set()
        for q in found:
            if q not in seen:
                seen.add(q)
                e.calls.append(q)
    for e in entries:
        for q in e.calls:
            by_qual[q].called_by.append(e.qual)
    for e in entries:
        e.called_by = sorted(set(e.called_by))


def slice_namespace(text, ns):
    """Return only the top-level `namespace ns { ... }` slice of a
    file (with its line offset), or (None, 0)."""
    lines = text.split("\n")
    pat = re.compile(r"^\s*namespace\s+" + ns + r"\s*\{")
    for i, line in enumerate(lines):
        if pat.match(line):
            close = capture_block(lines, i)
            return "\n".join(lines[i:close + 1]), i
    return None, 0


def main():
    sources = []

    def load(rel):
        with open(os.path.join(ROOT, rel), "r", encoding="utf-8",
                  errors="replace") as f:
            return f.read()

    entries = []

    cap = load("Source/Solver/SolverCapability.h")
    entries += parse_source(cap, "Source/Solver/SolverCapability.h")
    sources.append("SolverCapability.h")

    strafe = load("Source/Solver/SolverStrafe.h")
    entries += parse_source(strafe, "Source/Solver/SolverStrafe.h")
    sources.append("SolverStrafe.h")

    move = load("Source/Solver/SolverMove.cpp")
    fn_slice, off = slice_namespace(move, "Fn")
    if fn_slice:
        sub = parse_source(fn_slice, "Source/Solver/SolverMove.cpp")
        for e in sub:
            e.line0 += off
            e.line1 += off
            if not e.qual.startswith("Fn::"):
                e.qual = "Fn::" + e.qual.split("::")[-1]
        entries += sub
        sources.append("SolverMove.cpp (Fn::)")

    # the real mirror implementations the Fn:: exports forward to,
    # plus the whole-tick mirror itself. Blank the Fn slice first
    # (preserving line numbers) so the forwarders can't shadow the
    # real bodies.
    mirror_names = ["MoveTick", "WishFromInput", "CategorizePosition",
                    "CheckJumpButton", "Duck", "CanUnduck",
                    "FinishDuck", "FinishUnDuck",
                    "HandleDuckingSpeedCrop", "EngineClipVelocity"]
    move_blanked = move
    if fn_slice:
        blank = "\n".join("" for _ in fn_slice.split("\n"))
        move_blanked = move.replace(fn_slice, blank, 1)
    mirror = extract_named(move_blanked,
                           "Source/Solver/SolverMove.cpp",
                           mirror_names, "Move")
    entries += mirror
    sources.append("SolverMove.cpp (Move:: mirror bodies)")

    air = load("Source/Solver/SolverAir.cpp")
    air_slice, off = slice_namespace(air, "Air")
    if air_slice:
        sub = parse_source(air_slice, "Source/Solver/SolverAir.cpp",
                           name_filter={"WishInputs"})
        for e in sub:
            e.line0 += off
            e.line1 += off
            if not e.qual.startswith("Air::"):
                e.qual = "Air::" + e.qual.split("::")[-1]
        entries += sub
        sources.append("SolverAir.cpp (Air::WishInputs)")

    # dedup (same qual can appear if a namespace re-opens with an
    # identically named helper - keep the first, warn)
    seen = {}
    unique = []
    for e in entries:
        if e.qual in seen:
            print("WARN duplicate entry:", e.qual, "(second at",
                  e.path, e.line0, ") - keeping the first")
            continue
        seen[e.qual] = e
        unique.append(e)
    entries = unique

    build_call_graph(entries)

    data = {}
    total_lines = 0
    for e in entries:
        total_lines += e.code.count("\n") + 1
        data[e.qual] = {
            "kind": e.kind,
            "file": e.path,
            "line": e.line0,
            "line_end": e.line1,
            "parent": e.parent,
            "calls": e.calls,
            "called_by": e.called_by,
            "code": e.code,
        }
    meta = {
        "functions": sum(1 for e in entries if e.kind == "fn"),
        "methods": sum(1 for e in entries if e.kind == "method"),
        "structs": sum(1 for e in entries if e.kind == "struct"),
        "lines": total_lines,
        "sources": sources,
    }

    payload = ("const FNCODE = "
               + json.dumps(data, indent=0, sort_keys=True)
               + ";\nconst FNCODE_META = "
               + json.dumps(meta) + ";")
    # never allow the script block to terminate early
    payload = payload.replace("</", "<\\/")

    with open(HTML, "r", encoding="utf-8") as f:
        html = f.read()
    b = html.find(MARK_BEGIN)
    e2 = html.find(MARK_END)
    if b < 0 or e2 < 0:
        print("ERROR: FNCODE markers not found in", HTML)
        sys.exit(1)
    b_line_end = html.find("\n", b)
    new_html = (html[:b_line_end + 1] + payload + "\n"
                + html[e2:])
    with open(HTML, "w", encoding="utf-8", newline="\n") as f:
        f.write(new_html)

    print("embedded %d entries (%d fn, %d methods, %d structs), "
          "%d source lines, from: %s"
          % (len(entries), meta["functions"], meta["methods"],
             meta["structs"], meta["lines"], ", ".join(sources)))
    resolved_calls = sum(len(e.calls) for e in entries)
    print("call graph: %d intra-library call edges" % resolved_calls)


if __name__ == "__main__":
    main()
