#!/usr/bin/env python3
"""Emulate arduino-cli's .ino prototype generation (ctags step) closely enough
to catch ordering problems: finds top-level function definitions (not class /
struct / namespace members), builds 'static <ret> name(args);' prototypes,
skips ones with default arguments or an existing declaration, and inserts
them at the line of the first top-level function definition.

Usage: arduino_proto_emul.py <sketch.ino> <out.cpp>
"""
import re
import sys

src = open(sys.argv[1]).read()
lines = src.split("\n")


def strip_code(line, state):
    """Remove comments and string/char literals; state tracks /* */ across lines."""
    out, i, n = [], 0, len(line)
    while i < n:
        if state["block"]:
            j = line.find("*/", i)
            if j < 0:
                return "".join(out)
            state["block"] = False
            i = j + 2
            continue
        c = line[i]
        if line.startswith("//", i):
            break
        if line.startswith("/*", i):
            state["block"] = True
            i += 2
            continue
        if c in "\"'":
            q, i = c, i + 1
            while i < n and line[i] != q:
                i += 2 if line[i] == "\\" else 1
            i += 1
            out.append(" ")
            continue
        out.append(c)
        i += 1
    return "".join(out)


FUNC_RE = re.compile(
    r"^(?P<pre>(?:static\s+|inline\s+)*)"
    r"(?P<ret>(?:const\s+)?[A-Za-z_][\w:<>]*(?:\s*[*&]+)?)\s+"
    r"(?P<name>[A-Za-z_][\w:]*)\s*\((?P<args>[^)]*)\)\s*(?:const\s*)?\{")
KEYWORDS = {"if", "for", "while", "switch", "return", "else", "do", "case", "sizeof"}

state = {"block": False}
depth = 0
scope_stack = []          # what opened each brace: 'type' or 'other'
funcs, decls = [], set()
first_func_line = None
pending_type = False
for ln_no, raw in enumerate(lines, start=1):
    code = strip_code(raw, state)
    stripped = code.strip()
    if depth == 0 and stripped:
        if re.match(r"^(struct|class|enum|union)\b", stripped):
            pending_type = True
        m = FUNC_RE.match(stripped)
        if m and m.group("name") not in KEYWORDS and "::" not in m.group("name"):
            funcs.append((ln_no, m))
            if first_func_line is None:
                first_func_line = ln_no
        # existing forward declarations like "static void f(int);"
        d = re.match(r"^(?:static\s+|inline\s+)*(?:const\s+)?[A-Za-z_][\w:<>]*(?:\s*[*&]+)?\s+([A-Za-z_]\w*)\s*\([^)]*\)\s*;", stripped)
        if d:
            decls.add(d.group(1))
    for ch in code:
        if ch == "{":
            scope_stack.append("type" if (pending_type and depth == 0) else "other")
            pending_type = False if depth == 0 else pending_type
            depth += 1
        elif ch == "}":
            depth -= 1
            scope_stack.pop()

protos = []
for ln_no, m in funcs:
    name, args = m.group("name"), m.group("args")
    if "=" in args or name in decls or name in ("setup", "loop"):
        continue
    static = "static " if "static" in m.group("pre") else ""
    protos.append(f'#line {ln_no} "sketch.ino"\n{static}{m.group("ret")} {name}({args});')

print(f"first top-level function at line {first_func_line}: {lines[first_func_line - 1].strip()[:70]}")
print(f"{len(funcs)} top-level functions, {len(protos)} prototypes generated")
out = lines[: first_func_line - 1] + protos + [f'#line {first_func_line} "sketch.ino"'] + lines[first_func_line - 1:]
open(sys.argv[2], "w").write("\n".join(out))
