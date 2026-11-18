"""A strict subset of YAML, read with the standard library only.

The tools that talk to the arm use the standard library only (codebase plan §2), but the
config files are YAML. This loader reads the subset the config files use and refuses
everything else, so that a file it accepts means the same thing to PyYAML. The test
`tests/elfin/test_yamlmini.py` loads every config file with both and compares.

Supported:
- `# comments`, on their own line or after a value;
- block mappings, `key: value` or `key:` followed by a more-indented block (spaces only);
- block sequences of scalars or flow lists, `- 1.0` or `- [1.0, 2.0]`;
- flow lists of scalars and nested flow lists, `[1.0, [2, 3]]`;
- scalars: null, ~, true, false, integers, floats written with a decimal point
  (1.0, 0.5, 1.0e-3), and strings, plain or quoted.

Refused: tabs, anchors and aliases, tags, flow mappings, multi-line strings, several
documents, the YAML 1.1 booleans (yes/no/on/off), and numbers that YAML 1.1 and 1.2 read
differently (1e-3, 0x10, 1_000, .5).
"""
import re

_INT = re.compile(r"[-+]?[0-9]+\Z")
_FLOAT = re.compile(r"[-+]?[0-9]+\.[0-9]*(?:[eE][-+][0-9]+)?\Z")
_AMBIGUOUS = {"yes", "no", "on", "off", "y", "n", "true", "false", "null"}


class YamlError(ValueError):
    pass


def _strip_comment(line):
    out, quote = [], None
    for i, ch in enumerate(line):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "'\"":
            quote = ch
        elif ch == "#" and (i == 0 or line[i - 1] in " \t"):
            break
        out.append(ch)
    if quote:
        raise YamlError(f"unterminated quote: {line!r}")
    return "".join(out).rstrip()


def _scalar(text, where):
    t = text.strip()
    if t == "":
        return None
    if t[0] in "&*!|>{%@`":
        raise YamlError(f"{where}: unsupported YAML construct {t!r}")
    if t[0] in "'\"":
        if len(t) < 2 or t[-1] != t[0]:
            raise YamlError(f"{where}: bad quoted string {t!r}")
        body = t[1:-1]
        if t[0] == "'":
            return body.replace("''", "'")
        if "\\" in body:
            raise YamlError(f"{where}: escapes in double-quoted strings are not supported: {t!r}")
        return body
    if t in ("null", "~", "Null", "NULL"):
        return None
    if t in ("true", "True", "TRUE"):
        return True
    if t in ("false", "False", "FALSE"):
        return False
    if t.lower() in _AMBIGUOUS:
        raise YamlError(f"{where}: {t!r} means different things in YAML 1.1 and 1.2; quote it")
    if _INT.match(t):
        return int(t)
    if _FLOAT.match(t):
        return float(t)
    try:
        float(t.replace("_", ""))
        numeric = True
    except ValueError:
        numeric = t[:2] in ("0x", "0o", "0b")
    if numeric or t.startswith("."):
        raise YamlError(f"{where}: write numbers with a decimal point and a signed exponent, e.g. 1.0e-3: {t!r}")
    if ": " in t or t.endswith(":") or " #" in t:
        raise YamlError(f"{where}: ambiguous plain string {t!r}; quote it")
    return t


def _flow_list(text, where):
    """Parse `[a, b, [c, d]]` with scalar items."""
    pos = 0

    def parse_list():
        nonlocal pos
        assert text[pos] == "["
        pos += 1
        items = []
        while True:
            while pos < len(text) and text[pos] == " ":
                pos += 1
            if pos >= len(text):
                raise YamlError(f"{where}: unterminated list")
            if text[pos] == "]":
                pos += 1
                return items
            if text[pos] == "[":
                items.append(parse_list())
            else:
                start, quote = pos, None
                while pos < len(text):
                    ch = text[pos]
                    if quote:
                        quote = None if ch == quote else quote
                    elif ch in "'\"":
                        quote = ch
                    elif ch in ",]":
                        break
                    pos += 1
                item = text[start:pos].strip()
                if item == "":
                    raise YamlError(f"{where}: empty list item")
                items.append(_scalar(item, where))
            while pos < len(text) and text[pos] == " ":
                pos += 1
            if pos < len(text) and text[pos] == ",":
                pos += 1
            elif pos < len(text) and text[pos] == "]":
                continue
            else:
                raise YamlError(f"{where}: expected , or ] in {text!r}")

    value = parse_list()
    if text[pos:].strip():
        raise YamlError(f"{where}: trailing text after list: {text!r}")
    return value


def _value(text, where):
    t = text.strip()
    if t.startswith("["):
        return _flow_list(t, where)
    return _scalar(t, where)


def loads(source, name="<string>"):
    lines = []
    for n, raw in enumerate(source.splitlines(), 1):
        if "\t" in raw:
            raise YamlError(f"{name}:{n}: tabs are not allowed")
        if raw.startswith("---") or raw.startswith("..."):
            raise YamlError(f"{name}:{n}: one document per file")
        line = _strip_comment(raw)
        if line.strip():
            lines.append((n, len(line) - len(line.lstrip(" ")), line.strip()))

    pos = 0

    def block(indent):
        nonlocal pos
        n, ind, text = lines[pos]
        if ind != indent:
            raise YamlError(f"{name}:{n}: unexpected indentation")
        if text.startswith("- ") or text == "-":
            return seq(indent)
        return mapping(indent)

    def mapping(indent):
        nonlocal pos
        out = {}
        while pos < len(lines):
            n, ind, text = lines[pos]
            if ind < indent:
                break
            if ind > indent:
                raise YamlError(f"{name}:{n}: unexpected indentation")
            m = re.match(r"([A-Za-z0-9_.\-]+):(?: (.*))?\Z", text)
            if not m:
                raise YamlError(f"{name}:{n}: expected 'key: value', got {text!r}")
            key, rest = m.group(1), (m.group(2) or "").strip()
            if key in out:
                raise YamlError(f"{name}:{n}: duplicate key {key!r}")
            pos += 1
            if rest:
                out[key] = _value(rest, f"{name}:{n}")
            elif pos < len(lines) and lines[pos][1] > indent:
                out[key] = block(lines[pos][1])
            else:
                out[key] = None
        return out

    def seq(indent):
        nonlocal pos
        out = []
        while pos < len(lines):
            n, ind, text = lines[pos]
            if ind < indent:
                break
            if ind > indent or not (text.startswith("- ") or text == "-"):
                raise YamlError(f"{name}:{n}: expected '- item' at this indentation")
            item = text[1:].strip()
            if re.match(r"[A-Za-z0-9_.\-]+:(?: |\Z)", item):
                raise YamlError(f"{name}:{n}: sequences of mappings are not supported")
            out.append(_value(item, f"{name}:{n}"))
            pos += 1
        return out

    if not lines:
        return None
    result = block(lines[0][1])
    if pos != len(lines):
        raise YamlError(f"{name}:{lines[pos][0]}: unexpected text")
    return result


def load(path):
    with open(path, encoding="utf-8") as f:
        return loads(f.read(), str(path))
