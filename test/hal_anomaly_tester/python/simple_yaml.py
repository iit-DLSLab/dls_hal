"""Minimal YAML reader (the dls2 Python interpreter has no PyYAML).

Supports what the robot interface configs and servers.yaml use: block
mappings by indentation, block lists of scalars, flow lists/mappings, scalars
and comments. Uses PyYAML instead when it is available.
"""

from __future__ import annotations

import re

try:  # pragma: no cover - depends on the interpreter
    import yaml as _pyyaml
except ImportError:
    _pyyaml = None


def load_file(path: str):
    with open(path, encoding="utf-8") as file:
        return loads(file.read())


def loads(text: str):
    if _pyyaml is not None:
        return _pyyaml.safe_load(text) or {}
    lines = []
    for raw in text.splitlines():
        line = _strip_comment(raw).rstrip()
        if line.strip():
            lines.append((len(line) - len(line.lstrip(" ")), line.strip()))
    value, _ = _block(lines, 0, 0)
    return value or {}


def _strip_comment(line: str) -> str:
    quote = None
    for i, char in enumerate(line):
        if char in "'\"":
            quote = None if quote == char else (quote or char)
        elif char == "#" and quote is None and (i == 0 or line[i - 1] in " \t"):
            return line[:i]
    return line


def _block(lines, start, indent):
    if start >= len(lines):
        return None, start
    if lines[start][1].startswith("- "):
        items, i = [], start
        while i < len(lines) and lines[i][0] == indent and lines[i][1].startswith("- "):
            items.append(_scalar_or_flow(lines[i][1][2:].strip()))
            i += 1
        return items, i
    mapping, i = {}, start
    while i < len(lines) and lines[i][0] == indent:
        key, _, rest = lines[i][1].partition(":")
        key, rest = _scalar(key.strip()), rest.strip()
        i += 1
        if rest:
            mapping[key] = _scalar_or_flow(rest)
        elif i < len(lines) and lines[i][0] > indent:
            mapping[key], i = _block(lines, i, lines[i][0])
        else:
            mapping[key] = None
    return mapping, i


def _scalar_or_flow(text: str):
    if text[:1] in "[{":
        value, rest = _flow(text, 0)
        return value
    return _scalar(text)


def _flow(text: str, i: int):
    opening = text[i]
    closing = "]" if opening == "[" else "}"
    items: list = []
    mapping: dict = {}
    i += 1
    while True:
        while text[i] in " ,":
            i += 1
        if text[i] == closing:
            return (items if opening == "[" else mapping), i + 1
        if text[i] in "[{":
            value, i = _flow(text, i)
            items.append(value)
            continue
        match = re.compile(r"[^,\]}]*").match(text, i)
        token = match.group(0)
        i = match.end()
        if opening == "{":
            key, _, value = token.partition(":")
            if text[i:i + 1] in ("[", "{"):
                raise ValueError("nested flow values inside flow mappings are not supported")
            mapping[_scalar(key.strip())] = _scalar(value.strip())
        else:
            items.append(_scalar(token.strip()))


def _scalar(text: str):
    if len(text) >= 2 and text[0] == text[-1] and text[0] in "'\"":
        return text[1:-1]
    lowered = text.lower()
    if lowered in ("true", "yes", "on"):
        return True
    if lowered in ("false", "no", "off"):
        return False
    if lowered in ("null", "~", ""):
        return None
    if lowered in (".inf", "+.inf"):
        return float("inf")
    if lowered == "-.inf":
        return float("-inf")
    if lowered == ".nan":
        return float("nan")
    try:
        return int(text)
    except ValueError:
        pass
    try:
        return float(text)
    except ValueError:
        return text
