#!/usr/bin/env python3
# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
"""Embed a fallback style into each diagram, so that the SVG file also looks right on its own.

GitHub and other viewers show an SVG file without the CSS of the site. Then every class has no style, and the
shapes are black. This script adds a <style> and a <defs> element with the light-mode styles to each SVG file.
Both elements have the attribute data-aw-standalone. The diagram() macro in main.py removes them when it puts the
SVG into a page, so that the page uses the theme and follows the dark mode.

The script reads the diagram rules and the light-mode colors from docs/assets/css/theme.css, and the shared
markers, gradients, patterns, and filters from overrides/main.html. Each file gets only the rules and definitions
that it uses. Run it again after you change one of these files or a diagram:

    python3 tools/diagrams/standalone.py docs/design
"""

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
THEME = ROOT / "docs/assets/css/theme.css"
TEMPLATE = ROOT / "overrides/main.html"

# The colors and fonts that the theme of the site gives to the Material variables in light mode. The values have no
# quoted strings: the SVG formatter wraps long lines at spaces, and a line break inside a CSS string breaks the rule.
MATERIAL = {
    "--md-default-fg-color": "#000000de",
    "--md-default-fg-color--light": "#0000008c",
    "--md-default-fg-color--lighter": "#00000052",
    "--md-default-fg-color--lightest": "#0000000d",
    "--md-default-bg-color": "#fff",
    "--md-text-font-family": "Roboto, system-ui, sans-serif",
    "--md-code-font-family": "Roboto Mono, ui-monospace, monospace",
}
# Selectors whose custom properties hold the light-mode values, in the order that the theme applies them.
VARIABLE_SELECTORS = [
    ":root",
    "[data-md-color-scheme]",
    '[data-md-color-scheme="default"]',
    ".md-typeset .aw-diagram",
]
# Rules outside .aw-diagram that the diagrams also need: arrow heads, the dot pattern, and gradient stops.
EXTRA = re.compile(r"\.aw-d-arrow|\.aw-dot\b|\.aw-glass-[ab]\b|\.aw-flow-traj-[ab]\b")
BLOCK = re.compile(r'\s*<(style|defs) data-aw-standalone="">.*?</\1>', re.S)


def top_level(css):
    """Yield (selector, body) for each top-level item of a stylesheet, without comments."""
    css = re.sub(r"/\*.*?\*/", "", css, flags=re.S)
    i = 0
    while True:
        start = css.find("{", i)
        if start < 0:
            return
        depth, k = 0, start
        while True:
            if css[k] == "{":
                depth += 1
            elif css[k] == "}":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        yield css[i:start].strip(), css[start + 1 : k]
        i = k + 1


def variables(css):
    values = dict(MATERIAL)
    for selector, body in top_level(css):
        if selector in VARIABLE_SELECTORS:
            for name, value in re.findall(r"(--[\w-]+)\s*:\s*([^;]+);", body):
                values[name] = value.strip()
    return values


def diagram_rules(css):
    """Return (selectors, text) for each rule that styles a diagram, with the .aw-diagram prefix removed."""
    rules = []
    for selector, body in top_level(css):
        if selector.startswith("@keyframes aw-flow"):
            rules.append(([".flow"], f"{selector} {{{body}}}"))
        elif selector.startswith("@media"):
            inner = [(s, b) for s, b in top_level(body) if s.startswith(".aw-diagram ")]
            for s, b in inner:
                parts = [p.strip().replace(".aw-diagram ", "") for p in s.split(",")]
                rules.append((parts, f"{selector} {{{', '.join(parts)} {{{b}}}}}"))
        elif ".md-typeset" not in selector and (
            ".aw-diagram " in selector or EXTRA.search(selector)
        ):
            parts = [p.strip().replace(".aw-diagram ", "") for p in selector.split(",")]
            rules.append((parts, f"{', '.join(parts)} {{{body}}}"))
    return rules


def definitions(template):
    defs = re.search(r"<defs>(.*?)</defs>", template, re.S).group(1)
    items = {}
    for match in re.finditer(
        r"<(marker|linearGradient|radialGradient|pattern|filter)\b[^>]*\bid=\"([^\"]+)\".*?</\1>",
        defs,
        re.S,
    ):
        items[match.group(2)] = match.group(0)
    return items


def classes_in(selector):
    return set(re.findall(r"\.([\w-]+)", selector))


def standalone_block(svg, values, rules, defs):
    used = set()
    for attribute in re.findall(r'class="([^"]+)"', svg):
        used |= set(attribute.split())
    chosen_defs, chosen_rules = {}, []
    for _ in range(3):
        chosen_rules = [text for parts, text in rules if any(classes_in(p) <= used for p in parts)]
        ids = set(re.findall(r"url\(\"?#([\w-]+)", " ".join(chosen_rules) + svg))
        chosen_defs = {i: defs[i] for i in ids if i in defs}
        for text in chosen_defs.values():
            for attribute in re.findall(r'class="([^"]+)"', text):
                used |= set(attribute.split())
    needed = set(re.findall(r"var\((--[\w-]+)", " ".join(chosen_rules))) | {
        "--md-text-font-family",
        "--aw-d-bg",
    }
    for _ in range(3):
        for name in list(needed):
            needed |= set(re.findall(r"var\((--[\w-]+)", values.get(name, "")))
    declarations = "".join(f"{name}: {values[name]}; " for name in sorted(needed) if name in values)
    css = (
        f"svg {{ {declarations}font-family: var(--md-text-font-family); background-color: var(--aw-d-bg, #fff); }} "
        + " ".join(chosen_rules)
    )
    defs_text = "".join(chosen_defs[i] for i in sorted(chosen_defs))
    block = f'\n  <style data-aw-standalone="">{css}</style>'
    if defs_text:
        block += f'\n  <defs data-aw-standalone="">{defs_text}</defs>'
    return block


def main():
    folder = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else ROOT / "docs/design")
    css = THEME.read_text(encoding="utf-8")
    values, rules = variables(css), diagram_rules(css)
    defs = definitions(TEMPLATE.read_text(encoding="utf-8"))
    count = 0
    for path in sorted(folder.rglob("*.svg")):
        svg = path.read_text(encoding="utf-8")
        if 'class="' not in svg:
            continue
        svg = BLOCK.sub("", svg)
        title = re.search(r"</title>", svg)
        anchor = title.end() if title else re.search(r"<svg\b[^>]*>", svg).end()
        svg = svg[:anchor] + standalone_block(svg, values, rules, defs) + svg[anchor:]
        path.write_text(svg, encoding="utf-8")
        count += 1
    print(f"{count} diagrams updated")


if __name__ == "__main__":
    main()
