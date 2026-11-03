# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Check that the sections of the llmstxt plugin list every page.

Zensical writes the Markdown copy of a page only if a section of the llmstxt
plugin in zensical.toml lists the page. A page without a copy also has no copy
page button, and the build gives no warning. So every page must match a
pattern of a section, for example "home/*.md".
"""

import argparse
import fnmatch
from pathlib import Path
import sys
import tomllib

# Directories of docs/ that hold no pages.
IGNORED_DIRECTORIES = {"assets"}


def unlisted_pages(config_path, docs_dir):
    """Return the pages of docs_dir that no section of the llmstxt plugin lists."""
    with open(config_path, "rb") as file:
        config = tomllib.load(file)
    sections = config["project"]["plugins"]["llmstxt"]["sections"]
    patterns = []
    for entries in sections.values():
        for entry in entries:
            # An entry is a pattern, or a mapping of a pattern to its description.
            patterns.append(entry if isinstance(entry, str) else next(iter(entry)))
    docs_dir = Path(docs_dir)
    pages = []
    for path in sorted(docs_dir.rglob("*.md")):
        page = path.relative_to(docs_dir).as_posix()
        if page.split("/")[0] in IGNORED_DIRECTORIES:
            continue
        if not any(fnmatch.fnmatchcase(page, pattern) for pattern in patterns):
            pages.append(page)
    return pages


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--config", default="zensical.toml")
    parser.add_argument("--docs", default="docs")
    args = parser.parse_args()
    pages = unlisted_pages(args.config, args.docs)
    for page in pages:
        print(f"{page}: no section of the llmstxt plugin in {args.config} lists this page")
    return 1 if pages else 0


if __name__ == "__main__":
    sys.exit(main())
