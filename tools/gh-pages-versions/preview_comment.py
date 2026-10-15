#!/usr/bin/env python3

# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Write the pull request comment that links a published preview.

The comment names the commit of the preview and links every Markdown page that
the pull request changes, the way the deploy-docs action of
autoware-github-actions does it. The changed files are read from standard input,
one path per line.

    gh pr diff 812 --name-only | python3 tools/gh-pages-versions/preview_comment.py \
        https://autowarefoundation.github.io/autoware-documentation/pr-812/ 1a2b3c4
"""

import argparse
import re
import sys

DOCS_DIR = "docs/"


def page_url(base_url, path):
    """Return the address of the page of a Markdown file, or None for any other file."""
    if not path.startswith(DOCS_DIR) or not path.endswith(".md"):
        return None
    page = path[len(DOCS_DIR) :]

    # A translated page such as "page.ja.md" is linked as "page/ja/".
    language = ""
    match = re.fullmatch(r"(.*)\.(\w+)(\.md)", page)
    if match:
        page = match.group(1) + match.group(3)
        language = match.group(2) + "/"

    if page.endswith("README.md"):
        page = page[: -len("README.md")]
    elif page.endswith("index.md"):
        page = page[: -len("index.md")]
    else:
        page = page[: -len(".md")] + "/"
    return base_url + page + language


def comment_body(base_url, commit, changed_files):
    """Return the comment, with the address of each changed page.

    The file names come from the pull request, so each address is written as an
    autolink in angle brackets, and a name cannot add its own Markdown.
    """
    lines = [f"Documentation URL: <{base_url}>", f"Commit: {commit}", "Modified URLs:"]
    for path in changed_files:
        url = page_url(base_url, path)
        if url is not None and not any(char in url for char in "<>() \t\r\n"):
            lines.append(f"- <{url}>")
    return "\n".join(lines) + "\n"


def main(argv=None):
    """Print the comment for the changed files given on standard input."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("base_url", help="the address of the preview, ending with a slash")
    parser.add_argument("commit", help="the commit that the preview was built from")
    args = parser.parse_args(argv)

    changed_files = [line.strip() for line in sys.stdin if line.strip()]
    sys.stdout.write(comment_body(args.base_url, args.commit, changed_files))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
