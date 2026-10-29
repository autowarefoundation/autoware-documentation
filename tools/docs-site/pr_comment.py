# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Write the comment that links a pull request to its documentation preview.

The comment gives the address of the preview, and the address of each page
that the pull request changes. The deploy-docs action of
autowarefoundation/autoware-github-actions wrote the same comment, so the
publish-docs workflow updates the comments that it left.
"""

import argparse
import json
from pathlib import PurePosixPath
import subprocess
import sys

DOCS_DIRECTORY = "docs/"


def page_address(path):
    """Return the address of the page of a Markdown file, relative to the version.

    "docs/home/index.md" gives "home/", and "docs/home/vision.md" gives
    "home/vision/". A file that is not a page of the site gives None.
    """
    if not path.startswith(DOCS_DIRECTORY) or not path.endswith(".md"):
        return None
    page = PurePosixPath(path[len(DOCS_DIRECTORY) :])
    directory = "" if str(page.parent) == "." else f"{page.parent}/"
    if page.name in ("index.md", "README.md"):
        return directory
    return f"{directory}{page.stem}/"


def comment_body(base_url, changed_files):
    """Return the comment for a preview at base_url and the changed files."""
    lines = [
        f"Documentation URL: {base_url}",
        "The preview should appear a few minutes after the build, when the publish-docs workflow finishes.",
        "Modified URLs:",
    ]
    for path in changed_files:
        address = page_address(path)
        if address is not None:
            lines.append(f"- {base_url}{address}")
    return "\n".join(lines) + "\n"


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--repository", required=True, help="owner/name of the repository")
    parser.add_argument(
        "--pull-request", required=True, type=int, help="number of the pull request"
    )
    parser.add_argument("--site-url", required=True, help="address of the site root")
    args = parser.parse_args()

    site_url = args.site_url if args.site_url.endswith("/") else args.site_url + "/"
    pull = json.loads(
        subprocess.run(
            ["gh", "pr", "view", str(args.pull_request), "-R", args.repository, "--json", "files"],
            check=True,
            capture_output=True,
            text=True,
        ).stdout
    )
    changed = [file["path"] for file in pull["files"]]
    sys.stdout.write(comment_body(f"{site_url}pr-{args.pull_request}/", changed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
