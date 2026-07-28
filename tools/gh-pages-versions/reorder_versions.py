#!/usr/bin/env python3

# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Reorder the entries of the versions.json file on the gh-pages branch.

mike rewrites the whole file in its own order every time it deploys or deletes:
the versions whose name does not start with a digit first, so the pull request
previews and the branch previews end up above the releases in the version
dropdown of the site. This script puts the default branch and the releases on
top instead.

    python3 tools/gh-pages-versions/reorder_versions.py path/to/versions.json
"""

import argparse
import json
from pathlib import Path

from versions import reorder_entries

DEFAULT_BRANCH = "main"


def read_entries(path):
    """Read the entries of a versions.json file."""
    return json.loads(path.read_text(encoding="utf-8"))


def write_entries(path, entries):
    """Write the entries the way mike writes them, so the file is not reformatted."""
    path.write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")


def main(argv=None):
    """Reorder a versions.json file in place and report the resulting order."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "versions_json",
        type=Path,
        help="the versions.json file of the gh-pages branch",
    )
    parser.add_argument(
        "--default-branch",
        default=DEFAULT_BRANCH,
        help="the branch that is listed first (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    entries = read_entries(args.versions_json)
    reordered_entries = reorder_entries(entries, args.default_branch)

    # A safety net: reordering must never lose or duplicate a deployed version.
    if len(reordered_entries) != len(entries):
        raise SystemExit(
            f"Reordering changed the number of entries from {len(entries)} "
            f"to {len(reordered_entries)}, so the file was left untouched."
        )

    print("New order:")
    for entry in reordered_entries:
        print(f"  {entry['version']}")

    write_entries(args.versions_json, reordered_entries)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
