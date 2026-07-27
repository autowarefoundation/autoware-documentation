#!/usr/bin/env python3

# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Delete outdated release documentation from the gh-pages branch.

Every release tag deploys a full copy of the site into its own directory on the
gh-pages branch. This script keeps the newest minor release lines, and of each
of those lines only the newest patch, and asks mike to delete the rest.

Run it from the root of the repository, where mkdocs.yaml is, because mike reads
its configuration from there:

    python3 tools/gh-pages-versions/delete_old_version_docs.py --dry-run
"""

import argparse
import json
import subprocess

from versions import parse_release
from versions import select_versions_to_delete

DEFAULT_KEEP_LINES = 3


def positive_number(text):
    """Read a command line argument that has to be a number of at least 1."""
    number = int(text)
    if number < 1:
        raise argparse.ArgumentTypeError(f"expected a number of at least 1, got {number}")
    return number


def list_deployed_versions():
    """Return the versions that mike has deployed on the gh-pages branch."""
    result = subprocess.run(
        ["mike", "list", "--json"],
        check=True,
        capture_output=True,
        text=True,
    )
    return [entry["version"] for entry in json.loads(result.stdout)]


def delete_versions(versions):
    """Delete the given versions from the gh-pages branch and push the result."""
    subprocess.run(["mike", "delete", "--push", *versions], check=True)


def report(title, versions):
    """Print a titled list of versions, or say that the list is empty."""
    print(f"{title}:")
    for version in versions:
        print(f"  {version}")
    if not versions:
        print("  (none)")
    print()


def main(argv=None):
    """Delete the outdated release documentation and report what happened."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--keep-lines",
        type=positive_number,
        default=DEFAULT_KEEP_LINES,
        help="how many of the newest minor release lines to keep (default: %(default)s)",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="report what would be deleted without deleting anything",
    )
    args = parser.parse_args(argv)

    deployed_versions = list_deployed_versions()
    versions_to_delete = select_versions_to_delete(deployed_versions, args.keep_lines)

    kept_releases = sorted(
        (
            version
            for version in deployed_versions
            if parse_release(version) is not None and version not in versions_to_delete
        ),
        key=parse_release,
        reverse=True,
    )

    report("Deployed versions", deployed_versions)
    report("Kept releases", kept_releases)
    report("Outdated releases", versions_to_delete)

    if not versions_to_delete:
        print("Nothing to delete.")
        return 0

    if args.dry_run:
        print("Dry run, nothing was deleted.")
        return 0

    delete_versions(versions_to_delete)
    print("Deleted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
