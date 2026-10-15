#!/usr/bin/env python3

# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Publish a pull request preview that was built elsewhere to the gh-pages branch.

The build-docs-pr workflow builds the site of a pull request without any write
access, because it runs the code of the pull request. The deploy-docs workflow
then calls this script with trusted code only. The script treats the built site
as data: it checks it, copies it to the directory of the preview, and lists the
preview in versions.json the way mike does.

    python3 tools/gh-pages-versions/publish_preview.py path/to/site path/to/gh-pages pr-812
"""

import argparse
from pathlib import Path
import shutil

from reorder_versions import read_entries
from reorder_versions import write_entries
from versions import parse_pull_request_number
from versions import reorder_entries

DEFAULT_BRANCH = "main"


def check_version(version):
    """Allow only the name of a pull request preview, so nothing else on gh-pages is touched."""
    if parse_pull_request_number(version) is None:
        raise ValueError(f"expected a pull request preview such as pr-812, got {version!r}")


def check_site(site):
    """Refuse a site that could write outside its directory or that is not a site at all.

    The site comes from a build of untrusted code, so a link could point anywhere
    on the machine. Only plain directories and plain files are accepted.
    """
    if not site.is_dir() or site.is_symlink():
        raise ValueError(f"the site is not a directory: {site}")
    for path in site.rglob("*"):
        if path.is_symlink():
            raise ValueError(f"the site contains a link: {path.relative_to(site)}")
        if not path.is_dir() and not path.is_file():
            raise ValueError(f"the site contains a special file: {path.relative_to(site)}")
    if not (site / "index.html").is_file():
        raise ValueError("the site has no index.html")


def add_entry(entries, version, default_branch):
    """Return the versions.json entries with the preview listed once, in the dropdown order."""
    if not any(entry["version"] == version for entry in entries):
        entries = [*entries, {"version": version, "title": version, "aliases": []}]
    return reorder_entries(entries, default_branch)


def publish(site, gh_pages, version, default_branch):
    """Replace the directory of the preview on gh-pages with the site and list it."""
    check_version(version)
    check_site(site)

    target = gh_pages / version
    if target.is_symlink() or target.is_file():
        target.unlink()
    elif target.exists():
        shutil.rmtree(target)
    shutil.copytree(site, target)

    versions_json = gh_pages / "versions.json"
    entries = read_entries(versions_json) if versions_json.exists() else []
    write_entries(versions_json, add_entry(entries, version, default_branch))


def main(argv=None):
    """Publish a built site as a pull request preview and report what changed."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("site", type=Path, help="the built site of the pull request")
    parser.add_argument("gh_pages", type=Path, help="a checkout of the gh-pages branch")
    parser.add_argument("version", help="the name of the preview, for example pr-812")
    parser.add_argument(
        "--default-branch",
        default=DEFAULT_BRANCH,
        help="the branch that is listed first (default: %(default)s)",
    )
    args = parser.parse_args(argv)

    try:
        publish(args.site, args.gh_pages, args.version, args.default_branch)
    except ValueError as error:
        raise SystemExit(f"The preview was not published: {error}") from error

    print(f"Published {args.version} to {args.gh_pages}.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
