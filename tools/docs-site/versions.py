# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Reason about the versions of the documentation site.

The site holds one directory per version and lists them in versions.json. A
version is named after where its documentation comes from:

- the default branch, for example "main"
- a release tag, for example "1.8.0"
- a pull request preview, for example "pr-812"
- any other branch, for example "branch-my-feature"

This module classifies those names and puts the entries in the order they
should appear in the version dropdown of the site.
"""

# Releases are compared as major, minor and patch, so "1.9" counts as "1.9.0".
RELEASE_NUMBER_COUNT = 3

# What a pull request preview is called, for example "pr-812".
PULL_REQUEST_PREFIX = "pr-"

CATEGORY_DEFAULT_BRANCH = "default branch"
CATEGORY_RELEASE = "release"
CATEGORY_PULL_REQUEST = "pull request"
CATEGORY_OTHER = "other"


def is_number(text):
    """Tell whether the text is written with the plain digits 0 to 9."""
    return text.isascii() and text.isdigit()


def parse_release(version):
    """Return the numbers of a release version, or None if it is not a release.

    A release is written as two or three plain numbers separated by dots, and is
    always returned as three numbers, so "1.8.0" gives (1, 8, 0) and "1.9" gives
    (1, 9, 0).

    Everything else gives None: the default branch "main", a pull request
    preview "pr-812", a branch preview "branch-my-feature" and a pre-release tag
    such as "1.9.0-rc1" are all left alone by the callers of this function.
    """
    parts = version.split(".")
    if len(parts) < 2 or len(parts) > RELEASE_NUMBER_COUNT:
        return None
    if not all(is_number(part) for part in parts):
        return None

    numbers = [int(part) for part in parts]
    while len(numbers) < RELEASE_NUMBER_COUNT:
        numbers.append(0)
    return tuple(numbers)


def parse_pull_request_number(version):
    """Return the number of a pull request preview, or None if it is not one."""
    if not version.startswith(PULL_REQUEST_PREFIX):
        return None

    number = version[len(PULL_REQUEST_PREFIX) :]
    if not is_number(number):
        return None
    return int(number)


def categorize(version, default_branch):
    """Return the group of the version dropdown that a version belongs to."""
    if version == default_branch:
        return CATEGORY_DEFAULT_BRANCH
    if parse_release(version) is not None:
        return CATEGORY_RELEASE
    if parse_pull_request_number(version) is not None:
        return CATEGORY_PULL_REQUEST
    return CATEGORY_OTHER


def reorder_entries(entries, default_branch):
    """Return the versions.json entries in the order the dropdown should list them.

    The default branch comes first, then the releases from newest to oldest,
    then the pull request previews from newest to oldest, then everything else
    by name. Every entry is kept, none is added.
    """

    def entries_of(category):
        return [
            entry for entry in entries if categorize(entry["version"], default_branch) == category
        ]

    def release_of(entry):
        return parse_release(entry["version"])

    def pull_request_number_of(entry):
        return parse_pull_request_number(entry["version"])

    def name_of(entry):
        return entry["version"]

    return [
        *entries_of(CATEGORY_DEFAULT_BRANCH),
        *sorted(entries_of(CATEGORY_RELEASE), key=release_of, reverse=True),
        *sorted(entries_of(CATEGORY_PULL_REQUEST), key=pull_request_number_of, reverse=True),
        *sorted(entries_of(CATEGORY_OTHER), key=name_of),
    ]
