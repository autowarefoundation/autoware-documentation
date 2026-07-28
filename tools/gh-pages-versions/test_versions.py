# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for versions.py."""

import pytest
from versions import CATEGORY_DEFAULT_BRANCH
from versions import CATEGORY_OTHER
from versions import CATEGORY_PULL_REQUEST
from versions import CATEGORY_RELEASE
from versions import categorize
from versions import parse_pull_request_number
from versions import parse_release
from versions import reorder_entries
from versions import select_versions_to_delete

# What the gh-pages branch held when these scripts were written. The order is the
# one that was in versions.json at the time, to show that the input order does not
# matter.
DEPLOYED_ON_GH_PAGES = [
    "main",
    "pr-812",
    "pr-810",
    "pr-808",
    "pr-807",
    "pr-803",
    "1.5.0",
    "1.6.0",
    "1.7.0",
    "1.7.1",
    "1.8.0",
]


def entry(version):
    """Build a versions.json entry the way mike writes it."""
    return {"version": version, "title": version, "aliases": []}


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("1.8.0", (1, 8, 0)),
        ("1.7.1", (1, 7, 1)),
        ("2.0.0", (2, 0, 0)),
        ("1.10.0", (1, 10, 0)),
        ("1.9", (1, 9, 0)),
        ("0.0.0", (0, 0, 0)),
    ],
)
def test_parse_release_reads_the_numbers_of_a_release(version, expected):
    assert parse_release(version) == expected


@pytest.mark.parametrize(
    "version",
    [
        "main",
        "galactic",
        "pr-812",
        "branch-my-feature",
        "1.9.0-rc1",
        "v1.9.0",
        "1",
        "1.8.0.1",
        "1..0",
        "1.x.0",
        "",
    ],
)
def test_parse_release_rejects_everything_that_is_not_a_release(version):
    assert parse_release(version) is None


def test_parse_release_rejects_digits_that_are_not_plain_numbers():
    # "²" answers True to str.isdigit() but cannot be read as a number.
    assert parse_release("1.²") is None


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("pr-812", 812),
        ("pr-1", 1),
        ("main", None),
        ("1.8.0", None),
        ("pr-", None),
        ("pr-abc", None),
        ("branch-pr-1", None),
    ],
)
def test_parse_pull_request_number(version, expected):
    assert parse_pull_request_number(version) == expected


@pytest.mark.parametrize(
    ("version", "expected"),
    [
        ("main", CATEGORY_DEFAULT_BRANCH),
        ("1.8.0", CATEGORY_RELEASE),
        ("pr-812", CATEGORY_PULL_REQUEST),
        ("branch-my-feature", CATEGORY_OTHER),
        ("galactic", CATEGORY_OTHER),
        ("1.9.0-rc1", CATEGORY_OTHER),
    ],
)
def test_categorize(version, expected):
    assert categorize(version, default_branch="main") == expected


def test_categorize_uses_the_given_default_branch():
    assert categorize("galactic", default_branch="galactic") == CATEGORY_DEFAULT_BRANCH
    assert categorize("main", default_branch="galactic") == CATEGORY_OTHER


def test_select_deletes_the_old_lines_and_the_replaced_patches():
    # Keeps 1.8.0, 1.7.1 and 1.6.0: the three newest lines, newest patch each.
    assert select_versions_to_delete(DEPLOYED_ON_GH_PAGES, keep_lines=3) == ["1.7.0", "1.5.0"]


def test_select_keeps_only_the_newest_patch_of_a_line():
    versions = ["1.8.0", "1.8.1", "1.8.2"]
    assert select_versions_to_delete(versions, keep_lines=3) == ["1.8.1", "1.8.0"]


def test_select_deletes_nothing_when_there_are_fewer_lines_than_kept():
    versions = ["1.7.0", "1.8.0"]
    assert select_versions_to_delete(versions, keep_lines=3) == []


def test_select_compares_releases_as_numbers_and_not_as_text():
    versions = ["1.9.0", "1.10.0", "1.11.0", "2.0.0"]
    assert select_versions_to_delete(versions, keep_lines=3) == ["1.9.0"]


def test_select_treats_a_two_number_release_as_the_same_line():
    versions = ["1.6.0", "1.7", "1.8.0", "1.9.0"]
    assert select_versions_to_delete(versions, keep_lines=3) == ["1.6.0"]


def test_select_never_touches_versions_that_are_not_releases():
    versions = ["main", "galactic", "pr-812", "branch-my-feature", "1.9.0-rc1"]
    assert select_versions_to_delete(versions, keep_lines=3) == []


def test_select_keeps_a_single_line():
    assert select_versions_to_delete(DEPLOYED_ON_GH_PAGES, keep_lines=1) == [
        "1.7.1",
        "1.7.0",
        "1.6.0",
        "1.5.0",
    ]


def test_select_always_keeps_at_least_one_release():
    # Whatever is deployed, deleting everything must not be possible.
    for keep_lines in range(1, 5):
        deleted = select_versions_to_delete(DEPLOYED_ON_GH_PAGES, keep_lines)
        releases = [version for version in DEPLOYED_ON_GH_PAGES if parse_release(version)]
        assert set(deleted) < set(releases)


def test_select_handles_an_empty_list():
    assert select_versions_to_delete([], keep_lines=3) == []


def test_select_rejects_keeping_fewer_than_one_line():
    with pytest.raises(ValueError):
        select_versions_to_delete(DEPLOYED_ON_GH_PAGES, keep_lines=0)


def test_reorder_groups_the_versions_of_the_gh_pages_branch():
    entries = [entry(version) for version in DEPLOYED_ON_GH_PAGES]
    reordered = reorder_entries(entries, default_branch="main")

    assert [item["version"] for item in reordered] == [
        "main",
        "1.8.0",
        "1.7.1",
        "1.7.0",
        "1.6.0",
        "1.5.0",
        "pr-812",
        "pr-810",
        "pr-808",
        "pr-807",
        "pr-803",
    ]


def test_reorder_compares_releases_and_pull_requests_as_numbers():
    entries = [entry(version) for version in ["1.9.0", "pr-9", "1.10.0", "pr-10", "main"]]
    reordered = reorder_entries(entries, default_branch="main")

    assert [item["version"] for item in reordered] == ["main", "1.10.0", "1.9.0", "pr-10", "pr-9"]


def test_reorder_lists_branches_last_and_by_name():
    entries = [entry(version) for version in ["branch-zebra", "galactic", "main", "branch-apple"]]
    reordered = reorder_entries(entries, default_branch="main")

    assert [item["version"] for item in reordered] == [
        "main",
        "branch-apple",
        "branch-zebra",
        "galactic",
    ]


def test_reorder_keeps_every_entry_untouched():
    entries = [entry(version) for version in DEPLOYED_ON_GH_PAGES]
    reordered = reorder_entries(entries, default_branch="main")

    assert sorted(reordered, key=lambda item: item["version"]) == sorted(
        entries, key=lambda item: item["version"]
    )


def test_reorder_handles_an_empty_file():
    assert reorder_entries([], default_branch="main") == []


def test_reorder_handles_a_gh_pages_branch_without_the_default_branch():
    entries = [entry(version) for version in ["1.8.0", "pr-1"]]
    reordered = reorder_entries(entries, default_branch="main")

    assert [item["version"] for item in reordered] == ["1.8.0", "pr-1"]
