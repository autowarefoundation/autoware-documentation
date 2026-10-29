# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for pr_comment.py."""

from pr_comment import comment_body
from pr_comment import page_address
import pytest


@pytest.mark.parametrize(
    ("path", "expected"),
    [
        ("docs/home/index.md", "home/"),
        ("docs/home/vision-mission-core-values.md", "home/vision-mission-core-values/"),
        ("docs/index.md", ""),
        ("docs/contributing/README.md", "contributing/"),
        ("docs/home/images/picture.png", None),
        ("README.md", None),
        ("mkdocs.yaml", None),
    ],
)
def test_page_address(path, expected):
    assert page_address(path) == expected


def test_comment_body_lists_only_the_changed_pages():
    body = comment_body(
        "https://example.org/docs/pr-12/",
        ["docs/home/index.md", "mkdocs.yaml", "docs/demos/lane-change.md"],
    )
    assert body == (
        "Documentation URL: https://example.org/docs/pr-12/\n"
        "The preview should appear a few minutes after the build, when the publish-docs workflow finishes.\n"
        "Modified URLs:\n"
        "- https://example.org/docs/pr-12/home/\n"
        "- https://example.org/docs/pr-12/demos/lane-change/\n"
    )
