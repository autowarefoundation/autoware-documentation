# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for preview_comment.py."""

import preview_comment
import pytest

BASE_URL = "https://example.com/docs/pr-812/"


@pytest.mark.parametrize(
    ("path", "url"),
    [
        ("docs/how-to/page.md", BASE_URL + "how-to/page/"),
        ("docs/how-to/index.md", BASE_URL + "how-to/"),
        ("docs/how-to/README.md", BASE_URL + "how-to/"),
        ("docs/index.md", BASE_URL),
        ("docs/how-to/page.ja.md", BASE_URL + "how-to/page/ja/"),
        ("docs/how-to/image.png", None),
        ("README.md", None),
        ("tools/gh-pages-versions/README.md", None),
    ],
)
def test_page_url(path, url):
    assert preview_comment.page_url(BASE_URL, path) == url


def test_comment_body_names_the_commit_and_the_changed_pages():
    body = preview_comment.comment_body(
        BASE_URL, "1a2b3c4", ["docs/how-to/page.md", "mkdocs.yaml", "docs/index.md"]
    )

    assert body == (
        f"Documentation URL: {BASE_URL}\n"
        "Commit: 1a2b3c4\n"
        "Modified URLs:\n"
        f"- {BASE_URL}how-to/page/\n"
        f"- {BASE_URL}\n"
    )
