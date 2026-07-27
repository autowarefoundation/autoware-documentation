# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for reorder_versions.py."""

import json

import pytest
import reorder_versions

# A versions.json file in the order mike writes it: the names that do not start
# with a digit first, so the previews sit above the default branch and the releases.
VERSIONS_JSON = """[
  {
    "version": "pr-812",
    "title": "pr-812",
    "aliases": []
  },
  {
    "version": "main",
    "title": "main",
    "aliases": []
  },
  {
    "version": "1.8.0",
    "title": "1.8.0",
    "aliases": []
  }
]
"""

# The same file after reordering: the default branch, then the release, then the
# pull request preview. Only the order changes, the formatting stays the same.
REORDERED_VERSIONS_JSON = """[
  {
    "version": "main",
    "title": "main",
    "aliases": []
  },
  {
    "version": "1.8.0",
    "title": "1.8.0",
    "aliases": []
  },
  {
    "version": "pr-812",
    "title": "pr-812",
    "aliases": []
  }
]
"""


@pytest.fixture(name="versions_json")
def fixture_versions_json(tmp_path):
    """Write a versions.json file the way mike writes it."""
    path = tmp_path / "versions.json"
    path.write_text(VERSIONS_JSON, encoding="utf-8")
    return path


def test_main_reorders_the_file_in_place(versions_json):
    reorder_versions.main([str(versions_json)])

    assert versions_json.read_text(encoding="utf-8") == REORDERED_VERSIONS_JSON


def test_main_is_idempotent(versions_json):
    reorder_versions.main([str(versions_json)])
    reorder_versions.main([str(versions_json)])

    assert versions_json.read_text(encoding="utf-8") == REORDERED_VERSIONS_JSON


def test_main_accepts_another_default_branch(versions_json):
    reorder_versions.main([str(versions_json), "--default-branch", "galactic"])

    entries = json.loads(versions_json.read_text(encoding="utf-8"))
    assert [entry["version"] for entry in entries] == ["1.8.0", "pr-812", "main"]
