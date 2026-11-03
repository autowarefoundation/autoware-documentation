# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for llmstxt_sections.py."""

from helpers import write
from llmstxt_sections import unlisted_pages

CONFIG = """
[project.plugins.llmstxt.sections]
Home = ["home/*.md"]
"Reference design" = [{ "reference-design/*.md" = "The reference design" }]
"""


def test_every_page_of_a_listed_directory_passes(tmp_path):
    write(tmp_path / "zensical.toml", CONFIG)
    write(tmp_path / "docs" / "home" / "index.md", "# Home")
    write(tmp_path / "docs" / "home" / "roadmap" / "index.md", "# Roadmap")
    write(tmp_path / "docs" / "reference-design" / "index.md", "# Reference design")
    write(tmp_path / "docs" / "assets" / "notes.md", "# Not a page")
    assert unlisted_pages(tmp_path / "zensical.toml", tmp_path / "docs") == []


def test_a_new_top_level_directory_is_found(tmp_path):
    write(tmp_path / "zensical.toml", CONFIG)
    write(tmp_path / "docs" / "home" / "index.md", "# Home")
    write(tmp_path / "docs" / "datasets" / "index.md", "# Datasets")
    assert unlisted_pages(tmp_path / "zensical.toml", tmp_path / "docs") == ["datasets/index.md"]
