# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for publish_preview.py."""

import json

import publish_preview
import pytest

VERSIONS = [
    {"version": "main", "title": "main", "aliases": []},
    {"version": "1.8.0", "title": "1.8.0", "aliases": []},
    {"version": "pr-700", "title": "pr-700", "aliases": []},
]


@pytest.fixture(name="site")
def fixture_site(tmp_path):
    """Write a small built site."""
    site = tmp_path / "site"
    (site / "home").mkdir(parents=True)
    (site / "index.html").write_text("new", encoding="utf-8")
    (site / "home" / "index.html").write_text("new home", encoding="utf-8")
    return site


@pytest.fixture(name="gh_pages")
def fixture_gh_pages(tmp_path):
    """Write a gh-pages checkout with a versions.json file and an old preview."""
    gh_pages = tmp_path / "gh-pages"
    (gh_pages / "main").mkdir(parents=True)
    (gh_pages / "main" / "index.html").write_text("main", encoding="utf-8")
    (gh_pages / "pr-812").mkdir()
    (gh_pages / "pr-812" / "old.html").write_text("old", encoding="utf-8")
    (gh_pages / "versions.json").write_text(json.dumps(VERSIONS, indent=2) + "\n", encoding="utf-8")
    return gh_pages


def read_versions(gh_pages):
    return json.loads((gh_pages / "versions.json").read_text(encoding="utf-8"))


def test_publish_replaces_the_preview_and_lists_it(site, gh_pages):
    publish_preview.publish(site, gh_pages, "pr-812", "main")

    assert (gh_pages / "pr-812" / "index.html").read_text(encoding="utf-8") == "new"
    assert (gh_pages / "pr-812" / "home" / "index.html").is_file()
    assert not (gh_pages / "pr-812" / "old.html").exists()
    assert (gh_pages / "main" / "index.html").read_text(encoding="utf-8") == "main"
    assert [entry["version"] for entry in read_versions(gh_pages)] == [
        "main",
        "1.8.0",
        "pr-812",
        "pr-700",
    ]


def test_publish_lists_a_preview_only_once(site, gh_pages):
    publish_preview.publish(site, gh_pages, "pr-700", "main")

    assert [entry["version"] for entry in read_versions(gh_pages)] == ["main", "1.8.0", "pr-700"]


def test_publish_writes_versions_json_the_way_mike_does(site, gh_pages):
    publish_preview.publish(site, gh_pages, "pr-812", "main")

    text = (gh_pages / "versions.json").read_text(encoding="utf-8")
    assert '{\n    "version": "pr-812",\n    "title": "pr-812",\n    "aliases": []\n  }' in text
    assert text.endswith("]\n")


@pytest.mark.parametrize("version", ["main", "1.8.0", "branch-x", "pr-", "pr-1/..", "../pr-1"])
def test_publish_refuses_anything_but_a_preview(site, gh_pages, version):
    with pytest.raises(ValueError, match="pull request preview"):
        publish_preview.publish(site, gh_pages, version, "main")

    assert (gh_pages / "main" / "index.html").read_text(encoding="utf-8") == "main"


def test_publish_refuses_a_link_in_the_site(site, gh_pages, tmp_path):
    (tmp_path / "secret").write_text("secret", encoding="utf-8")
    (site / "home" / "secret").symlink_to(tmp_path / "secret")

    with pytest.raises(ValueError, match="link"):
        publish_preview.publish(site, gh_pages, "pr-812", "main")

    assert (gh_pages / "pr-812" / "old.html").exists()


@pytest.mark.parametrize("hidden", [".git/config", "home/.git", "home/.gitattributes", ".nojekyll"])
def test_publish_refuses_a_hidden_file_in_the_site(site, gh_pages, hidden):
    path = site / hidden
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("hidden", encoding="utf-8")

    with pytest.raises(ValueError, match="hidden file"):
        publish_preview.publish(site, gh_pages, "pr-812", "main")

    assert (gh_pages / "pr-812" / "old.html").exists()


def test_publish_refuses_a_linked_site(site, gh_pages, tmp_path):
    linked_site = tmp_path / "linked-site"
    linked_site.symlink_to(site)

    with pytest.raises(ValueError, match="not a directory"):
        publish_preview.publish(linked_site, gh_pages, "pr-812", "main")


def test_publish_refuses_a_site_without_index(site, gh_pages):
    (site / "index.html").unlink()

    with pytest.raises(ValueError, match="index.html"):
        publish_preview.publish(site, gh_pages, "pr-812", "main")


def test_main_reports_a_refused_site(site, gh_pages):
    (site / "index.html").unlink()

    with pytest.raises(SystemExit, match="was not published"):
        publish_preview.main([str(site), str(gh_pages), "pr-812"])
