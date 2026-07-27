# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for delete_old_version_docs.py."""

import json
import subprocess

import delete_old_version_docs
import pytest

# What mike reports for the gh-pages branch, in the order mike itself sorts it:
# the names that do not start with a digit first, then the releases.
DEPLOYED_ON_GH_PAGES = ["pr-812", "main", "1.8.0", "1.7.1", "1.7.0", "1.6.0", "1.5.0"]

LIST_COMMAND = ["mike", "list", "--json"]


@pytest.fixture(name="run_mike")
def fixture_run_mike(monkeypatch):
    """Replace mike with a fake that answers `mike list` and records every call.

    The tests use this instead of replacing the two functions that call mike, so
    that the command lines handed to mike are covered as well.
    """

    def install(deployed_versions=None):
        if deployed_versions is None:
            deployed_versions = DEPLOYED_ON_GH_PAGES
        commands = []

        def fake_run(command, **kwargs):
            commands.append(command)
            assert kwargs.get("check") is True, "a failing mike must fail the workflow"
            if command != LIST_COMMAND:
                return subprocess.CompletedProcess(command, 0)
            # The title of a version can differ from the version itself.
            listing = [
                {"version": version, "title": f"Release {version}", "aliases": []}
                for version in deployed_versions
            ]
            return subprocess.CompletedProcess(command, 0, stdout=json.dumps(listing))

        monkeypatch.setattr(subprocess, "run", fake_run)
        return commands

    return install


def test_main_asks_mike_to_delete_exactly_the_outdated_releases(run_mike):
    commands = run_mike()

    assert delete_old_version_docs.main([]) == 0
    assert commands == [LIST_COMMAND, ["mike", "delete", "--push", "1.7.0", "1.5.0"]]


def test_main_runs_no_deletion_on_a_dry_run(run_mike):
    commands = run_mike()

    assert delete_old_version_docs.main(["--dry-run"]) == 0
    assert commands == [LIST_COMMAND]


def test_main_runs_no_deletion_when_nothing_is_outdated(run_mike, capsys):
    commands = run_mike(["main", "1.8.0"])

    assert delete_old_version_docs.main([]) == 0
    assert commands == [LIST_COMMAND]

    printed = capsys.readouterr().out
    assert "Outdated releases:\n  (none)\n" in printed
    assert "Nothing to delete." in printed


def test_main_accepts_another_number_of_lines(run_mike):
    commands = run_mike()

    assert delete_old_version_docs.main(["--keep-lines", "1"]) == 0
    assert commands == [
        LIST_COMMAND,
        ["mike", "delete", "--push", "1.7.1", "1.7.0", "1.6.0", "1.5.0"],
    ]


def test_main_reports_what_it_did(run_mike, capsys):
    run_mike()

    delete_old_version_docs.main(["--dry-run"])

    printed = capsys.readouterr().out
    assert "Kept releases:\n  1.8.0\n  1.7.1\n  1.6.0\n" in printed
    assert "Outdated releases:\n  1.7.0\n  1.5.0\n" in printed
    assert "Dry run, nothing was deleted." in printed


@pytest.mark.parametrize("keep_lines", ["0", "-1", "three", ""])
def test_main_refuses_a_number_of_lines_that_is_not_a_positive_number(run_mike, keep_lines):
    run_mike()

    with pytest.raises(SystemExit) as refusal:
        delete_old_version_docs.main(["--keep-lines", keep_lines])

    assert refusal.value.code == 2  # What argparse exits with on a bad argument.
