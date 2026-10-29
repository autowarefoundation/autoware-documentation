# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Unit tests for verify.py."""

import json
import sys
import urllib.parse

from helpers import OTHER_PNG
from helpers import PNG
from helpers import SITE_URL
from helpers import SVG
from helpers import page
from helpers import read
from helpers import run
from helpers import shared
from helpers import write
import pytest
import verify
from verify import Resolver
from verify import TOOL_FILES
from verify import environment_digest
from verify import main as verify_main
from verify import verify_all
from verify import verify_version


def test_verify_accepts_the_assembled_site(bundles, tmp_path):
    output, _ = run(bundles, tmp_path)
    for version in ("main", "1.9.0"):
        assert verify_version(version, bundles / version, output, SITE_URL) == []


def test_verify_finds_a_missing_image(bundles, tmp_path):
    output, _ = run(bundles, tmp_path)
    (output / shared(PNG, ".png")).unlink()
    problems = verify_version("main", bundles / "main", output, SITE_URL)
    assert len(problems) == 1
    assert "picture.png" in problems[0]


def test_verify_finds_changed_text(bundles, tmp_path):
    output, _ = run(bundles, tmp_path)
    page_path = output / "main" / "home" / "index.html"
    page_path.write_text(read(page_path).replace("Hello", "Goodbye"), encoding="utf-8")
    assert verify_version("main", bundles / "main", output, SITE_URL) == [
        "main/home/index.html: the text differs"
    ]


def test_verify_skips_a_version_that_passed_and_did_not_change(bundles, tmp_path, capsys):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "state" / "verified.json"

    assert verify_all(bundles, output, SITE_URL, state)
    assert "main: 1 pages, 0 problems" in capsys.readouterr().out

    assert verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "main: unchanged since it passed" in out
    assert "1.9.0: unchanged since it passed" in out

    # A changed page is checked again. It fails, so it leaves the state.
    page_path = output / "main" / "home" / "index.html"
    page_path.write_text(read(page_path).replace("Hello", "Goodbye"), encoding="utf-8")
    assert not verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "main: 1 pages, 1 problems" in out
    assert "1.9.0: unchanged since it passed" in out
    assert list(json.loads(read(state))) == ["1.9.0"]


def test_verify_checks_a_version_again_when_its_bundle_changes(bundles, tmp_path, capsys):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    write(bundles / "1.9.0" / "home" / "extra.txt", "a new file of the release")
    capsys.readouterr()
    assert verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "1.9.0: 1 pages, 0 problems" in out
    assert "main: unchanged since it passed" in out


def test_a_new_version_does_not_check_the_other_versions_again(bundles, tmp_path, capsys):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    write(bundles / "pr-7" / "home" / "index.html", page("<p>Preview</p>"))
    output, _ = run(bundles, tmp_path)
    capsys.readouterr()
    assert verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "main: unchanged since it passed" in out
    assert "1.9.0: unchanged since it passed" in out
    assert "pr-7: 1 pages, 0 problems" in out
    assert sorted(json.loads(read(state))) == ["1.9.0", "main", "pr-7"]


def test_a_change_in_another_version_checks_a_version_that_points_there(bundles, tmp_path, capsys):
    # A release page points into main. The file does not exist yet, so the
    # reference is broken in the built site and in the assembled site alike.
    write(
        bundles / "1.9.0" / "home" / "index.html",
        page('<p>Hello</p><img src="../../main/home/new.png">'),
    )
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)

    # main gets the file. The assembled site moves it to the shared files, so
    # the reference of the release breaks, and the check must see that.
    write(bundles / "main" / "home" / "new.png", OTHER_PNG)
    write(bundles / "main" / "home" / "index.html", page('<p>Hello</p><img src="new.png">'))
    output, _ = run(bundles, tmp_path)
    capsys.readouterr()
    assert not verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "1.9.0: unchanged since it passed" not in out
    assert "new.png" in out
    assert "1.9.0" not in json.loads(read(state))


def test_the_command_line_keeps_the_state(bundles, tmp_path, monkeypatch, capsys):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "state" / "verified.json"
    arguments = ["verify.py", "--bundles", str(bundles), "--site", str(output)]
    arguments += ["--site-url", SITE_URL, "--state", str(state)]
    monkeypatch.setattr(sys, "argv", arguments)
    assert verify_main() == 0
    assert sorted(json.loads(read(state))) == ["1.9.0", "main"]
    capsys.readouterr()
    assert verify_main() == 0
    assert capsys.readouterr().out.count("unchanged since it passed") == 2


@pytest.mark.parametrize("content", ["", "not json", "[1, 2]"])
def test_an_unreadable_state_checks_every_version(bundles, tmp_path, capsys, content):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    write(state, content)
    assert verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "unchanged since it passed" not in out
    assert sorted(json.loads(read(state))) == ["1.9.0", "main"]


def test_the_environment_covers_the_tools_the_packages_python_and_the_address(
    tmp_path, monkeypatch
):
    tools = tmp_path / "tools"
    for tool in TOOL_FILES:
        write(tools / tool, f"# {tool}")
    first = environment_digest(SITE_URL, tools)
    assert environment_digest(SITE_URL, tools) == first
    assert environment_digest("https://example.com/", tools) != first
    write(tools / "requirements.txt", "minify-html==9.9.9")
    second = environment_digest(SITE_URL, tools)
    assert second != first
    write(tools / "assemble.py", "# changed")
    third = environment_digest(SITE_URL, tools)
    assert third != second
    monkeypatch.setattr(sys, "version", "9.9.9 (another Python)")
    assert environment_digest(SITE_URL, tools) != third


def test_a_change_of_the_environment_checks_every_version_again(
    bundles, tmp_path, capsys, monkeypatch
):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    capsys.readouterr()
    monkeypatch.setattr(sys, "version", "9.9.9 (another Python)")
    assert verify_all(bundles, output, SITE_URL, state)
    assert "unchanged since it passed" not in capsys.readouterr().out


def test_a_missing_shared_file_checks_the_version_again(bundles, tmp_path, capsys):
    # The shared files sit outside the directory of the version, so only the
    # dependencies on the site side notice that one is gone.
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    (output / shared(PNG, ".png")).unlink()
    capsys.readouterr()
    assert not verify_all(bundles, output, SITE_URL, state)
    assert "unchanged since it passed" not in capsys.readouterr().out


def test_a_changed_file_of_another_version_checks_a_version_that_points_there(
    bundles, tmp_path, capsys
):
    # A release page links to a file of main that stays in the directory of main.
    write(bundles / "main" / "data.txt", "first")
    write(
        bundles / "1.9.0" / "home" / "index.html",
        page('<p>Hello</p><img src="images/picture.png"><a href="../../main/data.txt">data</a>'),
    )
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)

    # Only the content changes, and only in the site, so the release must be
    # checked again, and the check must find the difference.
    write(output / "main" / "data.txt", "second")
    capsys.readouterr()
    assert not verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "1.9.0: unchanged since it passed" not in out
    assert "data.txt" in out


def drawio_link(address):
    return f"https://app.diagrams.net/?lightbox=1#U{urllib.parse.quote(address, safe='')}"


@pytest.mark.parametrize(
    "address",
    [f"{SITE_URL}main/d/diagram.drawio.svg", f"{SITE_URL}1.9.0/../main/d/diagram.drawio.svg"],
)
def test_a_diagram_of_another_version_is_tracked(bundles, tmp_path, capsys, address):
    # Releases link to the diagram of main. The second address only reaches it
    # through "..", which must not hide that the path is outside the release.
    write(bundles / "main" / "d" / "diagram.drawio.svg", SVG)
    write(
        bundles / "1.9.0" / "d" / "index.html", page(f'<a href="{drawio_link(address)}">open</a>')
    )
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)

    # The diagram leaves the built site of main, but not the assembled site.
    (bundles / "main" / "d" / "diagram.drawio.svg").unlink()
    capsys.readouterr()
    assert not verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "1.9.0: unchanged since it passed" not in out
    assert "1.9.0/d/index.html" in out


@pytest.mark.parametrize(
    "dependencies",
    [
        None,
        "not a mapping",
        {"bundles": {}},
        {"bundles": {}, "site": []},
    ],
)
def test_a_broken_entry_with_the_right_fingerprint_checks_the_version(
    bundles, tmp_path, capsys, dependencies
):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    entries = json.loads(read(state))
    entries["main"]["dependencies"] = dependencies
    write(state, json.dumps(entries))
    capsys.readouterr()
    assert verify_all(bundles, output, SITE_URL, state)
    out = capsys.readouterr().out
    assert "main: 1 pages, 0 problems" in out
    assert "1.9.0: unchanged since it passed" in out


def test_a_deeply_nested_state_checks_every_version(bundles, tmp_path, capsys):
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    write(state, "[" * 200000)
    assert verify_all(bundles, output, SITE_URL, state)
    assert "unchanged since it passed" not in capsys.readouterr().out


def test_an_error_while_reading_an_entry_checks_the_version(bundles, tmp_path, monkeypatch):
    # For example a path in a forged entry that the file system refuses.
    output, _ = run(bundles, tmp_path)
    state = tmp_path / "verified.json"
    assert verify_all(bundles, output, SITE_URL, state)
    entry = json.loads(read(state))["main"]

    def refuse(dependencies, bundles_root, site):
        raise OSError("refused")

    # Only the test whether the entry still holds fails. The full check runs.
    monkeypatch.setattr(verify, "dependencies_unchanged", refuse)
    environment = environment_digest(SITE_URL)
    _, problems = verify.check_version(
        "main", bundles / "main", output, SITE_URL, environment, entry
    )
    assert problems == []


def test_a_reference_to_the_site_root_is_a_dependency(tmp_path):
    # A link from the root page of a version to "../" leads to the site root,
    # which lies outside every version.
    write(tmp_path / "index.html", "<p>root</p>")
    write(tmp_path / "main" / "index.html", "<p>main</p>")
    resolver = Resolver(tmp_path, SITE_URL)
    assert resolver.resolve("main/index.html", "../") == ("page", ".")
    assert resolver.dependencies == {".": "page"}
