# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
# cspell:ignore javascripts

"""The fixture that the tests of assemble.py and verify.py share."""

from helpers import OTHER_PNG
from helpers import PNG
from helpers import SVG
from helpers import page
from helpers import write
import pytest


@pytest.fixture
def bundles(tmp_path):
    root = tmp_path / "bundles"
    for version in ("main", "1.9.0"):
        write(root / version / "home" / "images" / "picture.png", PNG)
        write(
            root / version / "home" / "index.html",
            page('<p>Hello</p><img src="images/picture.png" alt="picture">'),
        )
    write(root / "main" / "assets" / "images" / "logo.svg", SVG)
    write(root / "main" / "unused.png", OTHER_PNG)
    write(root / "main" / "assets" / "javascripts" / "bundle.js.map", "{}")
    write(root / "main" / "assets" / "javascripts" / "bundle.js", "console.log(1)")
    return root
