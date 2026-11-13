# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
# cspell:ignore charrefs

"""Shared helpers for the tests of assemble.py and verify.py."""

import hashlib
from html.parser import HTMLParser

from assemble import MEDIA_DIRECTORY
from assemble import assemble

SITE_URL = "https://example.org/docs/"
RELEASES_URL = "https://example.org/releases"

PNG = b"\x89PNG\r\n\x1a\nthe same picture"
OTHER_PNG = b"\x89PNG\r\n\x1a\nanother picture"
SVG = b'<svg xmlns="http://www.w3.org/2000/svg"></svg>'


def write(path, content):
    path.parent.mkdir(parents=True, exist_ok=True)
    if isinstance(content, bytes):
        path.write_bytes(content)
    else:
        path.write_text(content, encoding="utf-8")


def page(body, canonical="home/"):
    return (
        "<!doctype html><html><head>"
        f'<link rel="canonical" href="{SITE_URL}{canonical}">'
        "</head><body>"
        f"{body}"
        "</body></html>"
    )


def shared(content, suffix):
    return f"{MEDIA_DIRECTORY}/{media_name_of(content, suffix)}"


def media_name_of(content, suffix):
    return hashlib.sha256(content).hexdigest()[:20] + suffix


def run(bundles, tmp_path, default_version="main"):
    output = tmp_path / "site"
    summary = assemble(bundles, output, SITE_URL, default_version, RELEASES_URL)
    return output, summary


def read(path):
    return path.read_text(encoding="utf-8")


class Reader(HTMLParser):
    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.attributes = []
        self.text = ""

    def handle_starttag(self, tag, attrs):
        self.attributes.extend((tag, name, value) for name, value in attrs)

    def handle_data(self, data):
        self.text += data


def parse(path):
    """Return the attributes and the text of a page.

    The pages are minified, so the quotes and the order of the attributes can
    change. Reading the page as HTML ignores both.
    """
    reader = Reader()
    reader.feed(read(path))
    return reader


def values(path, tag, attribute):
    return [v for t, a, v in parse(path).attributes if t == tag and a == attribute]
