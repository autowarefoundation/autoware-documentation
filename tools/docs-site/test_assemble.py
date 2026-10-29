# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
# cspell:ignore javascripts charrefs

"""Unit tests for assemble.py and verify.py."""

import gzip
import hashlib
from html.parser import HTMLParser
import json
import urllib.parse

from assemble import MEDIA_DIRECTORY
from assemble import assemble
from assemble import media_name
import pytest
from verify import verify_version

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


def test_a_file_that_several_versions_contain_is_stored_once(bundles, tmp_path):
    output, summary = run(bundles, tmp_path)
    name = media_name_of(PNG, ".png")
    assert (output / MEDIA_DIRECTORY / name).read_bytes() == PNG
    assert not (output / "main" / "home" / "images" / "picture.png").exists()
    assert not (output / "1.9.0" / "home" / "images" / "picture.png").exists()
    for version in ("main", "1.9.0"):
        assert values(output / version / "home" / "index.html", "img", "src") == [
            f"../../{MEDIA_DIRECTORY}/{name}"
        ]
    assert summary["media_files"] == 1


def test_a_file_that_no_page_references_is_left_out(bundles, tmp_path):
    output, summary = run(bundles, tmp_path)
    assert not (output / MEDIA_DIRECTORY / media_name_of(OTHER_PNG, ".png")).exists()
    assert summary["unused_media_files"] == 2  # unused.png and the unreferenced logo


def test_source_maps_are_left_out_and_scripts_are_kept(bundles, tmp_path):
    output, _ = run(bundles, tmp_path)
    assert not (output / "main" / "assets" / "javascripts" / "bundle.js.map").exists()
    assert read(output / "main" / "assets" / "javascripts" / "bundle.js") == "console.log(1)"


def test_unquoted_attributes_srcset_and_css_are_rewritten(tmp_path):
    root = tmp_path / "bundles"
    write(root / "main" / "a" / "b.png", PNG)
    write(root / "main" / "a" / "c.svg", SVG)
    write(
        root / "main" / "a" / "index.html",
        page(
            "<img src=b.png alt=x>"
            '<img alt="x"src="b.png?v=1#frag">'
            '<img srcset="b.png 1x, c.svg 2x">'
            "<div style=\"background: url('c.svg')\"></div>"
            "<style>.x { background: url(b.png); }</style>"
        ),
    )
    output, _ = run(root, tmp_path)
    path = output / "main" / "a" / "index.html"
    png = f"../../{shared(PNG, '.png')}"
    svg = f"../../{shared(SVG, '.svg')}"
    assert values(path, "img", "src") == [png, f"{png}?v=1#frag"]
    assert values(path, "img", "srcset") == [f"{png} 1x, {svg} 2x"]
    assert values(path, "div", "style") == [f"background: url('{svg}')"]
    assert f"url({png})" in read(path)


def test_text_that_only_looks_like_an_attribute_is_kept(tmp_path):
    root = tmp_path / "bundles"
    write(root / "main" / "b.png", PNG)
    write(
        root / "main" / "index.html", page('<img src="b.png"><code>&lt;img src="b.png"&gt;</code>')
    )
    output, _ = run(root, tmp_path)
    assert '<img src="b.png">' in parse(output / "main" / "index.html").text


def test_external_and_page_references_are_kept(tmp_path):
    root = tmp_path / "bundles"
    write(root / "main" / "b.png", PNG)
    write(
        root / "main" / "index.html",
        page(
            '<img src="b.png"><a href="https://example.com/b.png">x</a>'
            '<a href="other/">y</a><a href="#top">z</a><img src="data:image/png;base64,AA">'
        ),
    )
    output, _ = run(root, tmp_path)
    path = output / "main" / "index.html"
    assert values(path, "a", "href") == ["https://example.com/b.png", "other/", "#top"]
    assert values(path, "img", "src")[1] == "data:image/png;base64,AA"


def test_drawio_links_point_at_the_shared_file(tmp_path):
    root = tmp_path / "bundles"
    for version in ("main", "1.5.0"):
        write(root / version / "d" / "diagram.drawio.svg", SVG)

    def link(address):
        return f"https://app.diagrams.net/?lightbox=1#U{urllib.parse.quote(address, safe='')}"

    write(
        root / "main" / "d" / "index.html",
        page(f'<a href="{link(SITE_URL + "/d/diagram.drawio.svg")}">open</a>'),
    )
    # Some releases point at the diagram of the default branch.
    write(
        root / "1.5.0" / "d" / "index.html",
        page(f'<a href="{link(SITE_URL + "main/d/diagram.drawio.svg")}">open</a>'),
    )
    output, _ = run(root, tmp_path)
    expected = link(f"{SITE_URL}{shared(SVG, '.svg')}")
    for version in ("main", "1.5.0"):
        assert values(output / version / "d" / "index.html", "a", "href") == [expected]
    assert (output / shared(SVG, ".svg")).exists()


def test_the_canonical_address_and_the_sitemap_get_the_version(bundles, tmp_path):
    write(
        bundles / "1.9.0" / "sitemap.xml",
        f"<urlset><url><loc>{SITE_URL}home/</loc></url></urlset>",
    )
    output, _ = run(bundles, tmp_path)
    assert values(output / "1.9.0" / "home" / "index.html", "link", "href") == [
        f"{SITE_URL}1.9.0/home/"
    ]
    sitemap = read(output / "1.9.0" / "sitemap.xml")
    assert f"<loc>{SITE_URL}1.9.0/home/</loc>" in sitemap
    with gzip.open(output / "1.9.0" / "sitemap.xml.gz", "rt", encoding="utf-8") as archive:
        assert archive.read() == sitemap


def test_a_canonical_address_with_the_version_is_kept(tmp_path):
    root = tmp_path / "bundles"
    write(root / "main" / "home" / "index.html", page("x", canonical="main/home/"))
    output, _ = run(root, tmp_path)
    assert values(output / "main" / "home" / "index.html", "link", "href") == [
        f"{SITE_URL}main/home/"
    ]


def test_the_root_files_are_written(tmp_path):
    root = tmp_path / "bundles"
    for version in ("1.8.0", "pr-12", "main", "1.10.0", "pr-3"):
        write(root / version / "index.html", page("x"))
    output, _ = run(root, tmp_path)
    versions = json.loads(read(output / "versions.json"))
    assert [entry["version"] for entry in versions] == ["main", "1.10.0", "1.8.0", "pr-12", "pr-3"]
    assert versions[0] == {"version": "main", "title": "main", "aliases": []}
    assert 'replace("main/"' in read(output / "index.html")
    not_found = read(output / "404.html")
    assert 'href="/main/"' in not_found
    assert RELEASES_URL in not_found


def test_the_default_version_must_have_a_bundle(tmp_path):
    root = tmp_path / "bundles"
    write(root / "1.9.0" / "index.html", page("x"))
    with pytest.raises(ValueError):
        run(root, tmp_path)


def test_minifying_keeps_the_content_of_pre_elements(tmp_path):
    root = tmp_path / "bundles"
    code = "line one\n    indented line\n\n  last"
    write(root / "main" / "index.html", page(f"<p>  a   b  </p>\n\n<pre><code>{code}</code></pre>"))
    output, _ = run(root, tmp_path)
    assert f"<pre><code>{code}</code></pre>" in read(output / "main" / "index.html")


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


def test_the_media_name_depends_only_on_the_content(tmp_path):
    first = tmp_path / "a.PNG"
    second = tmp_path / "other" / "b.png"
    write(first, PNG)
    write(second, PNG)
    assert media_name(first) == media_name(second) == media_name_of(PNG, ".png")
