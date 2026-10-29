# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0

"""Check that an assembled site shows the same pages as the bundles it came from.

For every HTML page of every version, the page in the site must have the same
text, the same content in each pre element, and references that lead to the
same files as the page in the bundle. A reference to an image can change its
address, but it must still lead to a file with the same content.

A reference that is broken in the bundle may stay broken in the site. A
reference that works in the bundle must work in the site.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
from html.parser import HTMLParser
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import sys
import urllib.parse

from assemble import DRAWIO_LINK
from assemble import is_external

REFERENCE_ATTRIBUTES = {"src", "href", "data", "poster"}


class PageReader(HTMLParser):
    """Collect the text, the pre content and the references of a page."""

    def __init__(self):
        super().__init__(convert_charrefs=True)
        self.text = []
        self.pre = []
        self.references = []
        self.pre_depth = 0
        self.skip_depth = 0

    def handle_starttag(self, tag, attrs):
        if tag in ("script", "style"):
            self.skip_depth += 1
        if tag == "pre":
            self.pre_depth += 1
            self.pre.append("")
        for name, value in attrs:
            if value is None:
                continue
            if name in REFERENCE_ATTRIBUTES:
                self.references.append((tag, name, value))
            elif name == "srcset":
                for candidate in value.split(","):
                    parts = candidate.split()
                    if parts:
                        self.references.append((tag, name, parts[0]))

    def handle_endtag(self, tag):
        if tag in ("script", "style") and self.skip_depth:
            self.skip_depth -= 1
        if tag == "pre" and self.pre_depth:
            self.pre_depth -= 1

    def handle_data(self, data):
        if self.skip_depth:
            return
        self.text.append(data)
        if self.pre_depth:
            self.pre[-1] += data


def read_page(path):
    reader = PageReader()
    reader.feed(path.read_text(encoding="utf-8"))
    # Whitespace between elements is what minifying removes, so it is ignored.
    text = re.sub(r"\s+", "", "".join(reader.text))
    return text, reader.pre, reader.references


class Resolver:
    """Turn a reference into what it leads to: a file content, a page or a link."""

    def __init__(self, root, site_url):
        self.root = Path(root)
        self.site_url = site_url
        self.hashes = {}

    def file_hash(self, path):
        if path not in self.hashes:
            self.hashes[path] = hashlib.sha256(path.read_bytes()).hexdigest()
        return self.hashes[path]

    def resolve(self, page, reference):
        """Return what a reference of a page leads to, relative to the version."""
        match = DRAWIO_LINK.match(reference)
        if match:
            address = urllib.parse.unquote(match.group(2))
            if address.startswith(self.site_url):
                relative = address[len(self.site_url) :].lstrip("/")
                path = self.root / relative
                if not path.is_file():
                    # An address without the version, as the bundles of a
                    # release write it. The file sits in the version.
                    path = self.root / PurePosixPath(page).parts[0] / relative
                return ("drawio", self.file_hash(path) if path.is_file() else "missing")
            return ("drawio", reference)
        if not reference or is_external(reference):
            return ("external", reference)
        path = reference.partition("#")[0].partition("?")[0]
        if not path:
            return ("external", reference)
        target = os.path.normpath(
            PurePosixPath(PurePosixPath(page).parent, urllib.parse.unquote(path)).as_posix()
        )
        full = self.root / target
        if full.is_file() and full.suffix.lower() != ".html":
            return ("file", self.file_hash(full))
        if full.is_dir() and (full / "index.html").is_file():
            target = (PurePosixPath(target) / "index.html").as_posix()
            full = full / "index.html"
        if full.is_file():
            # A page: compare its path inside the version.
            return ("page", PurePosixPath(*PurePosixPath(target).parts[1:]).as_posix())
        return ("missing", PurePosixPath(*PurePosixPath(target).parts[1:]).as_posix())


def verify_version(name, bundle, site, site_url):
    """Return the differences between a bundle and its version in the site."""
    problems = []
    bundle_resolver = Resolver(Path(bundle).parent, site_url)
    site_resolver = Resolver(site, site_url)
    for source in sorted(Path(bundle).rglob("*.html")):
        relative = source.relative_to(bundle).as_posix()
        page = f"{name}/{relative}"
        assembled = Path(site) / page
        if not assembled.is_file():
            problems.append(f"{page}: missing from the site")
            continue
        text_a, pre_a, references_a = read_page(source)
        text_b, pre_b, references_b = read_page(assembled)
        if text_a != text_b:
            problems.append(f"{page}: the text differs")
        if pre_a != pre_b:
            problems.append(f"{page}: the content of a pre element differs")
        if len(references_a) != len(references_b):
            problems.append(f"{page}: {len(references_a)} references became {len(references_b)}")
            continue
        for (tag, attribute, before), (_, _, after) in zip(references_a, references_b):
            was = bundle_resolver.resolve(page, before)
            now = site_resolver.resolve(page, after)
            if was == now:
                continue
            if was[0] == "missing" and now[0] == "missing":
                continue  # Broken before, and still broken.
            if attribute == "href" and tag == "link" and was[0] == "external":
                continue  # The canonical address gets the name of the version.
            problems.append(f"{page}: <{tag} {attribute}={before!r}> led to {was}, now {now}")
    return problems


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--bundles", required=True, help="directory with one built site per version"
    )
    parser.add_argument("--site", required=True, help="the assembled site")
    parser.add_argument("--site-url", required=True, help="address of the site root")
    args = parser.parse_args()
    site_url = args.site_url if args.site_url.endswith("/") else args.site_url + "/"

    bundles = [bundle for bundle in sorted(Path(args.bundles).iterdir()) if bundle.is_dir()]
    # The versions are independent, so they are checked at the same time.
    with ProcessPoolExecutor() as pool:
        results = pool.map(
            verify_version,
            [bundle.name for bundle in bundles],
            bundles,
            [Path(args.site)] * len(bundles),
            [site_url] * len(bundles),
        )
    failed = False
    for bundle, problems in zip(bundles, results):
        pages = len(list(bundle.rglob("*.html")))
        print(f"{bundle.name}: {pages} pages, {len(problems)} problems")
        for problem in problems[:20]:
            print(f"  {problem}")
        failed = failed or bool(problems)
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
