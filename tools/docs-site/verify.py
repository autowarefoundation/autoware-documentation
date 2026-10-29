# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
# cspell:ignore charrefs

"""Check that an assembled site shows the same pages as the bundles it came from.

For every HTML page of every version, the page in the site must have the same
text, the same content in each pre element, and references that lead to the
same files as the page in the bundle. A reference to an image can change its
address, but it must still lead to a file with the same content.

A reference that is broken in the bundle may stay broken in the site. A
reference that works in the bundle must work in the site.

A version that passed before is not checked again while nothing that its check
reads changed: its built site, its assembled pages, the paths outside the
version that its references lead to, the code and packages of the tools, the
Python version, and the site address. Releases never change, so usually only
the default branch and the previews are checked.
"""

import argparse
from concurrent.futures import ProcessPoolExecutor
import hashlib
from html.parser import HTMLParser
import json
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import sys
import urllib.parse

from assemble import DRAWIO_LINK
from assemble import MEDIA_DIRECTORY
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


def describe(root, target):
    """Return what a path holds, as far as resolving a reference can see it.

    A shared file counts only by its existence, because assemble.py names it
    after a hash of its content. That holds as long as nothing else writes to
    the shared files. assemble.py is part of environment_digest, so a change to
    how it names them checks every version again.
    """
    full = Path(root) / target
    if full.is_file():
        if PurePosixPath(target).parts[:1] == (MEDIA_DIRECTORY,):
            return "file"
        return "file:" + hashlib.sha256(full.read_bytes()).hexdigest()
    if full.is_dir():
        return "page" if (full / "index.html").is_file() else "directory"
    return "missing"


class Resolver:
    """Turn a reference into what it leads to: a file content, a page or a link.

    The paths that a reference leads to outside the version of its page are
    kept in `dependencies`, with what they held. The check of the version
    depends on them, although they belong to another version or to the shared
    files.
    """

    def __init__(self, root, site_url):
        self.root = Path(root)
        self.site_url = site_url
        self.hashes = {}
        self.dependencies = {}

    def note(self, page, target):
        """Keep a path that a reference of a page leads to, if it is outside its version."""
        if target in self.dependencies:
            return
        if PurePosixPath(target).parts[:1] != PurePosixPath(page).parts[:1]:
            self.dependencies[target] = describe(self.root, target)

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
                # Normalized like every other path, so that ".." cannot hide
                # a path outside the version.
                relative = os.path.normpath(address[len(self.site_url) :].lstrip("/"))
                in_version = os.path.normpath(f"{PurePosixPath(page).parts[0]}/{relative}")
                self.note(page, relative)
                self.note(page, in_version)
                path = self.root / relative
                if not path.is_file():
                    # An address without the version, as the bundles of a
                    # release write it. The file sits in the version.
                    path = self.root / in_version
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
        self.note(page, target)
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
    return verify_version_and_dependencies(name, bundle, site, site_url)[0]


def verify_version_and_dependencies(name, bundle, site, site_url):
    """Return the differences, and the paths outside the version that the check read.

    The paths are kept separately for the bundles and for the site, each with
    what it held, as `describe` returns it.
    """
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
    dependencies = {"bundles": bundle_resolver.dependencies, "site": site_resolver.dependencies}
    return problems, dependencies


# The code and the packages that decide how a version is assembled and checked.
# A change to them checks every version again.
TOOL_FILES = ["assemble.py", "verify.py", "versions.py", "requirements.txt"]


def environment_digest(site_url, directory=None):
    """Return a hash of what the check of every version depends on.

    That is the code of the tools, their packages, the Python version, and the
    address of the site.
    """
    directory = Path(directory) if directory else Path(__file__).parent
    digest = hashlib.sha256()
    for tool in TOOL_FILES:
        digest.update(f"{tool}\0".encode())
        digest.update((directory / tool).read_bytes())
    digest.update(f"{sys.version}\0{site_url}".encode())
    return digest.hexdigest()


def fingerprint(name, bundle, site, environment):
    """Return a hash of everything that the check of a version depends on.

    That is the environment from environment_digest, the built site of the
    version, and its assembled pages. The paths outside the version that the
    check read are compared separately, by `dependencies_unchanged`.
    """
    digest = hashlib.sha256(environment.encode())
    roots = [(Path(bundle), "bundle"), (Path(site) / name, "site")]
    for root, label in roots:
        for path in sorted(root.rglob("*")):
            if path.is_file():
                digest.update(f"{label}/{path.relative_to(root).as_posix()}\0".encode())
                digest.update(hashlib.sha256(path.read_bytes()).digest())
    return digest.hexdigest()


def dependencies_unchanged(dependencies, bundles_root, site):
    """Tell whether every path outside a version still holds what its check saw."""
    if not isinstance(dependencies, dict):
        return False
    for label, root in (("bundles", bundles_root), ("site", site)):
        paths = dependencies.get(label)
        if not isinstance(paths, dict):
            return False
        for target, held in paths.items():
            if describe(root, target) != held:
                return False
    return True


def check_version(name, bundle, site, site_url, environment, verified):
    """Check one version, unless the same version passed the check before.

    `verified` is the entry of the version in the state file. Return the new
    entry, and the problems, or None if the check was skipped.
    """
    current = fingerprint(name, bundle, site, environment)
    try:
        unchanged = (
            isinstance(verified, dict)
            and verified.get("fingerprint") == current
            and dependencies_unchanged(verified.get("dependencies"), Path(bundle).parent, site)
        )
    except Exception:  # noqa: BLE001 - a broken entry only costs a full check.
        unchanged = False
    if unchanged:
        return verified, None
    problems, dependencies = verify_version_and_dependencies(name, bundle, site, site_url)
    return {"fingerprint": current, "dependencies": dependencies}, problems


def read_state(state):
    """Return the versions that passed, from the state file.

    A missing or unreadable file means that no version passed yet, so every
    version is checked.
    """
    if not state or not state.exists():
        return {}
    try:
        verified = json.loads(state.read_text(encoding="utf-8"))
    except Exception:  # noqa: BLE001 - any broken state only costs a full check.
        return {}
    return verified if isinstance(verified, dict) else {}


def verify_all(bundles, site, site_url, state=None):
    """Check every version of the site and print the result.

    `state` is a JSON file that keeps, for each version that passed, its
    fingerprint and the paths outside the version that its check read. A
    version is skipped if both are unchanged, because nothing it depends on
    changed. Return True if every version passed.
    """
    site_url = site_url if site_url.endswith("/") else site_url + "/"
    state = Path(state) if state else None
    verified = read_state(state)
    bundles = [bundle for bundle in sorted(Path(bundles).iterdir()) if bundle.is_dir()]
    environment = environment_digest(site_url)
    # The versions are independent, so they are checked at the same time.
    with ProcessPoolExecutor() as pool:
        results = list(
            pool.map(
                check_version,
                [bundle.name for bundle in bundles],
                bundles,
                [Path(site)] * len(bundles),
                [site_url] * len(bundles),
                [environment] * len(bundles),
                [verified.get(bundle.name) for bundle in bundles],
            )
        )
    passed = {}
    failed = False
    for bundle, (entry, problems) in zip(bundles, results):
        if problems is None:
            print(f"{bundle.name}: unchanged since it passed, not checked again")
            passed[bundle.name] = entry
            continue
        pages = len(list(bundle.rglob("*.html")))
        print(f"{bundle.name}: {pages} pages, {len(problems)} problems")
        for problem in problems[:20]:
            print(f"  {problem}")
        if problems:
            failed = True
        else:
            passed[bundle.name] = entry
    if state:
        state.parent.mkdir(parents=True, exist_ok=True)
        state.write_text(json.dumps(passed, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    return not failed


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--bundles", required=True, help="directory with one built site per version"
    )
    parser.add_argument("--site", required=True, help="the assembled site")
    parser.add_argument("--site-url", required=True, help="address of the site root")
    parser.add_argument(
        "--state",
        help="JSON file that keeps the versions that passed, so that they are not checked again",
    )
    args = parser.parse_args()
    return 0 if verify_all(args.bundles, args.site, args.site_url, args.state) else 1


if __name__ == "__main__":
    sys.exit(main())
