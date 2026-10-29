# Copyright 2026 The Autoware Contributors
# SPDX-License-Identifier: Apache-2.0
# cspell:ignore noscript

"""Assemble the documentation site from one built site per version.

Every version of the documentation is a complete built site: the default
branch, every release, and every pull request preview. Most of their images
are the same, so copying each site as it is wastes most of the space that
GitHub Pages allows.

This script puts every version in its own directory, as before, but moves each
image, diagram and PDF into one shared directory named after a hash of its
content. A file that several versions contain is stored once. The references
in the HTML and CSS of each version are rewritten to point there.

It also minifies the HTML, gives each page the canonical address of its
version, and writes the files that the site root needs: versions.json for the
version selector, a redirect to the default version, and the 404 page.
"""

import argparse
import gzip
import hashlib
import json
import os
from pathlib import Path
from pathlib import PurePosixPath
import re
import shutil
import sys
import urllib.parse

import minify_html
from versions import reorder_entries

# The directory at the site root that holds the shared files.
MEDIA_DIRECTORY = "_media"

# The files that are shared between the versions.
MEDIA_SUFFIXES = {
    ".avif",
    ".gif",
    ".ico",
    ".jpeg",
    ".jpg",
    ".mp4",
    ".pdf",
    ".png",
    ".svg",
    ".webm",
    ".webp",
}

# Source maps only help the developers of the theme, so they are left out.
DROPPED_SUFFIXES = {".map"}

# The number of hexadecimal digits of the SHA-256 hash in a shared file name.
HASH_LENGTH = 20

# An opening tag, so that only real attributes are rewritten and never text
# that only looks like one, for example inside a code block.
TAG = re.compile(r"<[a-zA-Z][^<>]*>")
# An attribute follows whitespace, or the quote of the attribute before it when a
# minifier removed the space. Its value can be quoted or, after minifying, not.
ATTRIBUTE = re.compile(
    r"""((?<=[\s"'])(?:src|href|data|poster)\s*=\s*)(?:"([^"]*)"|'([^']*)'|([^\s"'=<>`]+))""",
    re.I,
)
SRCSET = re.compile(r"""((?<=[\s"'])srcset\s*=\s*)(?:"([^"]*)"|'([^']*)'|([^\s"'=<>`]+))""", re.I)
STYLE_ELEMENT = re.compile(r"(<style\b[^>]*>)(.*?)(</style>)", re.I | re.S)
CSS_URL = re.compile(r"""url\(\s*(["']?)([^"')]+)\1\s*\)""")
CANONICAL = re.compile(r"""<link\b[^>]*\brel=["']?canonical\b[^>]*>""", re.I)
HREF = re.compile(r"""(\bhref=)(["']?)([^"'\s>]+)\2""", re.I)
SITEMAP_LOCATION = re.compile(r"(<loc>)([^<]*)(</loc>)")

# A reference to a shared file, plain or percent-encoded as in the links to
# diagrams.net.
USED_MEDIA = re.compile(rf"{MEDIA_DIRECTORY}(?:/|%2F)([0-9a-f]{{{HASH_LENGTH}}}\.\w+)")

# The "open in diagrams.net" links of the drawio filter in main.py carry the
# address of the diagram, encoded, after "#U".
DRAWIO_LINK = re.compile(r"^(https://app\.diagrams\.net/[^#]*#U)(.+)$")

ROOT_REDIRECT = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>Redirecting</title>
    <noscript><meta http-equiv="refresh" content="1; url={version}/"></noscript>
    <script>window.location.replace("{version}/" + window.location.hash);</script>
  </head>
  <body>Redirecting to <a href="{version}/">{version}</a>...</body>
</html>
"""

NOT_FOUND_PAGE = """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <title>Page not found - Autoware Documentation</title>
    <style>
      :root {{ color-scheme: light dark; }}
      body {{ font-family: system-ui, sans-serif; line-height: 1.6; margin: 0 auto; max-width: 40rem; padding: 4rem 1.5rem; }}
      h1 {{ font-size: 1.6rem; }}
      li {{ margin-bottom: 0.5rem; }}
    </style>
  </head>
  <body>
    <h1>Page not found</h1>
    <p>The page you asked for does not exist. It may have belonged to a version whose documentation is no longer published on this site.</p>
    <ul>
      <li><a href="/{version}/">Read the current Autoware documentation</a></li>
      <li><a href="{releases_url}">Download the documentation of a release</a></li>
    </ul>
  </body>
</html>
"""


class Version:
    """One version of the site and the shared files it uses."""

    def __init__(self, name, source):
        self.name = name
        self.source = Path(source)
        # The path of each shared file in the version, mapped to its shared name.
        self.media = {}


def is_external(reference):
    """Tell whether a reference leaves the site or stays inside the page."""
    return reference.startswith(("#", "/", "data:", "mailto:", "tel:", "javascript:")) or bool(
        re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", reference)
    )


def media_name(path):
    """Return the shared name of a file: a hash of its content and its suffix."""
    digest = hashlib.sha256(path.read_bytes()).hexdigest()[:HASH_LENGTH]
    return digest + path.suffix.lower()


def relative_to_root(page):
    """Return the relative path from the directory of a page to the site root.

    A page path starts with its version, so "main/home/index.html" gives
    "../../".
    """
    depth = len(PurePosixPath(page).parts) - 1
    return "../" * depth


def rewrite_reference(reference, page, version):
    """Return the reference with a shared file replaced by its shared address.

    The page is the path of the page inside the site, for example
    "main/home/index.html". A reference that does not point at a shared file of
    the version is returned unchanged.
    """
    if not reference or is_external(reference):
        return reference
    path, separator, fragment = reference.partition("#")
    path, query_separator, query = path.partition("?")
    page_directory = PurePosixPath(page).parent
    target = os.path.normpath(PurePosixPath(page_directory, urllib.parse.unquote(path)).as_posix())
    prefix = version.name + "/"
    if not target.startswith(prefix):
        return reference
    shared = version.media.get(target[len(prefix) :])
    if shared is None:
        return reference
    rewritten = f"{relative_to_root(page)}{MEDIA_DIRECTORY}/{shared}"
    if query_separator:
        rewritten += "?" + query
    if separator:
        rewritten += "#" + fragment
    return rewritten


def rewrite_drawio_link(reference, version, versions, site_url):
    """Point the diagram of an "open in diagrams.net" link at its shared file.

    The diagram address is the site address followed by the path of the file,
    with or without the name of a version in between. Some releases point at
    the diagram of the default branch, so the version in the address is used
    when there is one.
    """
    match = DRAWIO_LINK.match(reference)
    if not match:
        return reference
    address = urllib.parse.unquote(match.group(2))
    if not address.startswith(site_url):
        return reference
    # Older builds of main.py wrote a double slash after the site address.
    path = address[len(site_url) :].lstrip("/")
    owner = version
    first, _, rest = path.partition("/")
    if first in versions:
        owner, path = versions[first], rest
    shared = owner.media.get(path)
    if shared is None:
        return reference
    shared_address = f"{site_url}{MEDIA_DIRECTORY}/{shared}"
    return match.group(1) + urllib.parse.quote(shared_address, safe="")


def rewrite_srcset(value, page, version):
    """Rewrite each candidate of a srcset attribute."""
    candidates = []
    for candidate in value.split(","):
        parts = candidate.strip().split(maxsplit=1)
        if not parts:
            continue
        parts[0] = rewrite_reference(parts[0], page, version)
        candidates.append(" ".join(parts))
    return ", ".join(candidates)


def rewrite_css(text, page, version):
    """Rewrite the url() references of CSS text."""

    def replace(match):
        quote, reference = match.group(1), match.group(2)
        return f"url({quote}{rewrite_reference(reference.strip(), page, version)}{quote})"

    return CSS_URL.sub(replace, text)


def canonical_address(address, version, site_url):
    """Return the address of a page with the name of its version in it."""
    if not address.startswith(site_url):
        return address
    path = address[len(site_url) :]
    if path.startswith(version.name + "/") or path == version.name:
        return address
    return f"{site_url}{version.name}/{path}"


def attribute_value(match):
    """Return the value of an attribute match and the quote around it."""
    for group, quote in ((2, '"'), (3, "'"), (4, "")):
        if match.group(group) is not None:
            return match.group(group), quote
    return "", '"'


def rewrite_html(text, page, version, versions, site_url):
    """Rewrite the references and the canonical address of an HTML page."""

    def replace_attribute(match):
        value, quote = attribute_value(match)
        rewritten = rewrite_drawio_link(value, version, versions, site_url)
        if rewritten == value:
            rewritten = rewrite_reference(value, page, version)
        if not quote and rewritten != value:
            quote = '"'
        return f"{match.group(1)}{quote}{rewritten}{quote}"

    def replace_srcset(match):
        value, quote = attribute_value(match)
        rewritten = rewrite_srcset(value, page, version)
        if not quote and rewritten != value:
            quote = '"'
        return f"{match.group(1)}{quote}{rewritten}{quote}"

    def replace_tag(match):
        tag = ATTRIBUTE.sub(replace_attribute, match.group(0))
        tag = SRCSET.sub(replace_srcset, tag)
        if "url(" in tag:
            tag = rewrite_css(tag, page, version)
        return tag

    def replace_style(match):
        return match.group(1) + rewrite_css(match.group(2), page, version) + match.group(3)

    def replace_canonical(match):
        def replace_href(href):
            address = canonical_address(href.group(3), version, site_url)
            return f"{href.group(1)}{href.group(2)}{address}{href.group(2)}"

        return HREF.sub(replace_href, match.group(0))

    text = TAG.sub(replace_tag, text)
    text = STYLE_ELEMENT.sub(replace_style, text)
    return CANONICAL.sub(replace_canonical, text)


def minify(text):
    """Remove the whitespace and the comments that a browser does not need.

    Inline scripts and styles are kept as they are, and so is the content of
    pre elements.
    """
    return minify_html.minify(
        text,
        keep_closing_tags=True,
        keep_html_and_head_opening_tags=True,
        minify_css=False,
        minify_js=False,
    )


def collect_media(version, media_directory, stored):
    """Find the shared files of a version and store each new one once.

    `stored` maps each shared name to the size of the file, for all versions.
    """
    for path in sorted(version.source.rglob("*")):
        if not path.is_file() or path.suffix.lower() not in MEDIA_SUFFIXES:
            continue
        name = media_name(path)
        version.media[path.relative_to(version.source).as_posix()] = name
        if name not in stored:
            shutil.copyfile(path, media_directory / name)
            stored[name] = path.stat().st_size


def copy_version(version, versions, output, site_url, used):
    """Copy a version into the site, without its shared files.

    `used` collects the shared names that the pages reference.
    """
    for path in sorted(version.source.rglob("*")):
        if not path.is_file():
            continue
        relative = path.relative_to(version.source).as_posix()
        suffix = path.suffix.lower()
        if suffix in MEDIA_SUFFIXES or suffix in DROPPED_SUFFIXES:
            continue
        page = f"{version.name}/{relative}"
        destination = output / page
        destination.parent.mkdir(parents=True, exist_ok=True)
        if suffix == ".html":
            text = rewrite_html(path.read_text(encoding="utf-8"), page, version, versions, site_url)
            destination.write_text(minify(text), encoding="utf-8")
            used.update(USED_MEDIA.findall(text))
        elif suffix == ".css":
            text = rewrite_css(path.read_text(encoding="utf-8"), page, version)
            destination.write_text(text, encoding="utf-8")
            used.update(USED_MEDIA.findall(text))
        elif relative == "sitemap.xml":
            text = path.read_text(encoding="utf-8")
            text = SITEMAP_LOCATION.sub(
                lambda m: m.group(1)
                + canonical_address(m.group(2), version, site_url)
                + m.group(3),
                text,
            )
            destination.write_text(text, encoding="utf-8")
            with gzip.open(destination.with_suffix(".xml.gz"), "wt", encoding="utf-8") as archive:
                archive.write(text)
        elif relative == "sitemap.xml.gz":
            continue  # Written again from sitemap.xml, with the new addresses.
        else:
            shutil.copyfile(path, destination)


def write_root_files(output, versions, default_version, releases_url):
    """Write versions.json, the redirect to the default version, and the 404 page."""
    entries = [{"version": v.name, "title": v.name, "aliases": []} for v in versions]
    entries = reorder_entries(entries, default_version)
    (output / "versions.json").write_text(json.dumps(entries, indent=2) + "\n", encoding="utf-8")
    (output / "index.html").write_text(
        ROOT_REDIRECT.format(version=default_version), encoding="utf-8"
    )
    (output / "404.html").write_text(
        NOT_FOUND_PAGE.format(version=default_version, releases_url=releases_url),
        encoding="utf-8",
    )


def assemble(bundles, output, site_url, default_version, releases_url):
    """Build the site in `output` from the bundles, one directory per version.

    Return a summary of what was written.
    """
    if not site_url.endswith("/"):
        site_url += "/"
    output = Path(output)
    if output.exists():
        shutil.rmtree(output)
    media_directory = output / MEDIA_DIRECTORY
    media_directory.mkdir(parents=True)

    versions = [
        Version(path.name, path) for path in sorted(Path(bundles).iterdir()) if path.is_dir()
    ]
    if default_version not in [version.name for version in versions]:
        raise ValueError(f"The default version {default_version} has no bundle")

    stored = {}
    used = set()
    # Every version is read first, because a page can point at a file of
    # another version.
    for version in versions:
        collect_media(version, media_directory, stored)
    by_name = {version.name: version for version in versions}
    for version in versions:
        copy_version(version, by_name, output, site_url, used)

    # A shared file that no page references cannot be reached, so it is removed.
    unused = sorted(set(stored) - used)
    for name in unused:
        (media_directory / name).unlink()

    write_root_files(output, versions, default_version, releases_url)
    return {
        "versions": [version.name for version in versions],
        "media_files": len(stored) - len(unused),
        "media_bytes": sum(size for name, size in stored.items() if name in used),
        "unused_media_files": len(unused),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--bundles", required=True, help="directory with one built site per version"
    )
    parser.add_argument("--output", required=True, help="directory to write the site to")
    parser.add_argument("--site-url", required=True, help="address of the site root")
    parser.add_argument("--default-version", default="main", help="version the site root opens")
    parser.add_argument("--releases-url", required=True, help="address of the release downloads")
    args = parser.parse_args()

    summary = assemble(
        args.bundles, args.output, args.site_url, args.default_version, args.releases_url
    )
    print(f"Versions: {', '.join(summary['versions'])}")
    print(
        f"Shared files: {summary['media_files']} ({summary['media_bytes'] / 1e6:.1f} MB), "
        f"{summary['unused_media_files']} unused files left out"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
