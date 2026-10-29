import os  # for relpath, cannot use pathlib
from pathlib import Path
from pathlib import PurePosixPath
import re
import sys
import types
import urllib

import jinja2

import yaml

# Zensical runs this module again for every page. Keep the cache of the page
# titles in a module that stays in sys.modules between the pages.
_cache = sys.modules.setdefault(
    "autoware_documentation_cache", types.ModuleType("autoware_documentation_cache")
)
if not hasattr(_cache, "titles"):
    _cache.titles = {}


def _page_url(src):
    path = PurePosixPath(src)
    parent = "" if str(path.parent) == "." else f"{path.parent}/"
    return parent if path.stem == "index" else f"{parent}{path.stem}/"


def _sort_key(src):
    # The order of the default navigation of MkDocs: the files of a directory
    # first, the index page before the other files, then the subdirectories.
    parts = PurePosixPath(src).parts
    return (parts[:-1], PurePosixPath(src).stem != "index", parts[-1])


def _nav_title(file):
    # A ".nav.yml" entry such as "Humble to Jazzy: humble-jazzy.md" sets the title.
    nav_file = file.parent / ".nav.yml"
    if not nav_file.exists():
        return None
    entries = list((yaml.safe_load(nav_file.read_text(encoding="utf-8")) or {}).get("nav") or [])
    while entries:
        entry = entries.pop()
        if isinstance(entry, dict):
            for title, value in entry.items():
                if isinstance(value, list):
                    entries.extend(value)
                elif value == file.name:
                    return title
    return None


def _page_title(docs_dir, src):
    file = Path(docs_dir, src)
    mtime = file.stat().st_mtime
    cached = _cache.titles.get(src)
    if cached and cached[0] == mtime:
        return cached[1]
    text = file.read_text(encoding="utf-8")
    title = _nav_title(file)
    meta = {}
    match = re.match(r"---\n(.*?)\n---\n", text, re.DOTALL)
    if match:
        meta = yaml.safe_load(match.group(1)) or {}
        text = text[match.end() :]
    if title is None and meta.get("title") is not None:
        title = str(meta["title"])
    if title is None:
        heading = re.search(
            r"^# (.+)$",
            re.sub(r"^```.*?^```", "", text, flags=re.DOTALL | re.MULTILINE),
            re.MULTILINE,
        )
        title = (
            heading.group(1).strip()
            if heading
            else PurePosixPath(src).stem.replace("-", " ").replace("_", " ").capitalize()
        )
    if "{{" in title:
        title = jinja2.Template(title).render(meta)
    _cache.titles[src] = (mtime, title)
    return title


def _sources(docs_dir):
    # Sort the pages again only if a file was added, removed, or renamed,
    # because each of these changes the modification time of its directory.
    directories = list(os.walk(docs_dir))
    signature = tuple((root, os.stat(root).st_mtime_ns) for root, _, _ in directories)
    cached = getattr(_cache, "sources", None)
    if cached and cached[0] == signature:
        return cached[1], cached[2]
    sources = sorted(
        (
            Path(root, name).relative_to(docs_dir).as_posix()
            for root, _, files in directories
            for name in files
            if name.endswith(".md")
        ),
        key=_sort_key,
    )
    positions = {src: index for index, src in enumerate(sources)}
    _cache.sources = (signature, sources, positions)
    return sources, positions


def _current_page(env):
    # Zensical 0.0.68 and later give the page to define_env. Zensical 0.0.67
    # does not, so read it from the context extension of the page.
    page = getattr(env, "page", None)
    if page is None:
        from zensical.config import get_config

        for extension in get_config().get("markdown_extensions", []):
            if type(extension).__name__ == "ContextExtension":
                page = extension._kwargs.get("page")
    return page


def _add_default_neighbors(env, page):
    # A page that the .nav.yml files leave out of the navigation has no
    # previous and next page. Under MkDocs, such a page kept the neighbors of
    # the default navigation, which includes all pages. Give the neighbors to
    # overrides/main.html, which shows them only if the page has none.
    docs_dir = Path(env.conf.get("root_dir", "."), env.conf.get("docs_dir", "docs")).resolve()
    sources, positions = _sources(docs_dir)
    index = positions.get(page.path)
    if index is None:
        return
    for key, neighbor in (("default_previous_page", index - 1), ("default_next_page", index + 1)):
        if 0 <= neighbor < len(sources):
            src = sources[neighbor]
            page.meta[key] = {"url": _page_url(src), "title": _page_title(docs_dir, src)}


def _relative_link(page, text, root_path):
    path = os.path.relpath(root_path, os.path.dirname(page.path))
    return f"[{text}]({path})"


def define_env(env):
    page = _current_page(env)
    if page is not None:
        _add_default_neighbors(env, page)

    @env.filter
    def drawio(image_path):
        image_url = urllib.parse.quote(f"{env.conf['site_url'].rstrip('/')}{image_path}", "")
        return f"https://app.diagrams.net/?lightbox=1#U{image_url}"

    # The macros read the page and the variables from the render context,
    # because Zensical 0.0.67 does not give them to define_env.
    @env.macro
    @jinja2.pass_context
    def create_relative_link(context, text, root_path):
        return _relative_link(context["page"], text, root_path)

    @env.macro
    @jinja2.pass_context
    def link_ad_api(context, name):
        return _relative_link(
            context["page"],
            name,
            f"design/autoware-architecture-v1/interfaces/ad-api/list/{name}.md",
        )

    @env.macro
    @jinja2.pass_context
    def resolve_msg_field(context, msg_type, name, ext):
        specs = context["autoware_interfaces"]["types"]
        for field in name.split("."):
            msg_type = msg_type.split("[")[0]
            msg_type = specs[msg_type][ext][field]
            ext = "msg"
        return msg_type
