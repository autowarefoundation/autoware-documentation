# Documentation versions on the gh-pages branch

The site is versioned with [mike](https://github.com/jimporter/mike), which keeps one directory
per deployed version on the `gh-pages` branch and lists them in `versions.json`. A version is
named after whatever was deployed: the default branch `main`, a release tag such as `1.8.0`, a
pull request preview such as `pr-812`, or a branch preview such as `branch-my-feature`.

The scripts here are used by the workflows that keep that list tidy. The logic lives in Python
with unit tests rather than in the workflows, so that it can be read and tested on its own.

- `versions.py` reads version names, and decides both what to delete and how to order the list.
  It is used by the two scripts below.
- `delete_old_version_docs.py` deletes outdated release documentation. It is used by the
  `delete-old-version-docs` workflow.
- `reorder_versions.py` sorts the entries of `versions.json`. It is used by the
  `reorder-versions` workflow.

## What is kept

`delete_old_version_docs.py` keeps the newest minor release lines, and of each of those lines
only the newest patch. With three lines, a `gh-pages` branch holding `1.5.0`, `1.6.0`, `1.7.0`,
`1.7.1` and `1.8.0` keeps `1.8.0`, `1.7.1` and `1.6.0`. It deletes `1.5.0` because its line is
too old, and `1.7.0` because `1.7.1` replaces it. The number of lines the release workflow keeps
is the default of `.github/workflows/delete-old-version-docs.yaml`.

Only releases are ever deleted. The default branch stays forever, and the pull request and
branch previews are cleaned up by the `delete-closed-pr-docs` and `delete-removed-branch-docs`
workflows.

## What is ordered

mike rewrites `versions.json` in its own order every time it deploys or deletes, which puts the
previews above the releases in the version dropdown of the site. `reorder_versions.py` puts the
default branch first, then the releases from newest to oldest, then the pull request previews,
then the branch previews.

## Usage

Run the deletion script from the root of the repository, where `mkdocs.yaml` is, because mike
reads its configuration from there. Without `--dry-run` it deletes the outdated versions from
the `gh-pages` branch and pushes the result.

```bash
python3 tools/gh-pages-versions/delete_old_version_docs.py --dry-run
python3 tools/gh-pages-versions/delete_old_version_docs.py --keep-lines 3
```

`reorder_versions.py` rewrites a `versions.json` file in place. It needs a checkout of the
`gh-pages` branch, and writes the file the way mike does so that only the order changes.

```bash
python3 tools/gh-pages-versions/reorder_versions.py path/to/versions.json --default-branch main
```

## Tests

The `test-tools` workflow runs these on every pull request that touches `tools/`.

```bash
pip3 install pytest
pytest tools/gh-pages-versions -v
```
