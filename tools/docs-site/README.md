# Documentation site assembly

The documentation site holds one directory per version: the default branch `main`, every release such as `1.9.0`, and a preview such as `pr-812` for every open pull request from this repository. Each version is a complete site that Zensical built. Most of their images are the same, so the scripts here store each image only once for all versions. This keeps the site far below the 1 GB limit of GitHub Pages.

The logic lives in Python with unit tests, so that you can read and test it without the workflows.

## How the site is published

1. The `deploy-docs` workflow builds the site of the default branch and of every pull request. It keeps each site as an artifact, named `docs-main` or `docs-pr-<number>`.
2. The `github-release` workflow builds the site of every release tag. It attaches the site to the release as `autoware-docs-site.tar.gz`.
3. The `publish-docs` workflow runs after each of these builds, after a pull request closes, and once a day. It runs these scripts and deploys the result to GitHub Pages. Only one run publishes at a time. A newer run that waits replaces an older one that waits, because it publishes the newer state anyway. A separate job comments the preview address on the pull request, so that a replaced run never loses its comment. Dependabot pull requests get no preview and no comment.

A pull request from a fork keeps its site as an artifact only. The `comment-docs-artifact` workflow links that artifact on the pull request. Dependabot pull requests get no preview.

## Scripts

- `collect_bundles.py` downloads one built site per version: every release archive, the newest `docs-main` artifact, and the newest artifact of every open pull request from this repository. It keeps the release archives in a cache directory, because they never change.
- `assemble.py` puts the versions together:
  - Every image, diagram, and PDF moves to the `_media` directory, named after a hash of its content. The script rewrites the references in the HTML and CSS of each version. A file that no page references is left out.
  - The HTML is minified.
  - Each page gets a canonical address with the name of its version, and so does each `sitemap.xml`. Each sitemap also lists the root of its version, because the version selector keeps the current page only if the sitemap of the target version lists that root.
  - The site root gets `versions.json` for the version selector, an `index.html` that redirects to the default version, and a `404.html`.
- `verify.py` compares every page of the assembled site with the page in its built site: the text, the content of each `pre` element, and what each reference leads to. A reference to an image must lead to a file with the same content. The `publish-docs` workflow stops before the deployment if a page differs. With `--state`, it keeps a fingerprint of each version that passed, and does not check that version again until its built site, its assembled pages, the code or packages of the tools, the Python version, or the site address change. It also keeps every path outside the version that the check read, for example a diagram of `main` that an older release links to, and checks the version again when such a path changes. Releases never change, so usually only `main` and the previews are checked.
- `versions.py` reads version names and puts `versions.json` in the order of the version selector: the default branch first, then the releases from newest to oldest, then the pull request previews.
- `pr_comment.py` writes the comment that links a pull request to its preview and to the pages it changes.

## Usage

Assemble and check a site from built sites on your computer. Put each site in a directory named after its version, for example `bundles/main` and `bundles/1.9.0`.

```bash
pip3 install -r tools/docs-site/requirements.txt
cd tools/docs-site
python3 assemble.py --bundles ../../bundles --output ../../public \
  --site-url https://autowarefoundation.github.io/autoware-documentation/ \
  --releases-url https://github.com/autowarefoundation/autoware-documentation/releases
python3 verify.py --bundles ../../bundles --site ../../public \
  --site-url https://autowarefoundation.github.io/autoware-documentation/
cd ../../public && python3 -m http.server 8000
```

Then open <http://localhost:8000/>.

## Tests

The `test-tools` workflow runs the tests on every pull request that changes `tools/`.

```bash
pip3 install pytest -r tools/docs-site/requirements.txt
pytest tools/docs-site -v
```
