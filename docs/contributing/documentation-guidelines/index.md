# Documentation guidelines

## Workflow

Contributions to Autoware's documentation are welcome, and the same principles [described in the contribution guidelines](../index.md#pull-requests) should be followed. Small, limited changes can be made by forking this repository and submitting a pull request, but larger changes should be discussed with the community and Autoware maintainers via GitHub Discussion first.

Examples of small changes include:

- Fixing spelling or grammatical mistakes
- Fixing broken links
- Making an addition to an existing, well-defined page, such as the [Troubleshooting](../../community/support/troubleshooting/index.md) guide.

Examples of larger changes include:

- Adding new pages with a large amount of detail, such as a tutorial
- Re-organization of the existing documentation structure

## Style guide

You should refer to the [Google developer documentation style guide](https://developers.google.com/style) as much as possible. Reading the [Highlights page](https://developers.google.com/style/highlights) of that guide is recommended, but if not then the key points below should be noted.

- [Use standard American English spelling](https://developers.google.com/style/spelling) and punctuation.
- [Use sentence case](https://developers.google.com/style/capitalization) for document titles and section headings.
- [Use descriptive link text](https://developers.google.com/style/link-text).
- [Write short sentences](https://developers.google.com/style/translation#write-short,-clear,-and-precise-sentences) that are easy to understand and translate.

## Tips

### How to preview your modification

There are two ways to preview your modification on a documentation website.

#### 1. Using GitHub Actions workflow

Follow the steps below.

1. Create a pull request to the repository.
2. Wait for the `deploy-docs-pr` check to finish. What happens next depends on where your branch is.
   - If your branch is in the repository itself, the `github-actions` bot adds a comment with the URL of the preview.
   - If your branch is in a fork, the site is built but not published. The `github-actions` bot adds a comment with a link to download the site, and the summary of the run explains how to view it on your computer.

If this is your first pull request to the repository, a maintainer must approve the workflows before they run.

#### 2. Running a local server

You can also build the documentation website on your local computer.
This site uses [Zensical](https://zensical.org/).
Most other Autoware repositories still use [MkDocs](https://www.mkdocs.org/).

To preview this site, install the dependencies of this repository in a virtual environment.

```bash
cd /PATH/TO/YOUR-autoware-documentation
python3 -m venv .venv
source .venv/bin/activate
pip install -r mkdocs-requirements.txt
```

Then start the server.

```bash
zensical serve
```

Open <http://127.0.0.1:8000/autoware-documentation/> to see the preview of the website.

To preview the documentation of another Autoware repository, install the shared MkDocs dependencies.

```bash
python3 -m pip install -U $(curl -fsSL https://raw.githubusercontent.com/autowarefoundation/autoware-github-actions/main/deploy-docs/mkdocs-requirements.txt)
```

Then run `mkdocs serve` in the directory of that repository, and open <http://127.0.0.1:8000/>.
