# Documentation guidelines

## Contribution guidelines on Autoware Documentation

Contributions to Autoware's documentation are welcome, and the same principles [described in the contribution guidelines](../index.md#pull-requests) should be followed. Small, limited changes can be made by forking this repository and submitting a pull request, but larger changes should be discussed with the community and Autoware maintainers via GitHub Discussion first.

Examples of small changes include:

- Fixing spelling or grammatical mistakes
- Fixing broken links
- Making an addition to an existing, well-defined page, such as the [Troubleshooting](../../support/troubleshooting.md) guide.

Examples of larger changes include:

- Adding new pages with a large amount of detail, such as a tutorial
- Re-organization of the existing documentation structure

In terms of style, you should refer to the [Google developer documentation style guide](https://developers.google.com/style) as much as possible. Reading the [Highlights page](https://developers.google.com/style/highlights) of that guide is recommended, but if not then the key points below should be noted.

- [Use standard American English spelling](https://developers.google.com/style/spelling) and punctuation.
- [Use sentence case](https://developers.google.com/style/capitalization) for document titles and section headings.
- [Use descriptive link text](https://developers.google.com/style/link-text).
- [Write short sentences](https://developers.google.com/style/translation#write-short,-clear,-and-precise-sentences) that are easy to understand and translate.

## How to preview your modification on Autoware Documentation

There are two ways to preview your modification on Autoware Documentation.

### 1. Use Github Actions workflow in `autowarefoundation/autoware-documentation` repository

You can use Github Actions in `autowarefoundation/autoware-documentation` repository to deploy your branch.

1. Create a PR of your branch to the repository.
2. Add a `documentation` label from the sidebar.
3. Wait for a couple of minutes, and the `github-actions` will notify the URL for your branch's preview.

### 2. Run mkdocs in your local environment

Instead of creating a PR, you can use `mkdocs` to build the Autoware Documentation website on your local computer.
Assuming that you are using Ubuntu OS, run the following to install required libraries.

```
sudo apt install -y mkdocs
git clone git@github.com:autowarefoundation/autoware-github-actions.git /tmp/autoware-github-actions
cd /tmp/autoware-github-actions/deploy-docs
pip install -U -r mkdocs-requirements.txt
```

Then, run `mkdocs serve` on your Autoware Documentation directory.

```
cd /PATH/TO/autoware-documentation
mkdocs serve
```

It will launch mkdocs server. Access [http://127.0.0.1:8000/autoware-documentation/](http://127.0.0.1:8000/autoware-documentation/) to see the preview of Autoware Documentation.
