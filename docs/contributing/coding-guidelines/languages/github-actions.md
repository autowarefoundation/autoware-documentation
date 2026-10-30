<!-- cspell: ignore endraw -->

# GitHub Actions

This page describes how to write GitHub Actions workflows in Autoware repositories. pre-commit formats and checks the workflow files with Prettier and yamllint, like all other YAML files.

## References

- [Workflow syntax for GitHub Actions](https://docs.github.com/en/actions/reference/workflows-and-actions/workflow-syntax)
- [Secure use reference](https://docs.github.com/en/actions/reference/security/secure-use)
- [autowarefoundation/autoware-github-actions](https://github.com/autowarefoundation/autoware-github-actions)
- [autowarefoundation/sync-file-templates](https://github.com/autowarefoundation/sync-file-templates)

## Do not edit synced workflows

Many workflows are the same in all Autoware repositories, for example `pre-commit`, `semantic-pull-request`, `spell-check-differential`, and `sync-files`. The `sync-files` workflow copies them from [sync-file-templates](https://github.com/autowarefoundation/sync-file-templates/tree/main/sources/.github/workflows). The list of the synced files of a repository is in its `.github/sync-files.yaml`. A synced file starts with this comment:

```yaml
# This file is automatically synced from:
# https://github.com/autowarefoundation/sync-file-templates
# To make changes, update the source repository and follow the guidelines in its README.
```

Do not change a synced file in your repository, because the next sync overwrites it. Send the change to `sync-file-templates`.

## Name the files

- Put workflows in `.github/workflows/` and use the `.yaml` extension.
- Give the workflow the same `name` as the file, for example `name: pre-commit` in `pre-commit.yaml`. Then the name of a check on a pull request tells you which file to open.
- Use a name that says what the workflow does. Add `-differential` to a workflow that checks only the changed packages or files, as in `build-and-test-differential` and `spell-check-differential`.

## Reuse actions and workflows

Do not copy the same steps into many workflows. Use the composite actions and the reusable workflows of [autoware-github-actions](https://github.com/autowarefoundation/autoware-github-actions), for example `colcon-build`, `colcon-test`, `get-modified-packages`, `pre-commit`, and `spell-check`. Refer to them with the `v1` tag:

```yaml
- name: Get modified packages
  id: get-modified-packages
  uses: autowarefoundation/autoware-github-actions/get-modified-packages@v1
```

The `v1` tag moves to each new release of `autoware-github-actions`. If a step is useful for more than one repository, add it to `autoware-github-actions` as a new action.

A step that only one repository uses can be a local composite action in `.github/actions/`. For example, the `autoware` repository has `.github/actions/free-disk-space`.

## Write the workflow

This is the `spell-check-differential` workflow of the `autoware` repository 2.0.0. It shows the main conventions:

{% raw %}

```yaml
name: spell-check-differential

on:
  pull_request:

concurrency:
  group: ${{ github.workflow }}-${{ github.event_name }}-${{ github.ref }}
  cancel-in-progress: true

jobs:
  spell-check-differential:
    runs-on: ubuntu-24.04
    steps:
      - name: Check out repository
        uses: actions/checkout@v7

      - name: Run spell-check
        uses: autowarefoundation/autoware-github-actions/spell-check@v1
        with:
          cspell-json-url: https://raw.githubusercontent.com/autowarefoundation/autoware-spell-check-dict/main/.cspell.json
          dict-packages: |
            https://github.com/autowarefoundation/autoware-spell-check-dict
            https://github.com/tier4/cspell-dicts
```

{% endraw %}

- Set a `concurrency` group with `cancel-in-progress: true` for pull request workflows. Then a new push cancels the run for the old commit.
- Use a fixed runner version, such as `ubuntu-24.04`, not `ubuntu-latest`. Then a new runner image does not change the build without a pull request.
- Give each step a `name` that starts with a verb, for example `Check out repository` or `Run spell-check`.
- Refer to third-party actions by their major version tag, for example `actions/checkout@v7`. Dependabot updates these tags. The `dependabot.yaml` of `sync-file-templates` checks the `github-actions` ecosystem every month.

## Formatting

yamllint checks the workflow files with the [`.yamllint.yaml`](https://github.com/autowarefoundation/sync-file-templates/blob/main/sources/.yamllint.yaml) of the repository:

- Do not start the file with `---`.
- Quote strings only when necessary.
- Put at least one space between the code and a `#` comment.
- The `truthy` rule does not check keys, so `on:` is allowed.

Prettier formats the files. It uses two spaces of indentation.

## Security

- Give the `GITHUB_TOKEN` only the permissions that the job needs. Use the `permissions` key.
- Do not run the code of a pull request in a workflow that has access to secrets. A workflow that runs on `pull_request_target` or `workflow_run` has access to the secrets of the repository. Do not check out and build the code of the pull request in such a workflow.
- Do not put untrusted input, such as the title of a pull request, directly into a `run` script with {% raw %}`${{ }}`{% endraw %}. Pass it in an environment variable instead.
- To push commits or open pull requests from a workflow, Autoware workflows use a GitHub App token, not a personal access token. See the `pre-commit` workflow for an example that uses the `APP_ID` and `PRIVATE_KEY` secrets.
