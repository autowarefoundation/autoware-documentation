# CI checks

Autoware has several checks for a pull request.
The results are shown at the bottom of the pull request page as below.

![ci-checks](images/ci-checks.png)

If the ❌ mark is shown, click the `Details` button and investigate the failure reason.

If the `Required` mark is shown, you cannot merge the pull request unless you resolve the error.
If not, it is optional, but preferably it should be fixed.

The following sections explain about common CI checks in Autoware.  
Note that some repositories may have different settings.

## DCO

The Developer Certificate of Origin (DCO) is a lightweight way for contributors to certify that they wrote or otherwise have the right to submit the code they are contributing to the project.

This workflow checks whether the pull request fulfills `DCO`.  
You need to confirm the [required items](https://developercertificate.org/) and commit with `git commit -s`.

For more information, refer to the [GitHub App page](https://github.com/apps/dco).

## semantic-pull-request

This workflow checks whether the pull request follows [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/).

For the detailed rules, see the [pull request rules](index.md#pull-request-rules).

## pre-commit

[pre-commit](https://pre-commit.com/) is a tool to run formatters or linters when you commit.

This workflow checks whether the pull request has no error with `pre-commit`.

In the workflow `pre-commit.ci - pr` is enabled in the repository, it will automatically fix errors by [pre-commit.ci](https://pre-commit.ci/) as many as possible.  
If there are some errors remain, fix them manually.

You can run `pre-commit` in your local environment by the following command:

```bash
pre-commit run -a
```

Or you can install `pre-commit` to the repository and automatically run it before committing:

```bash
pre-commit install
```

Since it is difficult to detect errors with no false positives, some jobs are split into another config file and marked as optional.  
To check them, use the `--config` option:

```bash
pre-commit run -a --config .pre-commit-config-optional.yaml
```

### Matching your local clang-format to pre-commit

Each repository pins its clang-format version in `.pre-commit-config.yaml`, as the `rev` of the `mirrors-clang-format` hook:

```yaml
- repo: https://github.com/pre-commit/mirrors-clang-format
  rev: v22.1.5
```

`pre-commit` installs that exact version in its own environment.
So `pre-commit run -a` gives the same result as CI, whatever clang-format you have installed.

Manual runs of `clang-format` use the binary on your `PATH`.
Editors differ:

- VS Code with the C/C++ extension uses the `clang-format` on your `PATH`, unless its bundled one is newer.
  To always use yours, set `C_Cpp.clang_format_path`.
- CLion uses its bundled `clang-format`, unless you set a custom path in **Settings | Editor | Code Style | C/C++**.
- clangd has the formatter built in, so its output follows the version of clangd, not of `clang-format`.

If the version that your editor uses differs from the pin, your editor may format code differently from CI.
The Ubuntu archives are often behind the pin: Ubuntu 22.04 ships at most `clang-format-15`, and Ubuntu 24.04 at most `clang-format-20`.

Repositories can pin different versions.
A system binary can match only one of them, so match the repository that you work on most.

In the commands below, set `CLANG_FORMAT_VERSION` to the `rev` of that repository without the leading `v`, for pipx.
For apt.llvm.org, set `CLANG_FORMAT_MAJOR` to the major version of that `rev`.

#### From pipx, for an exact match with CI

This installs the same Python wheel that the pre-commit hook uses.
It works on Ubuntu 22.04 and 24.04, on amd64 and arm64.

```bash
CLANG_FORMAT_VERSION=22.1.5

sudo apt-get install -y pipx
pipx ensurepath # adds ~/.local/bin to PATH in new shells
pipx install "clang-format==${CLANG_FORMAT_VERSION}"
```

Open a new shell, then check the version:

```bash
clang-format --version
```

To change the version later, run `pipx install --force "clang-format==${CLANG_FORMAT_VERSION}"`.
Do not run `pipx upgrade clang-format`: it moves to the latest release, which may not be the pin.
On Ubuntu 24.04, `pip install --user` fails with `externally-managed-environment` (PEP 668), so use pipx or a virtual environment.

#### From apt.llvm.org

[apt.llvm.org](https://apt.llvm.org/) has a package for each major version.
It keeps only the latest snapshot of that major version, so you cannot install a specific patch release.
The output usually matches CI, but this is not guaranteed.
Use pipx if you need an exact match.

```bash
CLANG_FORMAT_MAJOR=22
. /etc/os-release # sets UBUNTU_CODENAME to jammy or noble

sudo apt-get update
sudo apt-get install -y wget gnupg ca-certificates

sudo install -d -m 0755 /etc/apt/keyrings
wget -qO- https://apt.llvm.org/llvm-snapshot.gpg.key \
  | sudo tee /etc/apt/keyrings/apt.llvm.org.asc > /dev/null

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/apt.llvm.org.asc] https://apt.llvm.org/${UBUNTU_CODENAME}/ llvm-toolchain-${UBUNTU_CODENAME}-${CLANG_FORMAT_MAJOR} main" \
  | sudo tee /etc/apt/sources.list.d/apt.llvm.org-${CLANG_FORMAT_MAJOR}.list > /dev/null

sudo apt-get update
sudo apt-get install -y clang-format-${CLANG_FORMAT_MAJOR}
```

Optionally, verify the signing key:

```bash
gpg --show-keys --with-fingerprint /etc/apt/keyrings/apt.llvm.org.asc
# expected: 6084 F3CF 814B 57C1 CF12  EFD5 15CF 4D18 AF4F 7421
```

The package installs only the versioned binaries, such as `/usr/bin/clang-format-22`.
Manual runs and VS Code look for a plain `clang-format`, so register it:

```bash
sudo update-alternatives --install /usr/bin/clang-format clang-format /usr/bin/clang-format-${CLANG_FORMAT_MAJOR} ${CLANG_FORMAT_MAJOR}0
sudo update-alternatives --install /usr/bin/git-clang-format git-clang-format /usr/bin/git-clang-format-${CLANG_FORMAT_MAJOR} ${CLANG_FORMAT_MAJOR}0

clang-format --version
```

## spell-check-differential

This workflow detects spelling mistakes using [CSpell](https://github.com/streetsidesoftware/cspell) with [our dictionary file](https://github.com/tier4/autoware-spell-check-dict/blob/main/.cspell.json).
Since it is difficult to detect errors with no false positives, it is an optional workflow, but it is preferable to remove spelling mistakes as many as possible.

You have the following options if you need to use a word that is not registered in the dictionary.

- If the word is only used in a few files, you can use [inline document settings "cspell:ignore"](https://cspell.org/configuration/document-settings/) to suppress the check.
- If the word is widely used in the repository, you can create a local cspell json and pass it to the [spell-check action](https://github.com/autowarefoundation/autoware-github-actions/tree/main/spell-check).
- If the word is common and may be used in many repositories, you can submit pull requests to [tier4/autoware-spell-check-dict](https://github.com/tier4/autoware-spell-check-dict) or [tier4/cspell-dicts](https://github.com/tier4/cspell-dicts) to update the dictionary.

## build-and-test-differential

This workflow checks `colcon build` and `colcon test` for the pull request.  
To make the CI faster, it doesn't check all packages but only modified packages and the dependencies.

## build-and-test-differential-self-hosted

This workflow is the `ARM64` version of `build-and-test-differential`.  
You need to add the `ARM64` label to run this workflow.

For reference information, since ARM machines are not supported by GitHub-hosted runners, we use self-hosted runners prepared by the AWF.  
For the details about self-hosted runners, refer to [GitHub Docs](https://docs.github.com/en/actions/hosting-your-own-runners/about-self-hosted-runners).

## deploy-docs

This workflow deploys the preview documentation site for the pull request.  
You need to add the `deploy-docs` label to run this workflow.
