# Fixing dependent package versions

Autoware manages dependent package versions in `repositories/autoware.repos`.
For example, let's say you make a branch in autoware_universe and add new features.
Suppose you update other dependencies with `vcs pull` after cutting a branch from autoware_universe. Then the version of autoware_universe you are developing and other dependencies will become inconsistent, and the entire Autoware build will fail.
We recommend saving the dependent package versions when you start the development, as described in [Save the versions of your workspace](#save-the-versions-of-your-workspace).

This page explains how Autoware pins the versions, and how you save and restore the versions of your own workspace. For a description of all `.repos` files, see [`repositories/*.repos` files](../../design/repos-files.md).

## How Autoware pins the versions

Each entry of a `.repos` file names a repository, its URL, and a `version`. The `version` is a tag, a branch, or a commit hash. In the `2.0.0` tag of the `autoware` repository, `repositories/autoware.repos` pins most repositories to a release tag, for example:

```yaml
repositories:
  core/autoware_core:
    type: git
    url: https://github.com/autowarefoundation/autoware_core.git
    version: 1.10.0
  universe/autoware_universe:
    type: git
    url: https://github.com/autowarefoundation/autoware_universe.git
    version: 0.53.1
  launcher/autoware_launch:
    type: git
    url: https://github.com/autowarefoundation/autoware_launch.git
    version: 0.53.0
```

Some external repositories, for example `universe/external/muSSP`, use a commit hash. As a result, `vcs import src < repositories/autoware.repos` gives the same source code on every computer.

The `autoware-nightly.repos` file sets some of these repositories to their `main` branch. A branch is not a fixed version. Each `vcs import` or `vcs pull` can move it to a newer commit.

## Save the versions of your workspace

Before you start to develop, save the exact state of all repositories in `src/`:

```bash
vcs export src --exact > repositories/my_autoware.repos
```

The `--exact` option writes the commit hash of each repository, also for the repositories that are on a branch. If you want tag names where they exist, use `--exact-with-tags`. This option writes the tag of a commit when the commit has exactly one tag, and the commit hash in all other cases.

```bash
vcs export src --exact-with-tags > repositories/my_autoware.repos
```

Commit the file to your fork of the `autoware` repository, or keep it with your branch. Then other developers can build the same workspace.

## Restore the versions

To restore the saved versions, import the file:

```bash
vcs import src < repositories/my_autoware.repos
```

Then install the dependencies again and rebuild the workspace, as on the [source installation](../../installation/autoware/source-installation.md) page:

```bash
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Check the state of the workspace

To see which repositories have local changes or are not on the saved version, use these commands:

```bash
# Show the working tree status of each repository
vcs status src

# Show the last commit of each repository
vcs log -l 1 src
```

If a repository has changes that you did not commit, `vcs import` can fail to check out the saved version of that repository. Commit or stash the changes in that repository first.
