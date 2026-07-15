# Autoware Index

The [Autoware Index](https://autowarefoundation.github.io/autoware-index/) is a registry of community ROS 2 packages that extend Autoware.
It records which packages exist, where their source lives, and whether each one builds and passes tests against the latest Autoware release.

A regular [source installation](source-installation.md) does not require the Autoware Index.
Use it when you want community or extra packages that are not part of the default workspace:
you select packages from the registry, generate a `.repos` file from your selection, and import it with `vcs import` like any other [repos file](../../design/repos-files.md).

There are two ways to generate the file:

- [`aw-index-cli`](https://github.com/autowarefoundation/aw-index-cli): a command line tool that composes it from the registry.
- The [browse site](https://autowarefoundation.github.io/autoware-index/): its **Repos builder** composes and downloads the same file in your browser.

## Using `aw-index-cli`

1. Install the CLI with [pipx](https://pipx.pypa.io/).

   ```bash
   pipx install aw-index-cli   # first install
   pipx upgrade aw-index-cli   # update to the latest release
   ```

2. Compose the repos file from your Autoware workspace root (the cloned `autoware` directory).
   By default, it writes your selection to `repositories/autoware-index.repos`.

   ```bash
   cd autoware
   aw-index-cli compose --rosdistro jazzy --packages <package_name>
   ```

   You can also select whole repositories by their registry key, or filter by tags:

   ```bash
   # Pull in a whole repository entry by its registry key.
   aw-index-cli compose --rosdistro jazzy --repository <repository_name>

   # Select by tags (filters are ANDed).
   aw-index-cli compose --rosdistro jazzy --tags sensing perception
   ```

   Use the `--rosdistro` value that matches your ROS 2 distribution.
   The available distributions are listed in the [autoware-index](https://github.com/autowarefoundation/autoware-index/tree/main/distributions) repository.

3. Import the repositories and build your workspace as usual.

   ```bash
   vcs import src < repositories/autoware-index.repos
   rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
   colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
   ```

   > ⚠️ An imported repository may also contain packages that are not registered in the index.
   > To build only what you selected, scope the build with `colcon build --packages-up-to <package_name>`.

!!! note

    `autoware-index.repos` is a generated file.
    Re-run `aw-index-cli compose` to update it, and do not edit it by hand.
    Since the selection is user-specific, the file is ignored by git in the `autoware` repository.

### Checking your workspace

`aw-index-cli check` compares a composed `autoware-index.repos` file against the registry and the latest validation results.
Run it from the workspace root; it discovers the file automatically.

```bash
aw-index-cli check
```

It exits with `0` when all selected packages pass validation and match the registry, and with `1` when a package fails validation, has drifted from the registry, or was removed.
This makes it suitable as a CI gate after `vcs import`.

### Listing packages

To see the registered packages and their latest validation status from the terminal:

```bash
aw-index-cli list --rosdistro jazzy
```

## Using the browse site

If you prefer not to install anything:

1. Pick your packages on the [browse site](https://autowarefoundation.github.io/autoware-index/).
2. Open the **Repos builder** panel and download your selection as `autoware-index.repos`.
3. Move the file into the `repositories` directory of your workspace and import it:

   ```bash
   cd autoware
   mv ~/Downloads/autoware-index.repos repositories/
   vcs import src < repositories/autoware-index.repos
   ```

## Registering your packages

To make your own packages available through the index, register them in the [autoware-index](https://github.com/autowarefoundation/autoware-index) repository:

- Use the [guided register page](https://autowarefoundation.github.io/autoware-index/register.html).
  It writes the registry entry for you, runs the same checks as the pull request gate, and opens the pull request on your behalf.
- Or register by hand: fork the repository, add one entry under `repositories:` in `distributions/<distro>.yaml` for each distribution you support, and open a pull request.

After a registration is merged, CI builds and tests each registered package against the latest Autoware release and publishes the results on the browse site.
Packages registered with a `branch` ref are re-validated nightly, while `tag` and `sha` refs stay pinned.

For the registry format, tag vocabulary, and validation rules, refer to the [contribution guide](https://github.com/autowarefoundation/autoware-index/blob/main/CONTRIBUTING.md).
