# Troubleshooting

## How to resolve errors of the setup script?

### CUDA-related errors

When installing CUDA, it may cause errors because of version conflicts.
To resolve this, try either:

- Uninstall all CUDA-related libraries and rerun the setup script.

  ```bash
  sudo apt purge        \
    "cuda*"             \
    "libcudnn*"         \
    "libnvinfer*"       \
    "libnvonnxparsers*" \
    "libnvparsers*"     \
    "nvidia*"

  sudo apt autoremove

  ./setup-dev-env.sh
  ```

!!! warning

    Note that this may break your system and run carefully.

- Run the setup script without installing CUDA-related libraries.

  ```bash
  ./setup-dev-env.sh --no-nvidia
  ```

!!! warning

    Note that some components in `autoware.universe` don't work without CUDA.

    Also, although Autoware could work with different versions of CUDA libraries as well,
    remember it is not officially supported and may sometimes be broken.

## How to resolve build errors?

When you get build errors, follow the instruction below depending on the version you use.

### In case you use the latest version

Note that the software may sometimes be broken if you follow the latest version.
Most issues are due to out-of-date software or old build files.

To resolve the problem, try cleaning your build artifacts and rebuilding first:

```bash
rm -rf build/ install/ log/
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

If the error is not resolved, remove `src/` and update the workspace depending on your installation type:

- [Docker](../../installation/autoware/docker-installation/#how-to-update-a-workspace)
- [Source](../../installation/autoware/source-installation/#how-to-update-a-workspace)

!!! Warning

    Before removing `src/`, confirm that there are no necessary modifications left in your local environment.

If the error still persists, delete the entire workspace and restart from cloning the repository.

```bash
rm -rf autoware/
git clone https://github.com/autowarefoundation/autoware.git
```

### In case you use a fixed version

It is unusual that an error occurs when you use a fixed version.

Possible causes are:

- ROS 2 has been updated with breaking changes.
  - Check out [Packaging and Release Management](https://discourse.ros.org/c/release/16) tag on ROS Discourse to confirm that.
- Your local environment is broken.
  - Confirm your `.bashrc` file, environment variables, and library versions.

Also, there are two common cases that you misunderstand you use a fixed version.

1. You used a fixed version only for `autowarefoundation/autoware`.
   If you specify a hash of `autowarefoundation/autoware`, it doesn't mean the version of the entire workspace is fixed.
   You have to fix the versions in the `.repos` file to use a completely fixed version.

2. You didn't update the workspace after you changed the branch of `autowarefoundation/autoware`.
   If you change the branch of `autowarefoundation/autoware`, it doesn't affect the files under `src/`.
   You have to run the `vcs import` command to update them.
